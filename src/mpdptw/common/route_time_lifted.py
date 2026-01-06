from gurobipy import *
from mpdptw.common.big_M import tight_bigM


def Run_Time_Model(
    subset,
    inst,
    Time_Lim=False,
    Output=0,
    Time_Window=False,
    Capacity_Constraint=False,
    Threads=None,
    ENV=None,
):
    """
    Route-time pricing subproblem for a given subset of requests.

    Parameters
    ----------
    subset : iterable
        Collection of request ids to include in this subproblem.
    inst : dict
        Instance data dictionary containing graph, times, costs, etc.
    Time_Lim : bool
        If True, set a time limit proportional to |subset|.
    Output : int
        0 to silence Gurobi logs; non-zero to enable logging (and echo subset).
    Time_Window : bool
        If True, enforce time windows via big-M constraints on continuous times.
    Capacity_Constraint : bool
        If True, include load-flow arcs and capacity constraints.
    Threads : int or None
        If given, sets Gurobi "Threads" parameter.
    ENV : gurobipy.Env or None
        Optional pre-initialized Gurobi environment (e.g., for multiprocessing).

    Returns
    -------
    (model, s_cost, used_arcs, S_vals, runtime)
        model     : gurobipy.Model
        s_cost    : float or None
        used_arcs : list[(i, j)]
        S_vals    : dict[i -> S[i]] when Time_Window else None
        runtime   : float or None
    """
    # ------------------------- Model construction & params ------------------------- #
    if not ENV:
        model = Model("Route_Time")
    else:
        model = Model("Route_Time", env=ENV)

    if Output == 0:
        model.setParam("OutputFlag", 0)
    else:
        print("********************")
        print(subset)
        print("********************")
        model.setParam("OutputFlag", 1)

    if Threads is not None:
        model.setParam("Threads", Threads)
    if Time_Lim:
        model.setParam("TimeLimit", len(subset) * 3)
    model.setParam("Seed", 123)

    # ------------------------------- Instance data ------------------------------- #
    A_all = inst["A_feasible_ext"]
    Pr = inst["Pr"]
    Dr_single = inst["Dr_single"]
    e, l = inst["e"], inst["l"]
    d, q = inst["d"], inst["q"]
    t, c = inst["t_ext"], inst["c_ext"]
    depot = inst["depot"]
    sink = inst["sink"]
    Q = inst["Q"]

    # Requests in this subproblem
    R = subset
    Pickups = [i for r in R for i in Pr[r]]
    Dels = [Dr_single[r] for r in R]
    N = set(Pickups) | set(Dels)
    V = {depot, sink} | N

    # Induced subgraph 
    A_sub = [(i, j) for (i, j) in A_all if i in V and j in V and (i, j) in c and (i, j) in t]

    # Adjacency (restricted to A_sub)
    in_arcs = {v: [] for v in V}
    out_arcs = {v: [] for v in V}
    for (i, j) in A_sub:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))

    # Sanity: ensure no isolated customer nodes in the induced subgraph
    dead_in = [j for j in N if len(in_arcs[j]) == 0]
    dead_out = [i for i in N if len(out_arcs[i]) == 0]
    if dead_in or dead_out:
        raise ValueError(f"Subset yields isolated nodes — no-in={dead_in}, no-out={dead_out}")

    # Map node -> request 
    node_req = {}
    for r in R:
        for p in Pr[r]:
            node_req[p] = r
        node_req[Dr_single[r]] = r
    node_type = {"delivery": set(Dels), "pickup": set(Pickups)}

    # ------------------------------- Decision vars ------------------------------- #
    X = {(i, j): model.addVar(vtype=GRB.BINARY) for (i, j) in A_sub}

    # Cutoff based on sink's latest time
    model.Params.Cutoff = float(l[sink])

    # Objective: travel + service-at-origin
    model.setObjective(quicksum(X[i, j] * (c[i, j] + d.get(i, 0.0)) for (i, j) in A_sub), GRB.MINIMIZE)

    # -------------------------- Optional capacity constraints -------------------- #
    if Capacity_Constraint:
        # Continuous load on arcs F(i,j) after departing i along (i,j)
        F = {(i, j): model.addVar(vtype=GRB.CONTINUOUS, lb=0.0, ub=Q) for (i, j) in A_sub}

        # If arc unused (X=0), flow is 0; if used (X=1), flow ≤ Q
        CapCouple = {(i, j): model.addConstr(F[i, j] <= Q * X[i, j]) for (i, j) in A_sub}

        # Flow conservation at each customer node v: sum_out F - sum_in F == q[v]
        FlowBal = {
            v: model.addConstr(
                quicksum(F[v, j] for (_, j) in out_arcs[v]) -
                quicksum(F[i, v] for (i, _) in in_arcs[v]) == q[v]
            )
            for v in N
        }

        # Depart empty from depot
        StartEmpty = model.addConstr(
            quicksum(F[depot, j] for (_, j) in out_arcs[depot]) == 0.0
        )

        # Arrive empty at sink
        ArriveEmpty = model.addConstr(
            quicksum(F[i, sink] for (i, _) in in_arcs[sink]) == 0.0
        )

    # --------------------------- Optional time windows --------------------------- #
    if Time_Window:
        # Tight big-Ms and bounds for S
        M_ij, Earliest, Latest = tight_bigM(out_arcs, t, d, V, A_sub, sink, e, l, Pr={r: Pr[r] for r in R}, Dr_single={r: Dr_single[r] for r in R})
        S = {i: model.addVar(vtype=GRB.CONTINUOUS, lb=Earliest[i], ub=Latest[i]) for i in V}

        # Time-window feasibility on arcs
        TimeWindowFeas = {
            (i, j): model.addConstr(S[j] >= S[i] + d[i] + t[i, j] - M_ij[i, j] * (1 - X[i, j]))
            for (i, j) in A_sub
        }

    # ------------------------------- Degree / flow ------------------------------- #
    # Each customer node has exactly one incoming and one outgoing arc
    DegIn = {j: model.addConstr(quicksum(X[i, j] for (i, _) in in_arcs[j]) == 1) for j in N}
    DegOut = {i: model.addConstr(quicksum(X[i, j] for (_, j) in out_arcs[i]) == 1) for i in N}

    model.addConstr(quicksum(X[depot, j] for (_, j) in out_arcs[depot]) == 1)
    model.addConstr(quicksum(X[j, sink] for (j, _) in in_arcs[sink]) == 1)

    # -------------------------- Lazy lifted SECs (callback) ---------------------- #
    EPS = 1e-9
    N_only = set(N)

    def build_Sset(xvals):
        """Return internal cycles entirely within N, using hard 0/1 rounding of X."""
        Sset = set()
        succ = {}
        for i in N_only:
            outs = [(j, xvals.get((i, j), 0.0)) for (ii, j) in out_arcs.get(i, []) if j in N_only]
            if not outs:
                continue
            j_best, v_best = max(outs, key=lambda z: z[1])
            if v_best >= 1.0 - EPS:
                succ[i] = j_best

        unvisited = set(N_only)
        while unvisited:
            u = unvisited.pop()
            path, pos = [], {}
            while True:
                if u not in succ:
                    break
                if u in pos:
                    k = pos[u]
                    cycle = path[k:]
                    if len(cycle) >= 2:
                        Sset.add(frozenset(cycle))
                    break
                pos[u] = len(path)
                path.append(u)
                unvisited.discard(u)
                u = succ[u]
        return Sset

    def build_pi_sigma(S):
        """
        Compute π(S) and σ(S) for MPDPTW lifted SECs.

        Returns
        -------
        (pi_S, sigma_S) : tuple[set, set]
            pi_S    : union of pickup nodes for requests whose delivery ∈ S
            sigma_S : set of delivery nodes for requests having ≥1 pickup ∈ S
        """
        deliveries_in_S = S & node_type["delivery"]
        pickups_in_S = S & node_type["pickup"]

        reqs_with_delivery_in_S = {node_req[n] for n in deliveries_in_S if n in node_req}
        reqs_with_pickup_in_S = {node_req[n] for n in pickups_in_S if n in node_req}

        if reqs_with_delivery_in_S:
            pi_S = {n for r in reqs_with_delivery_in_S for n in Pr.get(r, ())}
        else:
            pi_S = set()

        sigma_S = {Dr_single[r] for r in reqs_with_pickup_in_S if r in Dr_single}
        return pi_S, sigma_S

    model._sec_seen = set()
    model._seen = set()

    def subtour_callback(model, where):
        if where != GRB.Callback.MIPSOL:
            return
        
        XV = model.cbGetSolution(X)

        def sum_x_within(S):
            return quicksum(X[i, j] for i in S for j in S if (i, j) in X)

        def sum_x(from_set, to_set):
            return quicksum(X[i, j] for i in from_set for j in to_set if (i, j) in X)

        def add_cut_once(key, expr, rhs):
            if key in model._seen:
                return False
            model._seen.add(key)
            model.cbLazy(expr <= rhs)
            return True

        def sum_x_within_val(S, XV_):
            return sum(XV_.get((i, j), 0.0) for i in S for j in S)

        def sum_x_val(F, T, XV_):
            return sum(XV_.get((i, j), 0.0) for i in F for j in T)

        # ---- SECs on internal components ----
        cut_added = False
        Sset = build_Sset(XV)

        for S in Sset:
            S = set(S)
            Sbar = N_only - S
            pi_S, sigma_S = build_pi_sigma(S)  # π(S), σ(S)
            rhs = len(S) - 1

            # σ-inequality (49)
            lhs_sigma_expr = sum_x_within(S) + sum_x(Sbar & sigma_S, S) + sum_x(Sbar - sigma_S, S & sigma_S)
            lhs_sigma_val = (
                sum_x_within_val(S, XV)
                + sum_x_val(Sbar & sigma_S, S, XV)
                + sum_x_val(Sbar - sigma_S, S & sigma_S, XV)
            )
            if lhs_sigma_val > rhs + EPS:
                key = ("LSEC_sigma", frozenset(S))

                cut_added |= add_cut_once(key, lhs_sigma_expr, rhs)

            # π-inequality (50)
            lhs_pi_expr = sum_x_within(S) + sum_x(S, Sbar & pi_S) + sum_x(S & pi_S, Sbar - pi_S)
            lhs_pi_val = (
                sum_x_within_val(S, XV)
                + sum_x_val(S, Sbar & pi_S, XV)
                + sum_x_val(S & pi_S, Sbar - pi_S, XV)
            )
            if lhs_pi_val > rhs + EPS:
                key = ("LSEC_pi", frozenset(S))
                cut_added |= add_cut_once(key, lhs_pi_expr, rhs)

        # ---- Route-based precedence cuts ----
        start_arc = next((arc for arc in out_arcs[depot] if XV.get(arc, 0.0) > 0.5), None)
        if start_arc is not None:
            # Follow successors until sink or repetition
            i0, j0 = start_arc
            route = [i0, j0]
            posr = {i0: 0, j0: 1}
            seen = {i0, j0}
            node = j0
            for _ in range(len(V) + 2):
                if node == sink:
                    break
                succs = [j for (_, j) in out_arcs.get(node, []) if XV.get((node, j), 0.0) > 0.5]
                if not succs:
                    break
                j = succs[0]
                if j in seen:
                    route.append(j)
                    posr[j] = len(route) - 1
                    break
                route.append(j)
                posr[j] = len(route) - 1
                seen.add(j)
                node = j

            reqs_on_route = {node_req[v] for v in route if v in node_req}

            # (54): delivery before all pickups of same request
            for r in reqs_on_route:
                dr = Dr_single[r]
                if dr not in posr:
                    continue
                idx_d = posr[dr]
                for p in Pr[r]:
                    if p in posr and posr[p] > idx_d:
                        S_list = route[idx_d + 1 : posr[p]]
                        if not S_list:
                            continue
                        S = set(S_list)
                        key = ("54", dr, p, tuple(S_list))
                        expr = sum_x({dr}, S) + sum_x_within(S) + sum_x(S, {p})
                        cut_added |= add_cut_once(key, expr, len(S))

            # (52): depot → ... → dr but some pickup is not before dr
            for r in reqs_on_route:
                dr = Dr_single[r]
                if dr not in posr:
                    continue
                idx_d = posr[dr]
                late_or_missing = any((p not in posr) or (posr[p] > idx_d) for p in Pr[r])
                if not late_or_missing:
                    continue
                S_list = route[1:idx_d]  # strictly between depot and dr
                if not S_list:
                    continue
                S = set(S_list)
                key = ("52", dr, tuple(S_list))
                expr = sum_x({depot}, S) + sum_x_within(S | {dr})
                cut_added |= add_cut_once(key, expr, len(S))

    
    model.Params.LazyConstraints = 1
    if not Time_Window and not Capacity_Constraint:
        model.Params.Heuristics = 0

    # ------------------------------- Solve & returns ------------------------------ #
    model.optimize(subtour_callback)



    if model.Status == GRB.OPTIMAL:
        used_arcs = [(i, j) for (i, j) in X if X[i, j].X > 0.5]
        if not Time_Window:
            return model, model.ObjVal, used_arcs, None, model.Runtime, model.Work
        else:
            # Include realized service start times S[i] only when time windows are modeled
            return model, model.ObjVal, used_arcs, {i: S[i].X for i in S}, model.Runtime, model.Work
    else:
        # Not proven optimal (e.g., TIME_LIMIT, INFEASIBLE, etc.)
        return model, None, [], None, None, model.Work
