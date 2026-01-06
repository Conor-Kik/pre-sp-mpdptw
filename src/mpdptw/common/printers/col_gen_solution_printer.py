# --- minimal safe getters / route extraction ---
from mpdptw.common.route_time_lifted import Run_Time_Model
from mpdptw.common.big_M import tight_bigM


def _as_delivery_node(d):
    if isinstance(d, (list, tuple, set)):
        return next(iter(d))
    return d

def _val(v):
    if hasattr(v, "X"):  # Gurobi var
        return float(v.X)
    try:
        return float(v)
    except Exception:
        return 0.0

def _x(X, i, j):
    """Safe getter for arc var/flag X[i,j]; returns 0.0 if missing."""
    try:
        v = X[i, j]
    except Exception:
        v = X.get((i, j), 0.0) if hasattr(X, "get") else 0.0
    return _val(v) if v is not None else 0.0

def _coerce_X(arcs):
    """Accept dict/tupledict of vars/values OR list of (i,j) → dict[(i,j)]=1.0."""
    # dict-like
    try:
        items = arcs.items()
        return {tuple(k): (_val(v) if v is not None else 0.0) for k, v in items}
    except Exception:
        pass
    # iterable of pairs
    X = {}
    try:
        for i, j in arcs:
            X[(i, j)] = 1.0
    except Exception:
        pass
    return X

def _succ_from_solution(V_ext, X, sink=None):
    succ, depot_starts = {}, []
    for i in V_ext:
        for j in V_ext:
            if i == j:
                continue
            if _x(X, i, j) > 0.5:
                if i == 0:
                    depot_starts.append(j)
                else:
                    succ[i] = j
    if sink is not None:
        depot_starts = [j for j in depot_starts if j != sink]
    return succ, depot_starts

def _extract_routes(V_ext, X, sink=None):
    succ, starts = _succ_from_solution(V_ext, X, sink=sink)
    routes, visited = [], set()
    for start in starts:
        rt = [0, start]
        visited.add(start)
        cur = start
        while True:
            nxt = succ.get(cur)
            if nxt is None:
                break
            rt.append(nxt)
            if (sink is not None and nxt == sink) or (nxt in visited):
                break
            visited.add(nxt)
            cur = nxt
        routes.append(rt)
    return routes

def _build_node_index(R, Pr, Dr, sink=None):
    node_type = {0: ("DepotStart", None)}
    if sink is not None:
        node_type[sink] = ("DepotEnd", None)
    for r in R:
        for p in Pr[r]:
            node_type[p] = ("Pickup", r)
        dnode = _as_delivery_node(Dr[r])
        node_type[dnode] = ("Delivery", r)
    return node_type


# --- the one function you asked for ---

def print_subset_solution(inst, p_subset, Capacity_Constraint = 0):
    """
    Re-run the subproblem for chosen subset p_subset and print a compact summary.
    Prints **travel-only objective** (sum of t[i,j] on chosen arcs), and also
    reports service/wait total (sum d[i]) and the global horizon.
    """
    # unpack instance bits we need
    V_ext = inst["V_ext"]
    R     = inst["R"]
    K     = inst["K"]
    Pr    = inst["Pr"]
    Dr    = inst["Dr"]
    e     = inst.get("e", {})
    l     = inst.get("l", {})
    q     = inst.get("q", {})
    d     = inst.get("d", {})
    t     = inst.get("t_ext", {})
    Q     = inst["Q"]
    EPS   = 1e-6
    sink  = inst.get("sink")
    V = inst["V_ext"]  
    A = inst["A_feasible_ext"]
    in_arcs  = {v: [] for v in V}
    out_arcs = {v: [] for v in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))

    def is_capacity_ok(arcs):

        succ = {i: j for (i, j) in arcs}
        route = [0]
        cur = 0
        for _ in range(len(arcs) + 2): 
            if cur == sink:
                break
            cur = succ[cur]
            route.append(cur)
        load = 0.0
        for v in route:
            load += q.get(v, 0.0)
            if load > Q + EPS:
                return False
        return True


    # solve the subproblem again to “turn arcs on”
    _m, s_cost, arcs, S, _,_ = Run_Time_Model(p_subset, inst, Time_Window=True)
    if Capacity_Constraint:
        if not is_capacity_ok(arcs):
            _m, s_cost, arcs, S, _ , _= Run_Time_Model(p_subset, inst, Time_Window=True ,Capacity_Constraint=True)

    X = _coerce_X(arcs)

    # reconstruct routes
    routes = _extract_routes(V_ext, X, sink=sink)
    node_info = _build_node_index(R, Pr, Dr, sink=sink)

    # manual S: S[0]=0; then S[v] = S[u] + d[u] + t[u,v]

    # --- compute travel-only objective + service/wait and horizon ---
    # travel-only obj = sum t[i,j] over arcs that are "on"
    chosen_arcs = [(i, j) for i in V_ext for j in V_ext if i != j and _x(X, i, j) > 0.5]
    travel_only = 0.0
    for (i, j) in chosen_arcs:
        tij = t.get((i, j))
        if tij is not None:
            travel_only += float(tij)

    # service/wait total over visited nodes (exclude depot 0; include sink’s incoming service if you model it in d)
    visited_nodes = set()
    for rt in routes:
        visited_nodes.update(rt)
    service_wait = sum(float(d.get(v, 0.0)) for v in visited_nodes if v != 0)

    # global horizon = max route end time; if route ends at sink, ensure S[sink] reflects arrival
    # We already computed S progressively, including t + d. If the last node is sink, S[sink] is arrival.
    route_ends = []
    for rt in routes:
        if not rt:
            continue
        last = rt[-1]
        route_ends.append(S.get(last, 0.0))
    global_horizon = max(route_ends) if route_ends else 0.0

    # print header with travel-only objective
    print("\n" + "="*76)
    print(f"SOLUTION for subset Requests {sorted(list(p_subset))}")
    print("="*76)
    print(f"Objective (travel-only) : {travel_only:.3f}")
    print(f"Service/Wait total (Σ d): {service_wait:.3f}")
    print(f"Global horizon (end)    : {global_horizon:.3f}")

    # vehicles used
    starts = [j for j in V_ext if j != 0 and (sink is None or j != sink) and _x(X, 0, j) > 0.5]
    print(f"Vehicles                 : {len(starts)} used / {len(K) if hasattr(K, '__len__') else int(K)} available")

    # routes detail (same style as before)
    print("\nROUTES (node, type, req | S within [e,l] | demand q | t(prev->v))")
    for ridx, route in enumerate(routes, 1):
        print("-"*76)
        print(f"Route #{ridx}: " + " -> ".join(map(str, route)))
        hdr = f"{'node':>5}  {'type':>10}  {'req':>4}  {'S':>10}   {'[e,l]':>13}  {'q':>6}  {'t(prev->v)':>11}"
        print(hdr)
        print("-"*len(hdr))
        total_time = 0.0
        for idx, v in enumerate(route):
            typ, req = node_info.get(v, ("Other", None))
            s_val = S.get(v, float("nan"))
            e_v = e.get(v, 0.0)
            l_v = l.get(v, 0.0)
            q_v = q.get(v, 0.0)
            if idx > 0:
                u = route[idx - 1]
                tij = float(t.get((u, v), float("nan")))
                tprev_str = f"{tij:.2f}" if tij == tij else "-"
                if tij == tij:
                    total_time += tij
            else:
                tprev_str = "-"
            req_str = "-" if req is None else str(req)
            el_str = f"[{e_v:.0f},{l_v:.0f}]"
            s_str = f"{s_val:.2f}" if s_val == s_val else "nan"
            print(f"{v:>5}  {typ:>10}  {req_str:>4}  {s_str:>10}   {el_str:>13}  {q_v:6.2f}  {tprev_str:>11}")
        print(f"{'':>56}Total travel: {round(total_time, 2)}")


    print("="*76)
