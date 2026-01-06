import os
from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.common.route_time_lifted import Run_Time_Model
from mpdptw.common.printers.col_gen_solution_printer import print_subset_solution
import time
from pathlib import Path
from math import comb

# ---------------------------- Configuration Flags ---------------------------- #
VEHICLE_CAPACITY = 0  # set to 1 to add vehicle capacity constraints (re-run check)
PRINT_ROUTES = 0      # set to 1 to print full routes instead of compact output

# --------------------------------- Main Logic -------------------------------- #
def generate_routes(instance: str, model: Model):
    """
    Enumerate request subsets with early pruning, price each subset with
    Run_Time_Model, optionally re-check capacity feasibility, then solve the
    master selection problem over generated columns.

    Notes
    -----
    - The subset enumeration uses bitmask-based infeasibility pruning:
      once a subset is infeasible, all supersets are skipped.
    - Capacity feasibility (if VEHICLE_CAPACITY == 1) is verified by a quick
      route traversal; if violated, the subset is re-solved with explicit
      capacity constraints via Run_Time_Model(..., VEHICLE_CAPACITY=1).
    - Function prints progress per subset size and final solution stats.
    """
    if VEHICLE_CAPACITY:
        print("Running with Vehicle Capacity Constraints")
    if not PRINT_ROUTES:
        model.setParam("OutputFlag", 0)
    # Parse instance and basic settings
    start_time = time.perf_counter()
    inst = build_milp_data(str(instance), generate_W_set=False)
    
    path = Path(instance)
    filename = path.name
    Time_Window = not filename.startswith("w")  # "w..." instances disable TW


    R = inst["R"]
    Pr = inst["Pr"]
    Dr_single = inst["Dr_single"]
    EPS = 1e-6
    K = len(inst["K"])  # vehicles
    Q = inst["Q"]
    d = inst["d"]
    q = inst["q"]
    depot = inst["depot"]
    sink = inst["sink"]
    l    = inst["l"]

    n = len(R)
    total_work_units = 0
    def is_capacity_ok(arcs):
        succ = {i: j for (i, j) in arcs}
        route = [depot]
        cur = depot
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

    # ----------------- Infeasible-subset masks for early pruning -------------- #
    infeasible_masks = set()
    bad_capacity = []
    def contains_infeasible_subset(mask):
        """Return True if 'mask' contains any known infeasible submask."""
        return any((mask & bad) == bad for bad in infeasible_masks)



    # ------------------- Subset Streaming with Early Pruning (sequential) ------------------ #

    costs = {}       # subset -> cost (objective contribution)
    curr_time = time.perf_counter()
    pruned = processed = optimal_pruning = 0
    W_max = n  # upper bound on subset size

    # Precompute service time of each request's required nodes
    service_time_r = {
        r: sum(d.get(v, 0.0) for v in Pr[r]) + d.get(Dr_single[r], 0.0) for r in R
    }

    # Initialize frontier with singletons (as tuples) that aren't immediately pruned
    frontier = []
    for p in range(n):
        m = 1 << p
        if not contains_infeasible_subset(m):
            frontier.append(((p,), m))  # (ids_tuple, mask)

    max_n = 0
    total_pruned = 0

    for k in range(1, W_max + 1):
        costs_time = {}        # raw s_cost per subset (for pruning bound)
        subsets_k = frontier   # already filtered by masks
        valid_subsets = []

        if not subsets_k:
            max_n = k
            print("Skipping - All subsets pruned")
            break

        print(
            f"[size {k}] Starting.\n"
            f"Routes to check {len(subsets_k)}"
            f"{f' - Infeasible Routes pre-pruned: {optimal_pruning}' if optimal_pruning else ''}"
        )

        start_process = processed
        total_pruned += pruned
        pruned = 0
        # ------------------------- Sequential processing path ------------------------- #
        for subset_ids, mask in subsets_k:
            # First pass (no explicit capacity constraint)
            _m, s_cost, arcs, _, runtime, work = Run_Time_Model(
                subset_ids, inst, Time_Window=Time_Window
            )
            total_work_units += work
            if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                infeasible_masks.add(mask)
                pruned += 1
                continue
            if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                bad_capacity.append(subset_ids)


            # Keep for frontier extension and cost computation
            valid_subsets.append((subset_ids, mask, runtime))
            costs_time[tuple(subset_ids)] = s_cost
            costs[tuple(subset_ids)] = s_cost - sum(service_time_r[r] for r in subset_ids)
            processed += 1

        print(
            f"Time: {time.perf_counter()-curr_time:.2f}, "
            f"Infeasible Found: {pruned}, "
            f"Valid Routes Found: {processed - start_process}\n"
        )
        curr_time = time.perf_counter()

        # ----------------- Build frontier for k+1 by extending k ---------------- #
        # Lexicographic extension to avoid duplicates: only add indices > last
        optimal_pruning = 0
        next_frontier = []


        for ids, mask, _ in valid_subsets:
            start = ids[-1] + 1
            for p in range(start, n):
                new_mask = mask | (1 << p)
                #Superset pruning - Lemma 1
                if contains_infeasible_subset(new_mask):
                    continue
                #Lemma 2 - Quick feasibility bound: s_cost + service_time_r[p] <= l[depot] 
                if costs_time[tuple(ids)] + service_time_r[p] > l[depot] + EPS:
                    optimal_pruning += 1
                    total_pruned += 1
                    continue
                next_frontier.append((ids + (p,), new_mask))

        frontier = next_frontier

    if max_n == 0:
        max_n = W_max + 1

    print(
        f"\nAll columns generated. Valid Routes: {processed}. "
        f"\nInfeasible Found: {total_pruned}, "
        f"Subsets (≤ length {max_n - 1}) pruned from infeasible: "
        f"{sum(comb(n, k) for k in range(1, max_n)) - processed - total_pruned}"
    )


    # ------------------------- Master Problem Assembly ------------------------ #
    # Map each request to all generated subsets that include it
    result = {i: [] for i in range(len(R))}
    for s in costs.keys():
        for elem in s:
            result[elem].append(s)


    original_obj = None
    def build_and_optimize():
        nonlocal original_obj
        model.reset()
        # Binary selection variables per generated subset
        Z = {p: model.addVar(vtype=GRB.BINARY) for p in costs}

        # Objective: minimize total cost
        model.setObjective(
            quicksum(Z[p] * costs[p] for p in costs.keys()), GRB.MINIMIZE
        )
        model.addConstr(quicksum(Z[p] for p in costs) <= K)
        # Cover each request exactly once
        for r in R:
            model.addConstr(quicksum(Z[ss] for ss in result[r]) == 1)
        model.optimize()
        if not original_obj:
            original_obj = model.ObjVal
        return Z
    
    Z = build_and_optimize()
    end_time = time.perf_counter()
    
    if VEHICLE_CAPACITY:
        
        print("**********************************************")
        print("CHECKING CAPACITY VIOLATIONS")
        k = 1
        cap_start_time = time.perf_counter()
        while True:
            print("**********************************************")
            print(f"Iteration: {k}")
            routes = [route for route in Z if Z[route].X > 0.5]
            changed = False
            for route in routes:
                if route not in bad_capacity:
                    continue
                _m, s_cost, _, _, _, work = Run_Time_Model(
                    route, inst, Time_Window=Time_Window, Capacity_Constraint=True
                    )
                total_work_units += work
                if _m.Status == GRB.OPTIMAL:
                    new_cost = s_cost - sum(service_time_r[r] for r in route)
                    if abs(new_cost - costs[route]) > EPS:
                        
                        print(f"Updated {route} cost from {costs[route]:.2f} to {new_cost:.2f}")
                        costs[route] = new_cost
                        bad_capacity.remove(route)
                        changed = True
                else:
                    # remove the infeasible route itself
                    del costs[route]
                    for r in route:
                        result[r].remove(route)
                    print(f"Removed {route} from valid route set")

                    # additionally prune any supersets of this route
                    for sup in list(costs.keys()):
                        if set(route).issubset(sup):
                            del costs[sup]
                            for r in sup:
                                result[r].remove(sup)
                            print(f"Removed superset {sup}")

                    changed = True

            if not changed:
                print("ALL ROUTES MEET CAPACITY REQUIREMENTS")
                print("**********************************************")
                break
            k += 1
            Z = build_and_optimize()
        cap_end_time = time.perf_counter()

    for p in costs.keys():
        if Z[p].x > 0.5:
            if PRINT_ROUTES:
                print_subset_solution(inst, p, Capacity_Constraint=VEHICLE_CAPACITY)
            else:
                print("Requests:", list(p), "Cost:", round(costs[p], 2))

    print("**********************************************")
    print(f"Column and Master runtime: {end_time - start_time:.2f}{f", Capacity Runtime: {cap_end_time-cap_start_time:.2f}" if VEHICLE_CAPACITY else ''}")
    if VEHICLE_CAPACITY:
        print(f"Uncapacitated Obj Value: {original_obj:.2f}")
    print(f"{"Capacitated " if VEHICLE_CAPACITY else ''}Obj Value: {model.ObjVal:.2f}")
    print(f"Total Work Units: {total_work_units:.2f}")
    print("**********************************************")


def main(argv=None):
    global VEHICLE_CAPACITY
    if os.environ.get("ROUTE_GEN_CAP") == "1":
        VEHICLE_CAPACITY = 1

    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    generate_routes(str(path), model)


if __name__ == "__main__":
    main()
