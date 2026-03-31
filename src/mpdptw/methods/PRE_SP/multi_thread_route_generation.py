from math import comb
import os
from gurobipy import *
from mpdptw.common.cli import parse_instance_argv
from mpdptw.common.parsers import build_milp_data
from mpdptw.common.route_time_lifted import Run_Time_Model
from mpdptw.common.printers.col_gen_solution_printer import print_subset_solution
import time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import psutil
import pandas as pd
from collections import defaultdict

# ---------------------------- Configuration Flags ---------------------------- #
VEHICLE_CAPACITY = 0  # whether to enforce capacity feasibility via re-run - don't change
PRINT_ROUTES = False  # whether to print full routes after optimization
MAX_SUBSET = None # Pruned enumeration termination - None if no termination
TIME_LIMIT = 3600 

def mask_to_ids(mask: int):
    ids = []
    while mask:
        lsb = mask & -mask
        ids.append(lsb.bit_length() - 1)
        mask ^= lsb
    return tuple(ids)

# ----------------------- Worker Initialization for Gurobi -------------------- #
def _pricing_worker_init():
    """
    Initialize a private Gurobi environment per process to avoid shared-state
    issues across processes. Suppresses solver output in workers.
    """
    global _ENV
    _ENV = Env(empty=True)
    _ENV.setParam("OutputFlag", 0)
    _ENV.start()


# --------------------------- Worker Task Definition -------------------------- #
def _solve_subset(mask, last, inst, Time_Window, threads):
    """
    Solve the time-window pricing subproblem for a given subset mask in a worker.

    Returns
    -------
    tuple
        (mask, last, status, s_cost, arcs, runtime, work)
    """
    try:
        ids_tuple = mask_to_ids(mask)
        _m, s_cost, arcs, _, runtime, _ = Run_Time_Model(
            ids_tuple,
            inst,
            False,
            Time_Window=Time_Window,
            Threads=threads,
            ENV=_ENV,
        )
        return (mask, last, _m.Status, s_cost, arcs, runtime)

    except Exception as e:
        print("ERROR", e)
        return (mask, last, "EXC", str(e), None, 0.0, 0.0)



# ------------------------------- Parallel Runner ----------------------------- #
def run_k_in_parallel(subsets_k, inst, Time_Window, workers, pricing_threads):
    """
    subsets_k: list of (mask, last)
    Returns list of (mask, last, status, s_cost, arcs, runtime, work)
    """
    results = []
    with ProcessPoolExecutor(max_workers=workers, initializer=_pricing_worker_init) as pool:
        futures = [
            pool.submit(_solve_subset, mask, last, inst, Time_Window, pricing_threads)
            for (mask, last) in subsets_k
        ]
        for fut in as_completed(futures):
                results.append(fut.result())
    return results



# ------------------------------- Main Generation ----------------------------- #
def generate_routes(instance: str, model: Model):
    """
    Generate route columns by enumerating request subsets with pruning,
    then solve the master problem.

    Notes
    -----
    - Uses a frontier that grows by subset size k with early pruning:
      * Infeasible masks (by time-window) are stored to avoid exploring supersets.
      * A quick service-time bound prunes obvious time-infeasible extensions.
    - Parallelization kicks in for large candidate sets.
    - Function prints summary stats and (optionally) chosen routes.
    """

    path = Path(instance)
    filename = path.name
    Time_Window = not filename.startswith("w")  # "w..." instances disable TW
    Narrow = filename.startswith("n")
    if Narrow:
        mt_threshold = 5
    else:
        mt_threshold = 3
    print("USING MULTI-THREADING FOR ROUTE GENERATION")
    print("Workers to use:", psutil.cpu_count(logical=False))
    
    cap_end_time = 0
    cap_start_time = 0
    start_time = time.perf_counter()
    time_limit_hit = False
    inst = build_milp_data(str(instance), generate_W_set=False)
    metrics = {

    }
    
    EPS = 1e-6
    R = inst["R"]
    print()
    print("Requests", len(R))
    print()
    Pr = inst["Pr"]
    Dr_single = inst["Dr_single"]
    d = inst["d"]

    l = inst["l"]
    V = inst["V"]
    depot = inst["depot"]
    K = len(inst["K"])  # vehicles
    Q = inst["Q"]
    q = inst["q"]
    sink = inst["sink"]

    n = len(R)

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

    # Track masks that are known infeasible to prune supersets quickly.
    
    infeasible_masks = set()
    infeasible_by_bit = defaultdict(list) 
    bad_capacity = []
    infeas_at_level = []
    result = {i: [] for i in range(len(R))}
    runtime_remove = 0




    original_obj = None
    time_in_master = 0
    def build_and_optimize(cost_values, route_subsets, final_run = False):
        nonlocal original_obj
        nonlocal time_in_master
        start_time = time.time()
        model = Model("Master Problem")
        model.setParam("OutputFlag", 0)
        # Binary selection variables per generated subset
        Z = {p: model.addVar(vtype=GRB.BINARY) for p in cost_values}

        # Objective: minimize total cost
        model.setObjective(
            quicksum(Z[p] * cost_values[p] for p in cost_values.keys()), GRB.MINIMIZE
        )
        model.addConstr(quicksum(Z[p] for p in cost_values) <= K)
        # Cover each request exactly once
        for r in R:
            model.addConstr(quicksum(Z[ss] for ss in route_subsets[r]) == 1)
        model.optimize()
        if final_run:
            original_obj = model.ObjVal
        time_in_master += time.time() - start_time
        return model, Z



    def add_infeasible_mask(m: int):
        infeasible_masks.add(m)
        mm = m
        while mm:
            b = mm & -mm
            infeasible_by_bit[b].append(m)
            mm ^= b

    def contains_infeasible_subset(m: int) -> bool:
        # Any subset of m must share at least one bit with m.
        mm = m
        while mm:
            b = mm & -mm
            for bad in infeasible_by_bit.get(b, ()):
                if (m & bad) == bad:
                    return True
            mm ^= b
        return False

    # ------------------- Subset Streaming with Early Pruning ------------------ #
    costs = {}  # subset -> cost (objective contribution)
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
            frontier.append((m, p))  # (mask, last_index)

    mask_pruned = 0 
    max_n = 0
    processed_total = 0
    infeasible_found = 0
    
    upper = min(W_max + 1, MAX_SUBSET + 2) if MAX_SUBSET is not None else W_max + 1

    for k in range(1, upper):
        costs_time = {}  # raw s_cost per subset (for pruning bound)
        subsets_k = frontier  # already filtered by masks
        valid_subsets = []
        infeasible_found += pruned
        pruned = 0
        if MAX_SUBSET is not None and k == MAX_SUBSET + 1:

            infeasible_found -= optimal_pruning
            print("Terminating - Max Subset Reached")
            break

        if not subsets_k:
            metrics["Max subset"] = k - 1
            print("Skipping - All subsets pruned")
            break

        print(
            f"[size {k}] Starting.\n"
            f"Routes to check {len(subsets_k)}"
            f"{f' - Infeasible Routes pre-pruned: {optimal_pruning}' if optimal_pruning else ''}"
        )

        start_process = processed
        

        if len(subsets_k) > 100 and k >= mt_threshold:
            # Parallel path
            batch = run_k_in_parallel(
                subsets_k,
                inst,
                Time_Window,
                workers=psutil.cpu_count(logical=False),
                pricing_threads=1,
            )
            for mask, last, status, s_cost, arcs, runtime in batch:
                if status in (GRB.INFEASIBLE, GRB.CUTOFF, "EXC"):
                    add_infeasible_mask(mask) 
                    pruned += 1
                    continue

                subset_ids = mask_to_ids(mask)

                if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                    bad_capacity.append(subset_ids)

                valid_subsets.append((mask, last, runtime))
                costs_time[mask] = s_cost + sum(service_time_r[r] for r in subset_ids)
                costs[subset_ids] = s_cost
                for elem in subset_ids:
                    result[elem].append(subset_ids)
                processed += 1


        else:
            # Sequential path
            for mask, last in subsets_k:
                
                subset_ids = mask_to_ids(mask)
                _m, s_cost, arcs, _, runtime, _ = Run_Time_Model(
                    subset_ids, inst, Time_Window=Time_Window
                )
                if _m.Status in (GRB.INFEASIBLE, GRB.CUTOFF):
                    add_infeasible_mask(mask) 
                    pruned += 1
                    continue
                if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                    bad_capacity.append(subset_ids)

                valid_subsets.append((mask, last, runtime))
                costs_time[mask] = s_cost + sum(service_time_r[r] for r in subset_ids)
                costs[subset_ids] = s_cost
                for elem in subset_ids:
                    result[elem].append(subset_ids)
                processed += 1

        print(
            f"Time: {time.perf_counter()-curr_time:.2f}, "
            f"Cumulative: {time.perf_counter() - start_time - runtime_remove:.2f}\n"
            f"Infeasible Found: {pruned}, "
            f"Valid Routes Found: {processed - start_process}\n"
         f"Infeas among Feas Rate: {100*pruned/len(subsets_k):.2f}%"
        )
        

        elapsed = time.perf_counter() - start_time
        if elapsed > TIME_LIMIT:
            print(
                f"\n=== Global time limit of {TIME_LIMIT:.0f} seconds reached ===\n"
                f"Stopping column generation early at subset size k = {k}."
            )
            time_limit_hit = True
            break
        
        print()
        curr_time = time.perf_counter()
        
        processed_total += processed

        # ----------------- Build frontier for k+1 by extending k ---------------- #
        # Lexicographic extension to avoid duplicates: only add indices > last
        optimal_pruning = 0
        next_frontier = []

        # heuristic: process longer-running subsets first
        valid_subsets.sort(key=lambda x: x[2], reverse=True)
        for mask, last, _ in valid_subsets:
            for p in range(last + 1, n):
                new_mask = mask | (1 << p)
                if contains_infeasible_subset(new_mask):
                    mask_pruned += 1
                    continue

                # Quick feasibility bound: s_cost + service_time_r[p] <= l[depot]
                if (not Time_Window and costs_time[mask] + service_time_r[p] > l[depot]):
                    optimal_pruning += 1
                    mask_pruned += 1
                    continue

                next_frontier.append((new_mask, p))
        #infeas_at_level.append(len(next_frontier)/len(subsets_k))
        infeas_at_level.append(len(subsets_k))

        frontier = next_frontier

    if time_limit_hit:
        print("\n**********************************************")
        print(
            "Time limit reached. Master problem not solved."
        )
        print("**********************************************")
        return metrics

    # ------------------------------- Summary Log ------------------------------ #
    print(
        f"\nAll columns generated. Valid Routes: {processed}. "
        f"\nInfeasible Found: {infeasible_found}, "
        f"Mask Pruned: {mask_pruned}\n"
        f"Infeasibility rate among solved {100*infeasible_found/(infeasible_found+processed):.2f}%\n"
    )



    model, Z = build_and_optimize(costs, result, final_run=True)
    end_time = time.perf_counter()
    end_time -= runtime_remove
    # print(model.ObjVal)
    if VEHICLE_CAPACITY:
        print("**********************************************")
        print("Routes in uncapacitated")
        for p in costs.keys():
            if Z[p].x > 0.5:
                if PRINT_ROUTES:
                    print_subset_solution(inst, p, Capacity_Constraint=False)
                else:
                    print("Requests:", list(p), "Cost:", round(costs[p], 2))
        print("\n**********************************************")
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
                _m, s_cost, _, _, runtime, _ = Run_Time_Model(
                    route, inst, Time_Window=Time_Window, Capacity_Constraint=True
                )
                if _m.Status == GRB.OPTIMAL:
                    new_cost = s_cost
                    print(
                        f"Updated {route} cost from {costs[route]:.2f} to {new_cost:.2f}"
                    )
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
            model, Z = build_and_optimize(costs, result)
        cap_end_time = time.perf_counter()

    if not VEHICLE_CAPACITY or (VEHICLE_CAPACITY and k >= 2):
        if VEHICLE_CAPACITY:
            print("New routes in capitated")
        for p in costs.keys():
            if Z[p].x > 0.5:
                if PRINT_ROUTES:
                    print_subset_solution(inst, p, Capacity_Constraint=VEHICLE_CAPACITY)
                else:
                    print("Requests:", list(p), "Cost:", round(costs[p], 2))

    print("\n**********************************************")
    print(
        f"Column and Master runtime: {end_time - start_time:.2f}{f", Capacity Runtime: {cap_end_time-cap_start_time:.2f}" if VEHICLE_CAPACITY and k >= 2 else ''}"
    )
    total_sp_time = cap_end_time-cap_start_time+ end_time - start_time-time_in_master
    enum_percentage = 100*total_sp_time/(total_sp_time+time_in_master)
    print(
        f"Time spent in subproblem: {total_sp_time:.2f} ~ {enum_percentage:.2f}% | {f", Time spend in master: {round(time_in_master,2) if time_in_master > 0.01 else "<0.01"} "}"
    )
    if VEHICLE_CAPACITY:
        print(f"Uncapacitated Obj Value: {original_obj:.2f}")
    print(f"{"Capacitated " if VEHICLE_CAPACITY else ''}Obj Value: {model.ObjVal:.2f}")
    print("**********************************************")

    
    
    metrics["uncap_time"] = end_time - start_time
    metrics["sp_time"] = time_in_master
    metrics["uncap_obj"] = original_obj
    if VEHICLE_CAPACITY:
        metrics["cap_iterations"] = k - 1
        metrics["cap_obj"] = model.ObjVal
        metrics["extra_time"] = cap_end_time - cap_start_time
    metrics["Feasible Routes"] = processed
    metrics["Infeasible Routes"] = infeasible_found
    metrics["Infeas at K"] = infeas_at_level
    metrics["Enumeration Percentage"] = enum_percentage
    return metrics

def main(argv=None):
    global VEHICLE_CAPACITY
    if os.environ.get("ROUTE_GEN_CAP") == "1":
        VEHICLE_CAPACITY = 1
    else:
        VEHICLE_CAPACITY = 0
    path, _ = parse_instance_argv(argv)
    model = Model("MPDTW")
    
    metrics = generate_routes(str(path), model)
    #print(metrics)
    print("Done.")
