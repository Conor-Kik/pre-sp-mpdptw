
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


# ---------------------------- Configuration Flags ---------------------------- #
VEHICLE_CAPACITY = 0 # whether to enforce capacity feasibility via re-run
PRINT_ROUTES = 0     # whether to print selected routes after optimization


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
def _solve_subset(
    ids_tuple,
    mask,
    inst,
    Time_Window,
    threads,
):
    """
    Solve the time-window pricing subproblem for a given subset in a worker.

    Returns
    -------
    tuple
        (ids_tuple, mask, status, s_cost, arcs, runtime)
    """
    try:
        _m, s_cost, arcs, _, runtime, work = Run_Time_Model(
            ids_tuple,
            inst,
            False,
            Time_Window=Time_Window,
            Threads=threads,
            ENV=_ENV,
        )
        return (ids_tuple, mask, _m.Status, s_cost, arcs, runtime, work)

    except Exception as e:
        # Keep worker robust and return error info upstream.
        print("ERROR", e)
        return (ids_tuple, mask, "EXC", str(e), None, 0.0)


# ------------------------------- Parallel Runner ----------------------------- #
def run_k_in_parallel(
    subsets_k,
    inst,
    Time_Window,
    workers,
    pricing_threads,
):
    """
    Submit all k-sized subsets to the process pool.

    Returns
    -------
    list
        List of worker results, each matching _solve_subset()'s return format.
    """
    results = []
    with ProcessPoolExecutor(
        max_workers=workers, initializer=_pricing_worker_init
    ) as pool:
        futures = [
            pool.submit(
                _solve_subset,
                tuple(ids),
                mask,
                inst,
                Time_Window,
                pricing_threads,
            )
            for ids, mask in subsets_k
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

    model.setParam("OutputFlag", 0)
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
    TIME_LIMIT = 3600.0
    cap_end_time = 0 
    cap_start_time = 0
    start_time = time.perf_counter()
    time_limit_hit = False
    inst = build_milp_data(str(instance), generate_W_set=False)
    metrics = {
        "sts": None,              # 0 or 1
        "uncap_time": None,       # end_time - start_time
        "uncap_obj": None,        # original_obj
        "cap_iterations": None,   # k-1
        "cap_obj": None,          # model.ObjVal
        "extra_time": None,       # cap_end_time - cap_start_time
    }
    EPS = 1e-6
    R = inst["R"]
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
    bad_capacity = []
    def contains_infeasible_subset(mask):
        return any((mask & bad) == bad for bad in infeasible_masks)


    # ------------------- Subset Streaming with Early Pruning ------------------ #
    costs = {}       # subset -> cost (objective contribution)
    curr_time = time.perf_counter()
    pruned = processed = optimal_pruning = 0
    W_max = n  # upper bound on subset size
    total_work_units = 0
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
    processed_total = 0
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

        if  len(subsets_k) > 100 and k >= mt_threshold:
            # Parallel path
            batch = run_k_in_parallel(
                subsets_k,
                inst,
                Time_Window,
                workers=psutil.cpu_count(logical=False),
                pricing_threads=1,
            )
            for subset_ids, mask, status, s_cost, arcs, runtime, work in batch:
                total_work_units += work
                if status in (GRB.INFEASIBLE, GRB.CUTOFF, "EXC"):
                    infeasible_masks.add(mask)
                    pruned += 1
                    continue
                if VEHICLE_CAPACITY and not is_capacity_ok(arcs):
                    bad_capacity.append(subset_ids)

                valid_subsets.append((subset_ids, mask, runtime))
                costs_time[tuple(subset_ids)] = s_cost
                # Pure travel cost: subtract service time contribution
                costs[tuple(subset_ids)] = s_cost - sum(
                    service_time_r[r] for r in subset_ids
                )
                processed += 1
        else:
            # Sequential path
            for subset_ids, mask in subsets_k:
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


                valid_subsets.append((subset_ids, mask, runtime))
                costs_time[tuple(subset_ids)] = s_cost
                costs[tuple(subset_ids)] = s_cost - sum(
                    service_time_r[r] for r in subset_ids
                )
                processed += 1

        print(
            f"Time: {time.perf_counter()-curr_time:.2f}, "
            f"Infeasible Found: {pruned}, "
            f"Valid Routes Found: {processed - start_process}\n"
        )

        elapsed = time.perf_counter() - start_time
        if elapsed > TIME_LIMIT:
            print(
                f"\n=== Global time limit of {TIME_LIMIT:.0f} seconds reached ===\n"
                f"Stopping column generation early at subset size k = {k}."
            )
            time_limit_hit = True
            break
        curr_time = time.perf_counter()
        processed_total += processed

        # ----------------- Build frontier for k+1 by extending k ---------------- #
        # Lexicographic extension to avoid duplicates: only add indices > last
        optimal_pruning = 0
        next_frontier = []

        # heuristic: process longer-running subsets first
        valid_subsets.sort(key=lambda x: x[2], reverse=True)
        for ids, mask, _ in valid_subsets:
            start = ids[-1] + 1
            for p in range(start, n):
                new_mask = mask | (1 << p)
                if contains_infeasible_subset(new_mask):
                    continue

                # Quick feasibility bound: s_cost + service_time_r[p] <= l[depot]
                if costs_time[tuple(ids)] + service_time_r[p] > l[depot]:
                    optimal_pruning += 1
                    total_pruned += 1
                    continue

                next_frontier.append((ids + (p,), new_mask))

        frontier = next_frontier

    if time_limit_hit:
        print("\n**********************************************")
        print("Time limit reached. Master problem not solved; instance marked as unsolved.")
        print("**********************************************")
        metrics["sts"] = 0
        return metrics

    # ------------------------------- Summary Log ------------------------------ #
    print(
        f"\nAll columns generated. Valid Routes: {processed}. "
        f"\nInfeasible Found: {total_pruned}, "
        f"Subsets (≤ length {max_n - 1}) pruned from infeasible: "
        f"{sum(comb(n, k) for k in range(1, max_n)) - processed - total_pruned}"
    )



    # Map each request to all subsets that contain it
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
                _m, s_cost, _, _, runtime, work = Run_Time_Model(
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
        
    print("\n**********************************************")
    print(f"Column and Master runtime: {end_time - start_time:.2f}{f", Capacity Runtime: {cap_end_time-cap_start_time:.2f}" if VEHICLE_CAPACITY and k >= 2 else ''}")
    if VEHICLE_CAPACITY:
        print(f"Uncapacitated Obj Value: {original_obj:.2f}")
    print(f"{"Capacitated " if VEHICLE_CAPACITY else ''}Obj Value: {model.ObjVal:.2f}")
    print(f"Total Work Units: {total_work_units:.2f}")
    print("**********************************************")
    
    
    metrics["cap_iterations"] = k - 1
    metrics["extra_time"] = cap_end_time - cap_start_time
    metrics["sts"] = 1
    metrics["uncap_time"] = end_time - start_time
    metrics["uncap_obj"] = original_obj
    metrics["cap_obj"] = model.ObjVal
    return metrics


# ------------------------------------ CLI ----------------------------------- #
def main(argv=None):
    global VEHICLE_CAPACITY
    if os.environ.get("ROUTE_GEN_CAP") == "1":
        VEHICLE_CAPACITY = 1
    else:
        VEHICLE_CAPACITY = 0
    path, _ = parse_instance_argv(argv, default_filename="l_4_25_4.txt")
    model = Model("MPDTW")
    generate_routes(str(path), model)
    #run_all_instances()


# fill_results.py (put this in project root)




def run_all_instances():
    project_root = Path(__file__).resolve().parents[4]

    instance_dir = project_root / "mpdtw_instances_2019"
    results_path = project_root / "results.xlsx"

    print("Project root:", project_root)
    print("Instance dir:", instance_dir)
    print("Results path:", results_path)

    # Load your results file
    df = pd.read_excel(results_path)

    # Column names as they currently appear in results.xlsx
    INSTANCE_COL = "Unnamed: 1"                     # Instance
    STS_COL = "Uncapacitated Column Generation"     # Sts
    UNCAP_OBJ_COL = "Unnamed: 3"                    # Obj
    UNCAP_TIME_COL = "Unnamed: 4"                   # Time (s)

    CAP_ITER_COL = "Capacitated Column Correction"  # Iterations
    CAP_OBJ_COL = "Unnamed: 6"                      # Obj
    EXTRA_TIME_COL = "Unnamed: 7"                   # Extra Time (s)

    # Same filter as your example main()
    filenames = [
        f for f in os.listdir(instance_dir)
        if f.endswith(".txt")
        # skip any w* containing _200
        and (f.startswith("w_8") and "_200_5" in f)
        # skip any l_4_200_*
        #and not f.startswith("l_4_200")
        # skip all w_4_100 except w_4_100_3
        #and not (f.startswith("w_4_100") and f != "w_4_100_3.txt")
    ]



    filenames.sort(key=str.lower)
    print(filenames)
    total = len(filenames)
    print(f"Found {total} instances")

    for i, filename in enumerate(filenames, start=1):
        instance_name = os.path.splitext(filename)[0]  # "l_4_25_1"
        row_mask = df[INSTANCE_COL] == instance_name

        if not row_mask.any():
            print(f"⚠ Instance {instance_name} not found in results.xlsx, skipping.")
            continue

        row_idx = df.index[row_mask][0]

        # --- SKIP IF THIS ROW ALREADY HAS RESULTS ---
        sts_val = df.loc[row_idx, STS_COL]
        if not (pd.isna(sts_val) or (isinstance(sts_val, str) and sts_val.strip() == "")):
            print(f"⏩ Skipping {instance_name} (already has Sts={sts_val})")
            continue

        path = instance_dir / filename

        print(f"▶ Running ({i}/{total}) {filename}")
        model = Model("MPDTW")
        metrics = generate_routes(str(path), model)
        print(f"✔ Done ({i}/{total}) {filename}\n")

        sts = metrics["sts"]

        def r(x):
            return "~" if x is None else round(x, 2)

        # If time limit hit: sts = 0 and all other columns = '~'
        if sts == 0:
            df.loc[row_idx, STS_COL] = 0
            df.loc[row_idx, UNCAP_OBJ_COL] = "~"
            df.loc[row_idx, UNCAP_TIME_COL] = "~"
            df.loc[row_idx, CAP_ITER_COL] = "~"
            df.loc[row_idx, CAP_OBJ_COL] = "~"
            df.loc[row_idx, EXTRA_TIME_COL] = "~"
            sleep_time = 300


        else:
            # Solved: sts = 1 and we fill numbers from metrics
            df.loc[row_idx, STS_COL] = 1

            # Uncapacitated
            df.loc[row_idx, UNCAP_OBJ_COL] = r(metrics["uncap_obj"])
            df.loc[row_idx, UNCAP_TIME_COL] = r(metrics["uncap_time"])

            # Capacitated corrections – only meaningful if VEHICLE_CAPACITY == 1
            if VEHICLE_CAPACITY == 1:
                df.loc[row_idx, CAP_ITER_COL] = r(metrics["cap_iterations"])
                df.loc[row_idx, CAP_OBJ_COL] = r(metrics["cap_obj"])
                df.loc[row_idx, EXTRA_TIME_COL] = r(metrics["extra_time"])
            else:
                # If you're running in purely uncapacitated mode, keep these blank
                df.loc[row_idx, CAP_ITER_COL] = "~"
                df.loc[row_idx, CAP_OBJ_COL] = "~"
                df.loc[row_idx, EXTRA_TIME_COL] = "~"
            total_runtime = metrics["uncap_time"] + (metrics["extra_time"] or 0)
            sleep_time = min(2 * total_runtime, 300)

        # Save after each instance so progress is never lost
        df.to_excel(results_path, index=False)
        print(f"Cooling down for {sleep_time:.1f} seconds...")
        time.sleep(sleep_time)
    print("All done, results.xlsx updated.")


if __name__ == "__main__":
    main()

