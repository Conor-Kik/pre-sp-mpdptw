# ==========================
# MPDTW parser
# ==========================
import itertools
from pathlib import Path

from gurobipy import GRB
from mpdptw.common.big_M import tight_bigM
from mpdptw.common.route_time_lifted import Run_Time_Model

OUTPUT_W_SET_MODEL = 0
# ── Line parsers ───────────────────────────────────────────────────────────────
def _parse_header(line):
    parts = line.strip().split()
    return int(parts[0]), int(parts[1])

def _parse_node(line):
    # node_id, x, y, demand, tw_start, tw_end, service, node_type(0/1), request_id
    p = line.strip().split()
    return {
        "raw_id":    int(p[0]),
        "x":         int(p[1]),
        "y":         int(p[2]),
        "demand":    int(p[3]),
        "tw_start":  int(p[4]),
        "tw_end":    int(p[5]),
        "service":   int(p[6]),
        "node_type": int(p[7]),   # 0=pickup, 1=delivery
        "request_id":int(p[8]),   # -1 only for depot line
    }

# ── Core parser: text → internal instance ─────────────────────────────────────
def parse_instance_text(text, name="instance"):
    # Strip comments/blank lines
    lines = [ln.strip() for ln in text.splitlines() if ln.strip() and not ln.strip().startswith("#")]

    # Header (vehicles, capacity)
    vehicles, capacity = _parse_header(lines[0])

    # Depot (second line)
    depot_node = _parse_node(lines[1])

    # Customers
    raw_nodes = [_parse_node(x) for x in lines[2:]]

    # Internal indices: 0 = depot, 1..m = customers
    V = [0]
    coords   = {0: (depot_node["x"], depot_node["y"])}
    demand  = {0: depot_node["demand"]}
    tw_a    = {0: depot_node["tw_start"]}
    tw_b    = {0: depot_node["tw_end"]}
    service = {0: depot_node["service"]}
    node_type = {0: -1}
    raw_id    = {0: depot_node["raw_id"]}
    depot = 0

    C, P, D = [], [], []
    R = {}  # rid → {"pickups": [], "deliveries": []}

    for i, n in enumerate(raw_nodes, start=1):
        V.append(i); C.append(i)
        coords[i]    = (n["x"], n["y"])
        demand[i]    = n["demand"]
        tw_a[i]      = n["tw_start"]
        tw_b[i]      = n["tw_end"]
        service[i]   = n["service"]
        node_type[i] = n["node_type"]
        raw_id[i]    = n["raw_id"]

        rid = n["request_id"]
        if rid not in R:
            R[rid] = {"pickups": [], "deliveries": []}
        if n["node_type"] == 0:
            P.append(i); R[rid]["pickups"].append(i)
        elif n["node_type"] == 1:
            D.append(i); R[rid]["deliveries"].append(i)

    return {
        "name": Path(name).stem,
        "vehicles": vehicles,
        "capacity": capacity,
        "depot": depot,
        "V": V,
        "C": C,
        "P": P,
        "D": D,
        "R": R,
        "coords": coords,
        "demand": demand,
        "tw_a": tw_a,
        "tw_b": tw_b,
        "service": service,
        "node_type": node_type,
        "raw_id": raw_id,
        "meta": None,
    }

def parse_instance_file(filename):
    if ("/" in filename) or ("\\" in filename):
        path = filename
    else:
        path = f"mpdtw_instances_2019/{filename}"
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    return parse_instance_text(text, name=filename)

# ── Geometry helpers ──────────────────────────────────────────────────────────
def _euclid(p, q):
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return (dx*dx + dy*dy) ** 0.5

def _euclid_time(p, q, spd):
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return ((dx*dx + dy*dy) ** 0.5) / spd

# ── Extended graph & S-sets ───────────────────────────────────────────────────
def _build_extended_graph_with_sink(V, coords, tw_a, tw_b, service, t_func, speed=1.0):
    """
    Build extended structures that include a distinct end-depot 'sink'.
    Returns: sink, V_ext, A_ext_no_loops, t_ext, c_ext, coords_ext, tw_a_ext, tw_b_ext, service_ext
    """
    sink = max(V) + 1
    V_ext = list(V) + [sink]

    coords_ext = dict(coords)
    coords_ext[sink] = coords_ext[0]

    tw_a_ext = dict(tw_a)
    tw_b_ext = dict(tw_b)
    tw_a_ext[sink] = tw_a_ext[0]
    tw_b_ext[sink] = tw_b_ext[0]

    service_ext = dict(service)
    service_ext[sink] = 0

    A_ext_no_loops = [(i, j) for i in V_ext for j in V_ext if i != j]

    t_ext, c_ext = {}, {}
    for (i, j) in A_ext_no_loops:
        tij = t_func(coords_ext[i], coords_ext[j], speed)
        t_ext[(i, j)] = tij
        c_ext[(i, j)] = tij
    return sink, V_ext, A_ext_no_loops, t_ext, c_ext, coords_ext, tw_a_ext, tw_b_ext, service_ext

def _compute_minimal_S_sets(Pr, Dr_single, sink, start=0):
    """
    Minimal S set per request for the two-index pairing/precedence family:
      S_r^min = Pr ∪ {sink}
    """
    S_list = []
    for r, pickups in Pr.items():
        dr = Dr_single[r]
        S = set(pickups)
        if start in S:
            S.remove(start)
        if dr in S:
            S.remove(dr)
        S.add(sink)
        S_list.append(frozenset(S))
    return S_list

# ── MILP builder: create (V,P,D,N,A,R,Pr,Dr) and (K,Q,e,l,d,q,t,c) ───────────
def build_milp_data(filename, cost_equals_time=True, speed=1.0, generate_W_set=True):
    """
    Returns a dictionary .
    """
    inst = build_dictionary(filename, cost_equals_time=True, speed=1.0)
    r = inst["Nodes_To_Reqs"]
    A_feas = inst["A_feasible_ext"]
    R = inst["R"]
    if generate_W_set:
        W = []
        for r1, r2 in itertools.combinations(R, 2):
            _m, _, _, _, _, _ = Run_Time_Model([r1,r2], inst, Time_Window=True)
            feasible = _m.Status == GRB.OPTIMAL
            if not feasible:
                W.append((r1, r2))
        inst["W"] = W
        inst["A_feasible_ext"] = [
            (i, j) for (i, j) in A_feas
            if (r[i], r[j]) not in W and (r[j], r[i]) not in W
        ]
            
    print(f"-----------\nINITAL ARCS: {len(inst["A"])}, ARCS CUTS: {len(inst["A"]) - len(inst["A_feasible_ext"])}, FINAL {len(inst["A_feasible_ext"])}\n-----------")
    return inst










def build_dictionary(filename, cost_equals_time=True, speed=1.0):
    inst = parse_instance_file(filename)

    # Sets
    V = inst["V"]
    P = inst["P"]
    D = inst["D"]
    N = inst["C"]
    R = list(inst["R"].keys())
    R_dict = inst["R"]
    Pr = {r: list(inst["R"][r]["pickups"])    for r in R}
    Dr = {r: list(inst["R"][r]["deliveries"]) for r in R}
    A = [(i, j) for i in V for j in V]  
    # Parameters
    K = range(int(inst["vehicles"]))
    Q = inst["capacity"]
    e = inst["tw_a"]
    l = inst["tw_b"]
    d = inst["service"]
    q = inst["demand"]
    coords = inst["coords"]


    t, c = {}, {}
    for (i, j) in A:
        tij = _euclid_time(coords[i], coords[j], speed)
        t[(i, j)] = tij
        c[(i, j)] = tij if cost_equals_time else tij
    # Extended graph with distinct sink
    sink, V_ext, A_ext_no_loops, t_ext, c_ext, coords_ext, e_ext, l_ext, d_ext = \
        _build_extended_graph_with_sink(V, coords, e, l, d, _euclid_time, speed=speed)

    # Also provide a no-self-loop list on  V
    A_no_loops = [(i, j) for (i, j) in A if i != j]

    # Single delivery per request (assumed coherent: pick first)
    Dr_single = {r: Dr[r][0] for r in R}


    # map each node to its request
    node_to_req = {}
    for r in R:
        for p in Pr[r]:
            node_to_req[p] = r
        node_to_req[Dr_single[r]] = r
    node_to_req[0] = None
    node_to_req[sink] = None

    deliveries_all = set(Dr_single.values())
    pickups_all    = {p for r in R for p in Pr[r]}
        # Adjacency (restricted to A_sub)
    in_arcs = {v: [] for v in V}
    out_arcs = {v: [] for v in V}
    for (i, j) in A:
        out_arcs[i].append((i, j))
        in_arcs[j].append((i, j))
    e[sink] = e[0]
    l[sink] = l[0]
    d[sink] = 0
    q[sink] = 0
     
    _, Earliest, Latest = tight_bigM(out_arcs, t, d, V+[sink], A, sink, e, l, Pr = Pr, Dr_single=Dr_single)


    def _feasible_arc_ext(i, j):
        if i == 0 and j == sink:
            return False
        if j == 0 or i == sink:
            return False
        if i in pickups_all and j == sink:
            return False
        if i == 0 and j in deliveries_all:
            return False
        if i in pickups_all and j == 0:
            return False
        if (i in deliveries_all) and (j in pickups_all) and (node_to_req[i] == node_to_req[j]):
            return False
        if i == j:
            return False
        return Earliest[i] + d_ext[i] + t_ext[(i, j)] <= Latest[j]

    A_feasible_ext = [(i, j) for (i, j) in A_ext_no_loops if _feasible_arc_ext(i, j)]
    
    # Minimal S-sets (one per request)
    S_minimal_ext = _compute_minimal_S_sets(Pr, Dr_single, sink, start=0)

    return {
        "Nodes_To_Reqs" : node_to_req, 
        "V": V, "P": P, "D": D, "N": N, "A": A,
        "R": R, "R_dict": R_dict, "Pr": Pr, "Dr": Dr,
        "K": K, "Q": Q,
        "e": e, "l": l, "d": d, "q": q,
        "t": t, "c": c,
        "depot": inst["depot"],
        "name": inst["name"],
        "raw_id": inst["raw_id"],
        "meta": inst["meta"],

        "sink": sink,
        "V_ext": V_ext,
        "A_no_loops": A_no_loops,
        "A_ext_no_loops": A_ext_no_loops,
        "A_feasible_ext": A_feasible_ext,
        "t_ext": t_ext,
        "c_ext": c_ext,
        "Dr_single": Dr_single,
        "S_minimal_ext": S_minimal_ext,
        "Coords" : coords

    }

# --------- Example ---------
if __name__ == "__main__":
    milp = build_milp_data("l_8_100_4.txt")

    print(f"Instance: {milp['name']}")
    print(f"Total requests: {len(milp['R'])}")
    print(f"Sink node id: {milp['sink']}")
    print(f"|V|={len(milp['V'])}, |V_ext|={len(milp['V_ext'])}")
    print(f"|A|={len(milp['A'])} (legacy; includes self-loops)")
    print(f"|A_no_loops|={len(milp['A_no_loops'])}, |A_feasible_ext|={len(milp['A_feasible_ext'])}")

    e = milp["e"]; l = milp["l"]; d = milp["d"]; q = milp["q"]
    Pr = milp["Pr"]; Dr = milp["Dr"]
    for rid in milp["R"][:3]:
        print(f"\nRequest {rid + 1}:")
        print("  Pickups:")
        for j in Pr[rid]:
            print(f"    node {j:>3} : e={e[j]}, l={l[j]}, d={d[j]}, q={q[j]}")
        print("  Deliveries:")
        for j in Dr[rid]:
            print(f"    node {j:>3} : e={e[j]}, l={l[j]}, d={d[j]}, q={q[j]}")
