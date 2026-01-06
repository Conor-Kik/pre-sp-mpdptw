from collections import deque

def tight_bigM(out_arcs, t, d, V, A, sink, e, l, Pr=None, Dr_single=None,
               max_iterations=50):
    """
    5.1 time-window tightening.
    - Worklist (label-correcting) updates, soft-capped at max_iterations (default 50)
    - Delivery-aware precedence tightening
    - Rebuild tight Big-M

    Returns: (M_ij, Earliest, Latest)
    """
    EPS=1e-10
    # Build incoming adjacency
    in_arcs = {j: [] for j in V}
    for i, j in A:
        in_arcs[j].append((i, j))

    # Ensure every node has an out list
    for k in V:
        out_arcs.setdefault(k, [])
    # Initialize from raw windows
    a = dict(e)  # Earliest
    b = dict(l)  # Latest
    # --- worklist (label-correcting) fixed-point loop over the four rules ---
    q = deque(V)                # start with all nodes
    inQ = {v: True for v in V}

    iters = 0
    while iters < max_iterations:
        iters += 1
        # process current frontier size once per "round"
        for _ in range(len(q)):
            k = q.popleft()
            inQ[k] = False
            ak_old, bk_old = a[k], b[k]

            # Step 1: a_k ≥ min_{i in A-(k)} (a_i + s_i + t_ik)
            if in_arcs[k]:
                cand = min(a[i] + d[i] + t[(i, k)] for i, _ in in_arcs[k])
                if cand > a[k] + EPS:
                    a[k] = cand

            # Step 2: a_k ≥ min{ b_k, min_{j in A+(k)} (a_j − s_k − t_kj) }
            if out_arcs[k]:
                cand2 = min(a[j] - d[k] - t[(k, j)] for _, j in out_arcs[k])
                cand2 = min(b[k], cand2)
                if cand2 > a[k] + EPS:
                    a[k] = cand2

            # Step 3: b_k ≤ max{ a_k, max_{i in A-(k)} (b_i + s_i + t_ik) }
            if in_arcs[k]:
                cand3 = max(b[i] + d[i] + t[(i, k)] for i, _ in in_arcs[k])
                cand3 = max(a[k], cand3)
                if cand3 < b[k] - EPS:
                    b[k] = cand3

            # Step 4: b_k ≤ max_{j in A+(k)} (b_j − s_k − t_kj)
            if out_arcs[k]:
                cand4 = max(b[j] - d[k] - t[(k, j)] for _, j in out_arcs[k])
                if cand4 < b[k] - EPS:
                    b[k] = cand4

            if (a[k] > ak_old + EPS) or (b[k] < bk_old - EPS):
                # a[k] increased → successors may need to lift their a (Step 2)
                for _, j in out_arcs[k]:
                    if not inQ.get(j, False):
                        q.append(j); inQ[j] = True
                # b[k] decreased → predecessors may need to lower their b (Step 3)
                for i, _ in in_arcs[k]:
                    if not inQ.get(i, False):
                        q.append(i); inQ[i] = True

        if not q:
            break 

    # --- delivery tightening ---
    if Pr is not None and Dr_single is not None:
        for r, dv in Dr_single.items():
            if dv in a:
                lb_dv = max(a[p] + d[p] + t[(p, dv)] for p in Pr[r])
                if lb_dv > a[dv] + EPS:
                    a[dv] = lb_dv

    # --- tight Big-M ---
    M_ij = {(i, j): max(0.0, b[i] + d[i] + t[(i, j)] - a[j]) for (i, j) in A}
    return M_ij, a, b
