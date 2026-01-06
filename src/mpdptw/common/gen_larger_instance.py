from pathlib import Path
import random

# ========= CONFIG (edit this) =========
TARGET_SIZE = 200            # <-- set to 200 (or 400 if you use that branch)
BASE_DIR = Path("mpdtw_instances_2019")
PATTERN = "w_8_100*.txt"
OUTPUT_PREFIX_200 = PATTERN[:4] + "200" 
OUTPUT_PREFIX_400 = "n_8_400"
RNG_SEED = 12345            
# =====================================

rng = random.Random(RNG_SEED)

def parse_instance(path: Path):
    """
    Paper-format MPDPTW TXT -> (vehicles, capacity, depot_row, requests:list).
    Row: [node_id, x, y, demand, tw_start, tw_end, service, node_type, request_id]
    Request: {'pickups': [rows...], 'delivery': row}
    """
    txt = path.read_text(encoding="utf-8", errors="ignore")
    lines = [ln.strip() for ln in txt.splitlines() if ln.strip()]
    if not lines:
        raise ValueError(f"Empty file: {path}")
    try:
        vehicles, capacity = map(int, lines[0].split()[:2])
    except Exception as e:
        raise ValueError(f"Bad header in {path}: {lines[0]!r}") from e

    depot = None
    rows = []
    for ln in lines[1:]:
        parts = ln.split()
        if len(parts) < 9:
            continue
        row = [
            int(float(parts[0])),  # node_id
            int(float(parts[1])),  # x
            int(float(parts[2])),  # y
            int(float(parts[3])),  # demand
            int(float(parts[4])),  # tw_start
            int(float(parts[5])),  # tw_end
            int(float(parts[6])),  # service
            int(float(parts[7])),  # node_type
            int(float(parts[8])),  # request_id
        ]
        if row[7] == -1 and depot is None:
            depot = row
        else:
            rows.append(row)
    if depot is None:
        raise ValueError(f"No depot row (node_type=-1) in {path}")

    # group by request_id
    by_req = {}
    for r in rows:
        rid = r[8]
        if rid < 0:
            continue
        d = by_req.setdefault(rid, {"pickups": [], "delivery": None})
        if r[7] == 0:
            d["pickups"].append(r)
        elif r[7] == 1:
            d["delivery"] = r

    # build list, ensure both sides present
    reqs = []
    for rid, d in by_req.items():
        if not d["pickups"] or d["delivery"] is None:
            continue
        d["pickups"].sort(key=lambda rr: rr[0])  # stable order
        reqs.append(d)

    return vehicles, capacity, depot, reqs

def write_merged(out_path: Path, vehicles: int, capacity: int, depot_row, requests):
    """
    Paper-format TXT writer:
      - header
      - depot line (as given)
      - requests with RIDs 0..m-1, fresh node IDs 1.. (no collisions)
    """
    next_node_id = 1
    with out_path.open("w", encoding="utf-8") as f:
        f.write(f"{vehicles} {capacity}\n")
        f.write(" ".join(map(str, depot_row)) + "\n")
        for rid, req in enumerate(requests):
            # pickups
            for r in req["pickups"]:
                x, y, dmd, tws, twe, svc = r[1], r[2], r[3], r[4], r[5], r[6]
                f.write(f"{next_node_id} {x} {y} {dmd} {tws} {twe} {svc} 0 {rid}\n")
                next_node_id += 1
            # delivery
            r = req["delivery"]
            x, y, dmd, tws, twe, svc = r[1], r[2], r[3], r[4], r[5], r[6]
            f.write(f"{next_node_id} {x} {y} {dmd} {tws} {twe} {svc} 1 {rid}\n")
            next_node_id += 1

def main():
    files = sorted(BASE_DIR.glob(PATTERN))
    if len(files) < 5:
        raise SystemExit(f"Need at least 5 files matching {BASE_DIR}/{PATTERN}")

    # Deterministically pick the first 5 files
    chosen = files[:5]

    # Parse once
    parsed = []
    for fp in chosen:
        v, c, depot, reqs = parse_instance(fp)
        parsed.append((fp, v, c, depot, reqs))

    if TARGET_SIZE == 200:
        # Build a 5-cycle: (f0,f1),(f1,f2),(f2,f3),(f3,f4),(f4,f0)
        pairs = []
        for i in range(5):
            a = parsed[i]
            b = parsed[(i + 1) % 5]
            pairs.append((a, b))

        print("\n=== 200-NODE INSTANCE GENERATION (5-cycle, each file used twice) ===")
        for idx, ((fp1, v1, c1, dep1, reqs1), (fp2, v2, c2, dep2, reqs2)) in enumerate(pairs, start=1):
            print(f"Instance {OUTPUT_PREFIX_200}_{idx}:  {fp1.name}  +  {fp2.name}")

            vehicles = v1
            capacity = c1
            merged_reqs = reqs1 + reqs2
            rng.shuffle(merged_reqs)

            out_name = f"{OUTPUT_PREFIX_200}_{idx}.txt"
            out_path = BASE_DIR / out_name
            write_merged(out_path, vehicles, capacity, dep1, merged_reqs)

            print(f"  -> saved as {out_path.resolve()}\n")

    elif TARGET_SIZE == 400:
        # Your 400-size logic kept for completeness, unchanged
        print("Merging 5 combos of 4 files (each source excluded once):")
        for i in range(5):
            group = [parsed[j] for j in range(5) if j != i]  # exclude i-th
            excluded = parsed[i][0].name
            # Header/depot from the first file in the group
            fp0, v0, c0, dep0, reqs0 = group[0]
            vehicles, capacity = 60, c0
            # Combine requests from all 4 files and shuffle
            merged_reqs = []
            for (_, _, _, _, reqs) in group:
                merged_reqs.extend(reqs)
            rng.shuffle(merged_reqs)
            out_name = f"{OUTPUT_PREFIX_400}_{i+1}.txt"
            out_path = BASE_DIR / out_name
            write_merged(out_path, vehicles, capacity, dep0, merged_reqs)
            print(f"Saved {out_path.resolve()} (excluded {excluded})")

    else:
        raise SystemExit("TARGET_SIZE must be 200 or 400")

if __name__ == "__main__":
    main()
