# MPDPTW – Exact Pruned Enumeration with Set Partitioning (PRE-SP)

This repository provides an exact solver for the Multi-Pickup and Delivery Problem with Time Windows (MPDPTW) based on a Pruned Route Enumeration with Set-Partitioning (PRE-SP) framework.
Unlike traditional mixed-integer formulations or column-generation approaches, PRE-SP enumerates feasible single-vehicle routes a priori, applies provably safe pruning rules, and then solves a set-partitioning master problem over the reduced route set to obtain a globally optimal solution.

---

## Method Overview

The PRE-SP framework consists of three main stages:

### 1. Preprocessing and Time-Window Tightening
Instance data is refined by tightening service windows and pruning infeasible arcs to reduce search complexity.

### 2. Pruned Route Enumeration
Feasible single-vehicle routes are generated using a route-feasibility subproblem:
- Time-window feasibility enforced  
- Pickup–delivery precedence respected  
- (Capacitated variant) load feasibility enforced through a dynamic re-checking process  

Route enumeration is multi-threaded by default to accelerate feasibility checking and pruning.

Two pruning principles guarantee efficiency without sacrificing optimality:
- Infeasibility Propagation — if a subset of requests is infeasible, all its supersets are discarded.
- Horizon-Based Pruning — subsets that cannot fit within the global time horizon are eliminated.

### 3. Set-Partitioning Master Problem
The resulting feasible routes form columns in a set-partitioning model that selects a minimum-cost set of routes covering all requests exactly once.

Capacity is enforced dynamically: routes chosen by the master problem are re-evaluated under full capacity constraints and corrected if necessary.

---

## Project Structure

```
MATH3205_MPDPTW_Project/
├── src/
│   └── mpdptw/
│       ├── common/                 # IO utilities, preprocessing, helpers
│       └── methods/
│           └── PRE_SP/     
│               └── multi_thread_route_generation.py
│
├── mpdtw_instances_2019/           # Benchmark instance files
├── routes/                         # Optimal route solutions
├── cli.py                          # CLI entrypoint
├── results.xlsx                    # Reported Results
└── README.txt
```

---

## Running the Solver (CLI)

The entry point is cli.py.

### General Usage
```bash
python cli.py pre_sp <instance_filename> [options]
```

### Arguments
- <instance_filename>  file in mpdtw_instances_2019/
- [options]
  - --cap enable capacity verification / correction

Multi-threaded route enumeration is enabled by default.

---

### Examples
```bash
python cli.py pre_sp l_4_25_1.txt
python cli.py pre_sp w_8_100_4.txt
python cli.py pre_sp n_8_200_4.txt --cap
```

---

## Benchmarking and Performance

This solver has been evaluated on the 120 classical MPDPTW benchmarks and additional 200-node synthetic instances.

---

## Environment

Verified with:
- Python 3.12.9
- Gurobi 12.0.1
- psutil 5.9.0

Ensure Gurobi is installed and properly licensed.

---

## Notes

- Always execute from the project root to ensure relative paths resolve.
- For reproducibility, use the recommended software versions above.
- Parallel execution is automatically enabled when beneficial.

---

## Reference

This repository accompanies ongoing research on an exact route-enumeration
framework (PRE-SP) for the Multi-Pickup and Delivery Problem with Time Windows.
A full manuscript is currently in preparation and has not yet been submitted.
Details and results may therefore evolve.

Benchmark datasets:
Leandro C. Coelho — Multi-Pickup and Delivery Dataset  
https://www.leandro-coelho.com/multi-pickup-and-delivery/
