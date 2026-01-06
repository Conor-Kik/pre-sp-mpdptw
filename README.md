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
│           └── pre_sp/             # PRE-SP solver implementation
│               └── ColGenSolver.py # (legacy naming, contains PRE-SP core logic)
│
├── mpdtw_instances_2019/           # Benchmark instance files
├── docs/                           # Report, figures, and results
├── cli.py                          # CLI entrypoint
└── README.txt
```

Note: although the directory name references “col_gen”, the implementation corresponds to the PRE-SP algorithm described in the manuscript.

---

## Running the Solver (CLI)

The entry point is cli.py.

### General Usage
```bash
python cli.py col_gen <instance_filename> [options]
```

Arguments:
- <instance_filename>  file in mpdtw_instances_2019/
- [options]
  - --mt enable multi-threaded enumeration
  - --cap enable capacity verification / correction

### Examples
```bash
python cli.py col_gen l_4_25_1.txt
python cli.py col_gen w_8_100_4.txt --mt
python cli.py col_gen w_8_100_4.txt --mt --cap
```

---

## Benchmarking and Performance

This solver has been evaluated on the 120 classical MPDPTW benchmarks and additional 200-node synthetic instances.

Key outcomes reported in the manuscript:
- Solves 116 / 120 benchmark instances to proven optimality
- Particularly strong on wide and weakly structured time-window cases (W-type) where classical MIP approaches struggle
- Successfully solves multiple 200-node instances, demonstrating scalability when structural pruning is effective

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

If using this solver, please cite the associated paper:

An Exact Pruned Enumeration Method for Multi-Pickup and Delivery Problems with Time Windows  
Conor Kikkert, University of Queensland  
Proposes the PRE-SP framework: pruned single-vehicle route enumeration + set-partitioning master optimization with demonstrated benchmark superiority.

Benchmark datasets:
Leandro C. Coelho — Multi-Pickup and Delivery Dataset  
https://www.leandro-coelho.com/multi-pickup-and-delivery/
