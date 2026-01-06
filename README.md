# MPDPTW Column Generation Solver

This repository contains an implementation and experimental framework for solving  the **Multi-Pickup and Delivery Problem with Time Windows (MPDPTW)** using a **column generation–based approach**.

------------------------------------------------------------------------

## Project Structure

```
MATH3205_MPDPTW_Project/
├── src/
│   └── mpdptw/
│       ├── common/                 # Shared utilities (parsers, printers, etc.)
│       └── methods/                # Solution methods (currently: column generation)
│           └── col_gen/
│               └── ColGenSolver.py
│
├── mpdtw_instances_2019/           # Benchmark instance files (.txt)
├── docs/                           # Project report and results
├── cli.py                          # CLI entrypoint
└── README.txt                      # This file
```

------------------------------------------------------------------------

## Running the Project (CLI)

The entry point is `cli.py` at the project root.

### General usage

```bash
python cli.py col_gen <instance_filename> [solver-args...]
```

- `<instance_filename>` → the instance file located inside
  `mpdtw_instances_2019/`
- `[solver-args...]` → optional arguments for the column generation
  solver

------------------------------------------------------------------------

## Available Methods

Currently implemented:

- **`col_gen`** → Column Generation approach  
  - Add `--mt` after the instance filename to enable
    **multi-threaded column generation**  
  - Add `--cap` to enforce **capacity constraints** in the master
    and pricing problems  

All solvers are contained under `src/mpdptw/methods/`.

------------------------------------------------------------------------

## Example Commands

```bash
python cli.py col_gen l_4_25_1.txt
python cli.py col_gen w_8_100_4.txt --mt
python cli.py col_gen w_8_100_4.txt --mt --cap
```

------------------------------------------------------------------------

## Environment

This project was developed and tested using the following versions:

- **Python:** 3.12.9  
- **Gurobi (gurobipy):** 12.0.1  
- **psutil:** 5.9.0  

These versions are recommended for full reproducibility.

------------------------------------------------------------------------

## Notes

- Always run from the **project root** so paths resolve correctly.
- The project uses **Gurobi** as the solver — ensure it is installed
  and licensed.

------------------------------------------------------------------------

## References

- This project builds on the work of  
  **Aziez, Côté, and Coelho (2020)**  
  *Exact algorithms for the multi-pickup and delivery problem with
  time windows*,  
  *European Journal of Operational Research, 284(3)*, pp. 906–919.

- The **benchmark MPDPTW instances** used in this project are sourced from:  
  **Leandro C. Coelho — Multi-Pickup and Delivery Dataset**  
  <https://www.leandro-coelho.com/multi-pickup-and-delivery/>