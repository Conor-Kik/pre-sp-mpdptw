import os
import sys
import importlib
from typing import Dict

# Ensure src/ is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

REGISTRY: Dict[str, str] = {
    "col_gen": "mpdptw.methods.col_generation.route_generation:main",
    "col_gen_mt": "mpdptw.methods.col_generation.multi_thread_route_generation:main",
}


def load_entry(entry: str):
    module_path, func_name = entry.split(":")
    mod = importlib.import_module(module_path)
    try:
        func = getattr(mod, func_name)
    except AttributeError as e:
        raise SystemExit(f"Callable '{func_name}' not found in {module_path}") from e
    return func


def main():
    if len(sys.argv) < 3 or sys.argv[1] in {"-h", "--help"}:
        print("Usage: python cli.py <method> <instance_filename> [solver-args...] [--cap]")
        print("\nAvailable methods:")
        for m in REGISTRY.keys():
            print(f"  {m}")
        sys.exit(0)

    method = sys.argv[1]
    if method not in REGISTRY:
        print(f"Unknown method '{method}'. Known: {', '.join(REGISTRY.keys())}")
        sys.exit(2)

    solver_argv = sys.argv[2:]

    # Handle --mt (multi-thread)
    if method == "col_gen" and "--mt" in solver_argv:
        solver_argv.remove("--mt")
        method = "col_gen_mt"

    # --- New: handle --cap flag for route generation ---
    if method in {"col_gen", "col_gen_mt"} and "--cap" in solver_argv:
        solver_argv.remove("--cap")  # remove flag so solver doesn’t choke
        os.environ["ROUTE_GEN_CAP"] = "1"  # mark it via env var

    func = load_entry(REGISTRY[method])
    func(solver_argv)


if __name__ == "__main__":
    main()
