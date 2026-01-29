import os
import sys
import importlib
from typing import Dict


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

# Multi-thread route generation is now the default for pre_sp.
REGISTRY: Dict[str, str] = {
    "pre_sp": "mpdptw.methods.PRE_SP.multi_thread_route_generation:main",
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

    # Handle --cap flag
    if method == "pre_sp" and "--cap" in solver_argv:
        solver_argv.remove("--cap")
        os.environ["ROUTE_GEN_CAP"] = "1"

    func = load_entry(REGISTRY[method])
    func(solver_argv)


if __name__ == "__main__":
    main()
