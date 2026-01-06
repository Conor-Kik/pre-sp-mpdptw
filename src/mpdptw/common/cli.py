# src/mpdptw/common/cli.py
from __future__ import annotations
from pathlib import Path
import argparse, sys

def parse_instance_argv(
    argv=None,
    *,
    default_filename: str = "l_4_25_1.txt",
    default_instances_dir: str = "mpdtw_instances_2019",
):
    """Parse CLI, resolve instance path, exit with friendly message if missing."""
    p = argparse.ArgumentParser(description="Solve MPDPTW instance")
    p.add_argument(
        "filename",
        nargs="?",
        default=default_filename,
        help=f"Instance filename inside --instances-dir OR a full/relative path (default: {default_filename})",
    )
    p.add_argument(
        "--instances-dir",
        default=default_instances_dir,
        help=f"Directory containing instances (ignored if filename is a path) (default: {default_instances_dir})",
    )
    args = p.parse_args(argv)

    fname = Path(args.filename)
    if fname.is_absolute() or fname.parent != Path("."):
        path = fname
    else:
        path = Path(args.instances_dir) / fname

    path = path.resolve()
    if not path.exists():
        print(f"Instance not found: {path}", file=sys.stderr)
        print("   → Provide a valid path or adjust --instances-dir.", file=sys.stderr)
        sys.exit(1)

    return path, args
