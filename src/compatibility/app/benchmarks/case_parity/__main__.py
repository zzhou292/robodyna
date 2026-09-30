"""Thin create-only entry point; no simulation, GPU initialization or rendering."""
import argparse
import json
from pathlib import Path

from .performance import assess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("request", type=Path)
    parser.add_argument("--sha256", required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.report.exists() or args.report.is_symlink():
        raise FileExistsError("Preserve existing report; use a new output path")
    result = assess({"path": str(args.request.resolve()), "sha256": args.sha256}, Path.cwd())
    payload = json.dumps(result, indent=2, allow_nan=False)
    if len(payload.encode()) > 16 << 20:
        raise ValueError("Parity report exceeds 16 MiB cap")
    with args.report.open("x", encoding="utf-8") as output:
        output.write(payload + "\n")
    print(result["status"])
    return 0  # Report creation succeeded; only the explicit verdict can admit a claim.


if __name__ == "__main__":
    raise SystemExit(main())
