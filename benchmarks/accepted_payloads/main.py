"""Create an exact comparison receipt; does not run or modify a simulation."""
import argparse
import json
from pathlib import Path

from compare import compare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--expected-host-delta", type=int, default=0)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.baseline, args.candidate, args.expected_host_delta)
    with args.report.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"PASS: {result['archive_files']} exact archive files and all non-timing summary values")


if __name__ == "__main__":
    main()
