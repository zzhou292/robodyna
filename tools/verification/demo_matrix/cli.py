"""Inspect demo coverage, prepare plans and explicitly compile guarded batches."""

import argparse
import collections
import json
from pathlib import Path

from tools.verification.demo_matrix.evidence import verify_build
from tools.verification.demo_matrix.inventory import read_inventory, refresh
from tools.verification.demo_matrix.plans import coverage, create_plan, read_matrix
from tools.verification.demo_matrix.query import PUBLIC_EXPRESSION, QUERY_EXPRESSION


def write_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    actions.add_parser("query-commands", help="Print the two read-only Bazel query arguments")
    action = actions.add_parser("refresh", help="Join pinned sources with completed query snapshots")
    action.add_argument("--repository", type=Path, required=True)
    action.add_argument("--query-xml", type=Path, required=True)
    action.add_argument("--public-labels", type=Path, required=True)
    action.add_argument("--query-receipt", type=Path)
    action.add_argument("--output", type=Path, required=True)
    action = actions.add_parser("status", help="Report source, declaration and matrix coverage")
    action.add_argument("--inventory", type=Path, required=True)
    action.add_argument("--matrix", type=Path, required=True)
    action.add_argument("--build-root", type=Path, help="Also verify closed per-target build receipts")
    action = actions.add_parser("plan", help="Create one explicit compile-only plan; never launch it")
    for name in ("inventory", "matrix", "environment", "output", "run-directory"):
        action.add_argument("--" + name, type=Path, required=True)
    action.add_argument("--batch", required=True)
    action = actions.add_parser("record-build", help="Verify closed guard and completed-target BEP evidence")
    for name in ("plan", "guard", "bep", "output"):
        action.add_argument("--" + name, type=Path, required=True)
    action = actions.add_parser("build-all", help="Compile all admitted batches sequentially under the existing guard")
    action.add_argument("--matrix", type=Path, required=True)
    action.add_argument("--environment", type=Path, required=True)
    action.add_argument("--output", type=Path, required=True)
    action.add_argument("--batch", help="Run one named batch instead of the full source denominator")
    action.add_argument("--resume", action="store_true", help="Verify and resume the same immutable source/environment request")
    args = parser.parse_args()
    if args.action == "query-commands":
        print(json.dumps([["query", QUERY_EXPRESSION, "--output=xml", "--noimplicit_deps"],
                          ["query", PUBLIC_EXPRESSION, "--output=label", "--noimplicit_deps"]], indent=2))
    elif args.action == "refresh":
        value = refresh(args.repository, args.query_xml, args.public_labels, args.query_receipt)
        write_new(args.output, value)
        print(json.dumps({"source_counts": value["counts"], "declared_counts": value["declaration_counts"]}, indent=2))
    elif args.action == "status":
        value, matrix = read_inventory(args.inventory), read_matrix(args.matrix)
        result = {"source_counts": value["counts"], "declared_counts": value["declaration_counts"], "coverage": coverage(value, matrix)}
        if args.build_root:
            from tools.verification.demo_matrix.status import build_status
            result["recorded_builds"] = build_status(args.build_root)
        else:
            result["build_status"] = "Discovery only; supply --build-root to read verified compilation receipts"
        print(json.dumps(result, indent=2))
    elif args.action == "plan":
        value = create_plan(read_inventory(args.inventory), read_matrix(args.matrix), args.batch,
                            json.loads(args.environment.read_text()), args.inventory, args.matrix, args.run_directory)
        write_new(args.output, value)
        print("Created compile-only plan:", args.output)
    elif args.action == "record-build":
        write_new(args.output, verify_build(json.loads(args.plan.read_text()), args.guard, args.bep))
    else:
        from tools.verification.demo_matrix.executor import execute
        value = execute(args.matrix, args.environment, args.output, args.batch, args.resume)
        print(json.dumps({"status": value["status"], "batches": len(value["batches"])}))


if __name__ == "__main__":
    main()
