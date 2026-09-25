#!/usr/bin/env python3
"""Prepare complete native sliding routines and phase membership snapshots."""
import argparse
from pathlib import Path
from Sources import ROOT, read, constants
from SlidingSource import routines, memberships


def generated():
    source = read()
    original = routines(source)
    result = {}
    for name, text in original.items():
        # Only the foreign-symbol compilation boundary is renamed. All native
        # declarations, loops, expressions, tests, writes and returns remain.
        assert text.count("      USE TRI7BOX\n") == 1
        result[name] = text.replace("      USE TRI7BOX\n", "      USE LIFECYCLE_FOREIGN\n")
    result["LifecycleConstants.F90"] = constants(source["constant_mod.F"], original.values()).replace(
        "module selection_constants", "module lifecycle_constants")
    continuation, impact = memberships(source)
    template = (ROOT / "LifecycleMembership.F.in").read_text()
    for name, block in [("CONTINUATION", continuation), ("NEW_IMPACT", impact)]:
        marker = "@" + name + "@"
        assert template.count(marker) == 1
        template = template.replace(marker, block)
    result["LifecycleMembership.F"] = template
    for name in ["LifecycleBoundary.F90", "LifecycleMemory.F90", "LifecycleSliding.F90"]:
        result[name] = (ROOT / name).read_text()
    result["implicit_f.inc"] = ("      USE ISO_C_BINDING\n      USE LIFECYCLE_CONSTANTS\n"
                                "      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n")
    result["comlock.inc"] = ""
    result["task_c.inc"] = "      INTEGER ISPMD\n      COMMON /LIFECYCLE_PROCESS/ ISPMD\n"
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, text in generated().items():
        if args.output:
            target = args.output / name
            if args.check:
                assert target.read_text() == text
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(text)
    print("Pinned complete sliding routines and original COMP_2 memberships prepared")
