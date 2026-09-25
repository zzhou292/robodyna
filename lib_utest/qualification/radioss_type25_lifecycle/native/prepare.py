#!/usr/bin/env python3
"""Prepare complete native OPTCD/sliding routines and lifecycle phase snapshots."""
import argparse
from pathlib import Path
from Sources import ROOT, read, constants
from SlidingSource import routines, memberships, phase_blocks
from OptimizedSource import original as original_optcd, observed as observed_optcd, release


def generated():
    source = read()
    original = routines(source)
    result = {"OptimizedCandidates.F": observed_optcd(original_optcd(source))}
    for name, text in original.items():
        # Only the foreign-symbol compilation boundary is renamed. All native
        # declarations, loops, expressions, tests, writes and returns remain.
        assert text.count("      USE TRI7BOX\n") == 1
        result[name] = text.replace("      USE TRI7BOX\n", "      USE LIFECYCLE_FOREIGN\n")
    result["LifecycleConstants.F90"] = constants(source["constant_mod.F"], [*original.values(), original_optcd(source)]).replace(
        "module selection_constants", "module lifecycle_constants")
    continuation, impact = memberships(source)
    template = (ROOT / "LifecycleMembership.F.in").read_text()
    for name, block in [("CONTINUATION", continuation), ("NEW_IMPACT", impact)]:
        marker = "@" + name + "@"
        assert template.count(marker) == 1
        template = template.replace(marker, block)
    result["LifecycleMembership.F"] = template
    clear, finish = phase_blocks(source)
    phases = (ROOT / "LifecyclePhases.F.in").read_text()
    for name, block in [("CLEAR_SLIDING", clear), ("FINISH_MARKERS", finish)]:
        marker = "@" + name + "@"
        assert phases.count(marker) == 1
        phases = phases.replace(marker, block)
    release_block = release(source)
    assert phases.count("@RELEASE_MAIN@") == 1
    phases = phases.replace("@RELEASE_MAIN@", release_block)
    result["LifecyclePhases.F"] = phases
    for name in ["LifecycleBoundary.F90", "LifecycleMemory.F90", "LifecycleSliding.F90", "LifecycleObservation.F90", "LifecycleOptimized.F90"]:
        result[name] = (ROOT / name).read_text()
    result["implicit_f.inc"] = ("      USE ISO_C_BINDING\n      USE LIFECYCLE_CONSTANTS\n"
                                "      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n")
    result["comlock.inc"] = ""
    result["mvsiz_p.inc"] = "      INTEGER,PARAMETER :: MVSIZ=128\n"
    result["scr05_c.inc"] = "      INTEGER IRESP\n      COMMON /LIFECYCLE_PRECISION/ IRESP\n"
    result["com01_c.inc"] = ("      INTEGER NSPMD,NTHREAD,NVSIZ\n"
                            "      COMMON /LIFECYCLE_SCHEDULE/ NSPMD,NTHREAD,NVSIZ\n")
    result["com08_c.inc"] = "      REAL(C_DOUBLE) DT1\n      COMMON /LIFECYCLE_CLOCK/ DT1\n"
    result["param_c.inc"] = ""
    result["parit_c.inc"] = "      INTEGER LSKYI_COUNT\n      COMMON /LIFECYCLE_COUNTER/ LSKYI_COUNT\n"
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
    print("Pinned complete OPTCD/sliding routines and original lifecycle phases prepared")
