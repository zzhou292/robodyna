#!/usr/bin/env python3
"""Prepare complete native selection stages and explicit qualification boundaries."""
import argparse
from pathlib import Path
from Sources import ROOT, read, routine, constants
from ContinuationSource import routines as continuation_routines, observed as continuation_observed
from NewImpactSource import routines as impact_routines, observed as impact_observed


def generated():
    source = read()
    retained = {"CorRetained.F": routine(source["i25cor3.F"], "I25COR3_1"),
                "DstRetained.F": routine(source["i25dst3_1.F"], "I25DST3_1"),
                "GlobRetained.F": routine(source["i25dst3_1.F"], "I25GLOB_1")}
    # A read-only observation after every numerical operation exposes DD, which
    # the source initializes in all sectors but does not return in its ABI.
    observed = retained["DstRetained.F"]
    assert observed.count("      USE TRI7BOX\n") == 1
    assert observed.count("      RETURN\n") == 1
    observed = observed.replace("      USE TRI7BOX\n",
        "      USE TRI7BOX\n      USE SELECTION_OBSERVATIONS\n")
    observed = observed.replace("      RETURN\n",
        "      retained_distance_squared = DD\n      RETURN\n")
    output = dict(retained)
    output["DstRetained.F"] = observed
    continuation = continuation_routines(source)
    output.update(continuation_observed(continuation))
    impact = impact_routines(source)
    output.update(impact_observed(impact))
    output["Constants.F90"] = constants(source["constant_mod.F"],
        [*retained.values(), *continuation.values(), *impact.values()])
    for name in ["LocalBoundary.F90", "Observations.F90", "RetainedWrapper.F90"]:
        output[name] = (ROOT / name).read_text()
    output["implicit_f.inc"] = ("      USE ISO_C_BINDING\n      USE SELECTION_CONSTANTS\n"
                                "      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n")
    output["mvsiz_p.inc"] = "      INTEGER,PARAMETER :: MVSIZ=1\n"
    output["comlock.inc"] = ""
    output["vectorize.inc"] = ""
    return output


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
    print("Pinned complete retained, continuation and new-impact native stages prepared")
