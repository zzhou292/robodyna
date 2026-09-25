#!/usr/bin/env python3
"""Prepare an unmodified pinned I25PEN3 routine and its bounded ABI wrapper."""
import argparse
import hashlib
import json
import re
from pathlib import Path
HERE = Path(__file__).resolve().parent

def generated():
    source = {}
    for entry in json.loads((HERE / "source-manifest.json").read_text())["files"]:
        path = HERE.parent / entry["path"]
        data = path.read_bytes()
        assert len(data) == entry["bytes"] and hashlib.sha256(data).hexdigest() == entry["sha256"]
        source[path.name] = data.decode()
    names = {"ZERO", "ONE", "TWO", "FOUR", "TEN", "HUNDRED", "FOURTH", "EM03", "EM20", "EM30", "EP30", "ONEP01", "ZEP01", "EM02"}
    names.update("EP%02d" % i for i in range(2, 21))
    declarations = []
    for line in source["constant_mod.F"].splitlines():
        found = re.search(r"my_real, parameter ::\s*(\w+)\s*=", line)
        if found and found[1] in names:
            declarations.append(line.replace("my_real", "real(c_double)", 1))
    assert len(declarations) == len(names)
    routine = source["i25pen3.F"]
    routine = routine[routine.index("      SUBROUTINE I25PEN3("):]
    search = source["i25trivox.F"]
    bounds = search[search.index("          XX1=X(1,M1)"):search.index("c        index of voxels occupied by the facet")]
    pair_begin = search.index("                    XS = X(1,NN)")
    pair = search[pair_begin:search.index("                  ELSE",pair_begin)]
    screen_begin = search.index("                  IF(XS<=XMINE-AAA)")
    screen = search[screen_begin:search.index("                  J_STOK = J_STOK + 1",screen_begin)]
    wrapper = (HERE / "ScreenWrapper.F.in").read_text().replace("@@BOUNDS@@",bounds).replace("@@PAIR@@",pair).replace("@@SCREEN@@",screen)
    return {
        "Constants.F90": "module pen3_constants\n use iso_c_binding\n implicit none\n" +
            "\n".join(declarations) + "\nend module\n",
        "NativePen3.F": routine,
        "NativeScreen.F": wrapper,
        "NativeCor3t.F": "      MODULE COR3T_REFERENCE\n      CONTAINS\n" + source["i25cor3t.F"][source["i25cor3t.F"].index("      SUBROUTINE I25COR3T"):] + "      END MODULE\n",
        "PackingWrapper.F90": (HERE / "PackingWrapper.F90").read_text(),
        "Tri7Box.F90": "module tri7box\n use iso_c_binding\n real(c_double) :: xrem(10,1)=0\n integer :: irem(7,1)=0\nend module\n",
        "com08_c.inc": "      REAL(C_DOUBLE) DT1\n      COMMON /QUAL_COR3T_DT/ DT1\n",
        "Wrapper.F90": (HERE / "Wrapper.F90").read_text(),
        "implicit_f.inc": "      USE ISO_C_BINDING\n      USE PEN3_CONSTANTS\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n",
        "mvsiz_p.inc": "      INTEGER, PARAMETER :: MVSIZ=2\n",
    }

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
    print("Pinned whole PEN3 and explicit two-row ABI verified")
