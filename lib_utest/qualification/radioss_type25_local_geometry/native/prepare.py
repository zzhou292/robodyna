#!/usr/bin/env python3
"""Extract pinned native geometry expressions, never the C++ translation."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parent
P1 = ROOT.parent.parent / "radioss_type25_normal/native/prepare.py"
spec = importlib.util.spec_from_file_location("normal_geometry_source", P1)
normal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(normal)


def sources():
    normal.source()
    result = {}
    for item in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        path = ROOT.parent / item["path"]
        data = path.read_bytes()
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise RuntimeError("Pinned geometry donor changed: " + item["path"])
        result[path.name] = data.decode()
    return result


def constants(text):
    # Preserve original declaration order and the actual dependency expressions.
    names = {"ZERO", "ONE", "TWO", "FOUR", "TEN", "HUNDRED", "HALF", "FOURTH",
             "EM03", "EM04", "EM20", "EM30", "EP30"}
    names.update("EP%02d" % i for i in range(2, 21))
    declarations = []
    for line in text.splitlines():
        match = re.search(r"my_real, parameter ::\s*(\w+)\s*=", line)
        if match and match[1] in names:
            declarations.append(line.replace("my_real", "REAL(C_DOUBLE)", 1))
    assert len(declarations) == len(names)
    return "\n".join(declarations)


def observe_defined_point(text):
    # Only observe assignment readiness. Do not seed undefined XP with geometry.
    # Each source XP/YP/ZP triple is complete when its ZP assignment executes.
    output = []
    assignments = reads = 0
    for line in text.splitlines():
        if re.match(r"\s*XC=XP\(I\)\+GAPM\*NX\(I\)", line):
            output.extend(["            IF(.NOT.RD_POINT_READY(I))THEN",
                           "              DEFINED=0", "              RETURN", "            ENDIF"])
            reads += 1
        output.append(line)
        if re.match(r"\s*ZP\(I\)\s*=LA\(I\)\*ZZ\(I,5\)", line):
            output.append("            RD_POINT_READY(I)=.TRUE.")
            assignments += 1
    assert assignments == 4 and reads == 1
    return "\n".join(output) + "\n"


def generate():
    source = sources()
    cor = source["i25cor3.F"].split("      SUBROUTINE I25COR3_3(", 1)[1]
    center = normal.between(cor, "           IF(IX3(I) /= IX4(I))THEN\n", "C\n           GAPNM")
    normals = normal.between(cor, "      DO I=1,JLT\nC\n        L  = CAND_E(I)\nC\n        NNX(I,1)", "      IF(IGSTI<=1)THEN")
    dst = source["i25dst3_3.F"]
    raw = normal.between(dst, "      EPSEG = (TWO+HALF)/HUNDRED\n", "C      PENE_OLD(5) <=> Initial Penetration")
    offsets = normal.between(dst, "      IF (TIME==ZERO) THEN\n", "      ELSE ! IVIS2==-1 (Adhesion case)")
    offsets += "      ENDIF\n"
    main = source["i25mainf.F"]
    stiffness = normal.between(main,
        "          DO I = 1 ,JLT\nC\nC       Needs to compute STIF_OLD even if PENE ==0 (cf INACTI=5)",
        "          IF(INTTH==0.AND.JLT_NEW == 0")
    outputs = {}
    for name, template, blocks in [
        ("GeometryReference.F", "RawWrapper.F.in", {"CENTER": center, "NORMALS": normals,
             "GEOMETRY": observe_defined_point(raw)}),
        ("GeometryHistoryReference.F", "HistoryWrapper.F.in", {"OFFSETS": offsets, "STIFFNESS": stiffness})]:
        text = (ROOT / template).read_text()
        blocks["CONSTANTS"] = constants(source["constant_mod.F"])
        for tag, block in blocks.items():
            assert text.count("@" + tag + "@") == 1
            text = text.replace("@" + tag + "@", block)
        outputs[name] = text
    return outputs


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, text in generate().items():
        if args.output:
            target = args.output / name
            if args.check:
                assert target.read_text() == text
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(text)
    print("Pinned TYPE25 local geometry and two-pass history source verified")
