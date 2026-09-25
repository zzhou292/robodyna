#!/usr/bin/env python3
"""Qualification-only native coefficient blocks; no production numeric calls."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parent
P1 = ROOT.parent.parent / "radioss_type25_normal/native/prepare.py"
spec = importlib.util.spec_from_file_location("coefficient_normal_source", P1)
normal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(normal)


def sources():
    normal.source()
    result = {}
    for item in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        path = ROOT.parent / item["path"]
        data = path.read_bytes()
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise RuntimeError("Pinned coefficient donor changed: " + item["path"])
        result[path.name] = data.decode()
    proof = json.loads(result["coefficient-controls.json"])
    observation = next(x for x in proof["observations"] if x["stage"] == "COR3_entry")
    for key, value in {"IGSTI": 4, "ISTIF_MSDT": 0, "KMIN": 0, "KMAX": 1e30}.items():
        assert observation["controls"][key] == value
    assert observation["row"]["main_stiffness"] == 2800
    assert observation["row"]["signed_secondary_stiffness"] == 2410
    return result


def constants(text):
    names = {"ZERO", "ONE", "TWO", "THREE", "FOUR", "EIGHT", "SIXTEEN", "TEN", "HUNDRED",
             "THIRD", "ONE_OVER_8", "EM30", "EP30"}
    names.update("EP%02d" % i for i in range(2, 21))
    rows = []
    for line in text.splitlines():
        match = re.search(r"my_real, parameter ::\s*(\w+)\s*=", line)
        if match and match[1] in names:
            rows.append(line.replace("my_real", "REAL(C_DOUBLE)", 1))
    assert len(rows) == len(names)
    return "\n".join(rows)


def generate():
    donor = sources()
    gapm = donor["i25sti3.F"].split("      SUBROUTINE I25GAPM(", 1)[1]
    blocks = {}
    for tag, index in [("QUAD", "NELC"), ("TRIANGLE", "NUMELC+NELTG")]:
        start = "              IF ( THK(" + index + ") /= ZERO .AND. IINTTHICK == 0) THEN"
        # The first matching sequence here is within the ordinary PM20 branch;
        # the preceding special branches use GEO702 or PM_STACK instead.
        ordinary = start + gapm.split(start)[-1]
        block = ordinary.split("            ENDIF\nC\n            STF(I)=MAX(STF(I),STC)", 1)[0]
        assert "PM(20,MT)" in block and "PM_STACK" not in block and "GEO(IPGMAT" not in block
        blocks[tag] = block + "            STF(I)=MAX(STF(I),STC)\n"
    solid = normal.between(gapm, "            IF (ICONTR==1 ) THEN\n", "            IF(IELEM_M(2,I) > 0) THEN")
    corrections = normal.between(gapm, "          IF(NELS>NUMELS8.AND.NELS<=NUMELS8+NUMELS10)THEN\n", "C -----Friction model ------")
    blocks["SOLID"] = solid + "          GAP_N(1,I)=VOL/AREA\n" + corrections
    blocks["NODAL"] = normal.between(donor["asstifi.F"], "      DO N=1,NUMNOD\n", "C\n      RETURN")
    blocks["SECONDARY"] = normal.between(donor["i25stslav.F"], "      NSN   =IPARI(5)\n", "C\n      RETURN")
    cor = donor["i25cor3.F"].split("      SUBROUTINE I25COR3_3(", 1)[1]
    pair = normal.between(cor, "      ELSEIF(IGSTI==4.OR.IGSTI==6)THEN\n", "      ELSEIF(IGSTI==5)THEN")
    blocks["PAIR"] = pair.replace("ELSEIF", "IF", 1) + "      ENDIF\n"
    blocks["CLAMP"] = normal.between(cor,
        "      DO I=1,JLT\n         STIF(I)=MAX(KMIN,MIN(STIF(I),KMAX))\n", "C----------")
    outputs = {}
    for name, tags in [("MainReference.F", ["QUAD", "TRIANGLE", "SOLID"]),
                       ("NodalReference.F", ["NODAL", "SECONDARY"]),
                       ("PairReference.F", ["PAIR", "CLAMP"])]:
        text = (ROOT / (name + ".in")).read_text()
        text = text.replace("@CONSTANTS@", constants(donor["constant_mod.F"]))
        for tag in tags:
            assert text.count("@" + tag + "@") == 1
            text = text.replace("@" + tag + "@", blocks[tag])
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
    print("Pinned TYPE25 coefficient expressions and captured pair controls verified")
