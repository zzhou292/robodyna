#!/usr/bin/env python3
"""Reuse P1 pinned source preparation and add exact friction/phase blocks."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parent
P1 = ROOT.parent.parent / "radioss_type25_normal/native/prepare.py"
spec = importlib.util.spec_from_file_location("type25_normal_source", P1)
normal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(normal)

def sources():
    for item in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        data = (ROOT.parent / item["path"]).read_bytes()
        assert len(data) == item["bytes"] and hashlib.sha256(data).hexdigest() == item["sha256"], item["path"]
    proof = json.loads((ROOT / "original/friction-controls.json").read_text())
    assert proof["status"] == "observed_at_first_actual_i25for3_entry"
    for key, value in {"MFROT": 2, "IFQ": 10, "IORTHFRIC": 0, "INTTH": 0,
                       "ALPHA0": 1, "VISCFFRIC": 0}.items():
        assert proof["argument_values"][key] == value
    assert proof["common_inconv"] == 1
    geometry = (ROOT / "original/i25dst3_3.F").read_text()
    assert "REAL*4 VTX_BISECTOR(3,2,*)" in geometry
    assert "NN1(I)= VTX_BISECTOR(1,1,IBX)" in geometry
    return normal.source()

def generate():
    donor = sources()
    p1_generated = normal.generate()
    constants = p1_generated.split("      TYPE :: FOREIGN_HISTORY", 1)[0].split("      INTEGER(C_INT), INTENT(OUT) :: TERMS\n", 1)[1]
    isotropic = donor.split("        ELSE\nC++ Orthotropic Friction", 1)[0]
    coefficient = normal.between(isotropic, "          ELSEIF(MFROT==2)THEN\nC---        Darmstad LAW", "          ELSEIF (MFROT==3) THEN")
    coefficient = coefficient.replace("ELSEIF", "IF", 1) + "          ENDIF\n"
    tangent_start = "            IF (INCONV==1) THEN\n              DO I=1,JLT\n                IF(PENE(I) == ZERO)CYCLE\n                FX = STIF0(I)*VX(I)*DT12"
    tangent = normal.between(donor, tangent_start, "C--------implicit non converge---") + "            ENDIF\n"
    phase_source = (ROOT / "original/i25irtlm.F").read_text()
    begin = normal.between(phase_source.split("      ELSE ! IVIS2 == -1", 1)[0], "        DO N=1,NSN\n", "!$omp end do")
    end = normal.between((ROOT / "original/i25mainf.F").read_text(),
        "        DO N=NSNFT, NSNLT         \n", "        DO N=NSNRFT, NSNRLT")
    outputs = {}
    for name, blocks in {"FrictionReference.F": {"CONSTANTS": constants, "COEFFICIENT": coefficient, "TANGENT": tangent},
                         "PhaseReference.F": {"CONSTANTS": constants, "BEGIN": begin, "END": end}}.items():
        template = "FrictionWrapper.F.in" if name.startswith("Friction") else "PhaseWrapper.F.in"
        text = (ROOT / template).read_text()
        for tag, block in blocks.items():
            assert text.count("@" + tag + "@") == 1
            text = text.replace("@" + tag + "@", block)
        outputs[name] = text
    from actual_fixture import header
    outputs["ActualInput.h"] = header(ROOT)
    return outputs

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("--output", type=Path); p.add_argument("--check", action="store_true")
    a = p.parse_args()
    for name, text in generate().items():
        if a.output:
            target = a.output / name
            if a.check: assert target.read_text() == text
            else:
                target.parent.mkdir(parents=True, exist_ok=True); target.write_text(text)
    print("Pinned TYPE25 coupled friction and row-phase source preparation verified")
