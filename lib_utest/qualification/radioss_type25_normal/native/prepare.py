#!/usr/bin/env python3
"""Compile pinned native blocks; no production translation participates in oracle."""
import argparse
import hashlib
import json
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parent

def source():
    for item in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        data = (ROOT.parent / item["path"]).read_bytes()
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise RuntimeError("Pinned donor changed: " + item["path"])
    controls = json.loads((ROOT / "original/engine-controls.json").read_text())
    assert controls["status"] == "observed_at_first_actual_i25for3_entry"
    assert controls["controls"] == {"kdtint": 0, "idtmins": 0, "idtmins_int": 0}
    log = (ROOT / "original/engine-controls.log").read_text()
    assert "RD_CONTROL_FLAGS KDTINT=0 IDTMINS=0 IDTMINS_INT=0" in log
    assert "Double Precision Version" in log
    resolved = json.loads((ROOT / "original/starter-manifest.json").read_text())["native_values"]
    assert resolved["stiffness_formulation"] == 4
    assert resolved["quadratic_damping_flag"] == 1
    assert resolved["normal_damping_factor"] == 0.05
    assert resolved["initial_penetration_mode"] == 5
    return (ROOT / "original/i25for3.F").read_text()

def between(text, start, stop):
    assert text.count(start) == 1, start
    tail = text.split(start, 1)[1]
    assert stop in tail
    return start + tail.split(stop, 1)[0]

def generate():
    donor = source()
    constants = (ROOT / "original/constant_mod.F").read_text()
    names = {"ZERO", "ONE", "TWO", "TEN", "HUNDRED", "HALF", "EM10", "EM30", "EP30"}
    names.update("EP%02d" % i for i in range(2, 21))
    declarations = []
    for line in constants.splitlines():
        match = re.search(r"my_real, parameter ::\s*(\w+)\s*=", line)
        if match and match[1] in names:
            declarations.append(line.replace("my_real", "REAL(C_DOUBLE)", 1))
    assert len(declarations) == len(names)
    zero = between(donor, "          H1(I) = ZERO\n", "        ELSEIF(PENE(I) == ZERO)THEN")
    history = between(donor, "      DO I=1,JLT\nC\n        IF(PENE(I) == ZERO) CYCLE\nC\n        DPENE(I)", "C-------------------------------------------\n      IF (NCFIT >0) THEN")
    spring = between(donor, "      ELSEIF(IVIS2/=-1)THEN ! NO ADHESION CASE", "      ELSE ! ADHESION CASE")
    spring = spring.replace("ELSEIF", "IF", 1) + "      ENDIF\n"
    energy = between(donor, "      DO I=1,JLT\n        IF(PENE(I) == ZERO)CYCLE\n        ECONTT", "C---------------------------------\nC     DAMPING + FRIC")
    damping = between(donor, "      FF(1:JLT)=ZERO", "C---------------------------------\nC     Energy absorbed (damping)")
    # Observation-only assignment, immediately after the donor's scalar C/FF
    # coefficient assignment and before FF is overwritten as a signed force.
    lines = []
    for line in damping.splitlines():
        lines.append(line)
        if re.match(r"\s*FF\(I\)\s*=\s*FAC\s*\*\s*VISC\s*\*\s*VIS\s*$", line):
            lines.append("              OBS_C(I)=FF(I)")
        elif re.match(r"\s*C\(I\)\s*=\s*FAC\s*\*\s*VISC\s*\*\s*VIS\s*$", line):
            lines.append("              OBS_C(I)=C(I)")
    damping = "\n".join(lines) + "\n"
    work = between(donor, "      DO I=1,JLT\n        IF(PENE(I) == ZERO)CYCLE\n        JG = NSVG(I)\n        IF(JG > 0)THEN\n          N  = CAND_N_N(I)\n          ECONTDT", "C---------------------------------\nC     SAUVEGARDE")
    result = (ROOT / "Wrapper.F.in").read_text()
    for tag, block in {"CONSTANTS": "\n".join(declarations), "ZERO": zero,
                       "HISTORY": history, "SPRING": spring, "ENERGY": energy,
                       "DAMPING": damping, "WORK": work}.items():
        assert result.count("@" + tag + "@") == 1
        result = result.replace("@" + tag + "@", block)
    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    text = generate()
    if args.output:
        if args.check:
            assert args.output.read_text() == text
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text)
    print("Pinned native TYPE25 block preparation verified")
