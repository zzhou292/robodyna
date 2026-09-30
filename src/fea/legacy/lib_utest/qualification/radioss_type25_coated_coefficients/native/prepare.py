#!/usr/bin/env python3
"""Native source-only oracle: actual ordinary-material coating branches."""
from pathlib import Path
import argparse
import importlib.util
ROOT = Path(__file__).resolve().parent
BASE = ROOT.parent.parent / "radioss_type25_coefficients/native/prepare.py"
spec = importlib.util.spec_from_file_location("pinned_coefficients", BASE)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)

def generate():
    donor = base.sources()  # Complete existing source digest/control authentication.
    gapm = donor["i25sti3.F"].split("      SUBROUTINE I25GAPM(", 1)[1]
    blocks = {}
    for name, index in [("QUAD", "NELC"), ("TRIANGLE", "NUMELC+NELTG")]:
        marker = "              IF ( THK(" + index + ") /= ZERO .AND. IINTTHICK == 0) THEN"
        pos = gapm.rindex(marker)
        # Keep the real MAX and complete encoded-partner write block, including
        # its disabled metadata branches; no reimplementation in expected math.
        stop = gapm.index("C\n          ELSE\n", pos)
        branch_end = gapm.index("            ENDIF\nC\n            STF(I)=MAX(STF(I),STC)", pos)
        assignment = gapm.index("            STF(I)=MAX(STF(I),STC)", branch_end)
        # Omit only the enclosing special-material IF terminator: the selected
        # ordinary arm and downstream assignment block remain verbatim.
        blocks[name] = gapm[pos:branch_end] + gapm[assignment:stop]
        assert "STF(I)=MAX(STF(I),STC)" in blocks[name]
        assert "STF(J)  = STC" in blocks[name]
        assert "IF(J > NRTT) J=J-NRTT" in blocks[name]
        assert "PM_STACK" not in blocks[name] and "GEO(IPGMAT" not in blocks[name]
    solid_start = gapm.index("            IF (ICONTR==1 ) THEN\n")
    solid_stop = gapm.index("            IF(IELEM_M(2,I) > 0) THEN", solid_start)
    blocks["SOLID"] = gapm[solid_start:solid_stop]
    marker = "          GAP_N(1,I)=VOL/AREA\n"
    assert gapm.count(marker) == 1
    blocks["LENGTH"] = marker
    out = (ROOT / "Coating.F.in").read_text().replace("@CONSTANTS@", base.constants(donor["constant_mod.F"]))
    for name, value in blocks.items():
        assert out.count("@"+name+"@") == 1
        out = out.replace("@"+name+"@", value)
    return out

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    text = generate()
    if args.output:
        path = args.output / "Coating.F"
        if args.check:
            assert path.read_text() == text
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
    print("Pinned I25GAPM solid/shell MAX and encoded partner writes verified")
