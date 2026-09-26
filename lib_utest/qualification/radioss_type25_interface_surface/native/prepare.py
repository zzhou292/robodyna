#!/usr/bin/env python3
"""Authenticate full IN24 and literal I25SURFI selected blocks; no port math."""
import argparse
import hashlib
import importlib.util
import json
import re
from pathlib import Path
ROOT = Path(__file__).resolve().parent

def generate(tl_root):
    spec = importlib.util.spec_from_file_location("interface_native_extract",
        tl_root / "lib_utest/qualification/radioss_type25_selection/native/Sources.py")
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    sources = {}
    for row in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        path = (ROOT / row["path"]).resolve()
        assert path.is_relative_to(tl_root.resolve())
        raw = path.read_bytes()
        assert len(raw) == row["bytes"] and hashlib.sha256(raw).hexdigest() == row["sha256"]
        assert hashlib.sha1(b"blob " + str(len(raw)).encode() + bytes([0]) + raw).hexdigest() == row["git_blob"]
        sources[Path(row["path"]).name] = raw.decode()
    classify = helper.routine(sources["i24surfi.F"], "IN24COQ_SOL3")
    classify += helper.routine(sources["i24surfi.F"], "SEG_INS")
    classify, changes = re.subn(r'(#include\s+)"implicit_f.inc"', r'\1"interface_implicit.inc"', classify)
    assert changes == 2
    marker = "  500  CONTINUE\n"
    assert classify.count(marker) == 1
    classify = classify.replace(marker, marker + "      CALL RD_INTERFACE_MATCH(NELS)\n")
    surfi = sources["i25surfi.F"]
    start = surfi.index("      IF(ISU1 /= 0.AND..NOT.LINE_SEG_M)THEN\n")
    raw_loop = surfi[start:surfi.index("      NSU1 = L\n", start)]
    # Read-only observations surrounding the unchanged original complete call.
    marker = "          CALL IN24COQ_SOL3("
    assert raw_loop.count(marker) == 1
    raw_loop = raw_loop.replace(marker, "          CALL RD_INTERFACE_RESET()\n" + marker)
    marker = "     .                 KNOD2ELS,NOD2ELS,IXS ,IXS10 ,IXS16 ,IXS20 )\n"
    assert raw_loop.count(marker) == 1
    raw_loop = raw_loop.replace(marker, marker + "          MATCHED(L)=LAST_MATCHED_SOLID\n")
    start = surfi.index("      IRECTMP_SAV(1:6,1:NRTM) =  IRECTMP(1:6,1:NRTM)\n")
    stop = "      CALL MY_ORDERS( MODE, WORK, IRECTMP, INDEX, NRTM , 6)\n"
    end = surfi.index(stop, start) + len(stop)
    keys = surfi[start:end]
    start = surfi.index("      IF(IALLO==1)THEN", end)
    counts = surfi[start:].split("        NEDGEP = 0\n", 1)[0]
    counts = counts.split("\n", 1)[1]  # Wrapper executes this full selected count branch.
    start = surfi.index("      ELSE ! IF(IALLO==1)THEN")
    output = surfi[start:].split("          NEDGEP = 0\n", 1)[0].split("\n", 1)[1]
    for index in ("I1", "I2"):
        marker = "        IRECT(1,NRTM)=IRECTMP_SAV(1," + index + ")\n"
        # I2 has four extra spaces; preserve all source numerical statements.
        if index == "I2":
            marker = "    " + marker
        assert output.count(marker) == 1
        output = output.replace(marker, marker +
            "        WINNER(NRTM)=" + index + "\n" +
            "        RAW_PRIMARY(" + index + ")=NRTM\n")
    at = output.rfind("        END DO")
    assert at != -1
    output = output[:at] + "          RAW_PRIMARY(I2)=NRTM\n" + output[at:]
    cnel = sources["build_cnel.F"]
    start = cnel.index("      DO  K=2,9\n")
    counts_cnel = cnel[start:cnel.index("      DO I=1,NUMELIG3D\n", start)]
    at = cnel.index("C building the matrix Nod -> Solid element")
    start = cnel.index("      DO  K=2,9\n", at)
    fill_cnel = cnel[start:cnel.index("      DO K=2,3\n", start)]
    wrapper = (ROOT / "Wrapper.F.in").read_text()
    for name, value in {
        "CNEL_COUNTS": counts_cnel, "CNEL_FILL": fill_cnel,
        "RAW_CLASSIFICATION": raw_loop, "FILTER_KEYS": keys,
        "FILTER_COUNTS": counts, "FILTER_OUTPUT": output,
    }.items():
        marker = "@" + name + "@"
        assert wrapper.count(marker) == 1
        wrapper = wrapper.replace(marker, value)
    return {
        "Classification.F": classify,
        "InterfaceWrapper.F": wrapper,
        "interface_implicit.inc": "      USE ISO_C_BINDING\n      USE STARTUP_NATIVE_CONSTANTS\n"
            "      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n",
        "Observation.F90": (ROOT / "Observation.F90").read_text(),
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--tl-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, content in generate(args.tl_root).items():
        path = args.output / name
        if args.check:
            assert path.read_text() == content, name
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
    print("Complete IN24 and literal I25SURFI classification/tag/filter blocks authenticated")
