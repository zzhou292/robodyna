#!/usr/bin/env python3
"""Pinned native contact-channel statements; no production numerical call."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json

ROOT = Path(__file__).resolve().parent
QUAL = ROOT.parent.parent


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def generate():
    sources = {}
    for item in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        data = (ROOT.parent / item["path"]).read_bytes()
        assert len(data) == item["bytes"] and hashlib.sha256(data).hexdigest() == item["sha256"]
        assert hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest() == item["git_blob"]
        sources[Path(item["path"]).name] = data.decode()

    coefficients = module("seed_coefficient_sources", QUAL / "radioss_type25_coefficients/native/prepare.py")
    coefficient_sources = coefficients.sources()
    contributions = module("seed_contribution_sources", QUAL / "radioss_type25_nodal_contributions/native/prepare.py")
    contribution_sources = contributions.generate()  # Also authenticates complete parent donor closure.
    constants = coefficients.constants(coefficient_sources["constant_mod.F"])

    def lines(name, first, last):
        text = sources[name].splitlines(keepends=True)
        assert 1 <= first <= last <= len(text)
        return "".join(text[first - 1:last])

    # Complete selected contact loops; unrelated mass/thermal/rotational
    # producers and unsupported higher-order solid groups are not invoked.
    locations = {
        "SOLID_SORT": (95, 99), "SOLID": (241, 248),
        "QUAD_SORT": (319, 323), "QUAD": (437, 448),
        "TRUSS_SORT": (450, 453), "TRUSS": (462, 470),
        "BEAM_SORT": (472, 475), "BEAM": (500, 510),
        "SPRING_SORT": (512, 515), "SPRING": (531, 545),
        "TRIANGLE_SORT": (547, 550), "TRIANGLE": (633, 642),
    }
    blocks = {name: lines("spmd_msin.F", *where) for name, where in locations.items()}
    assert "IXS(11,I)" in blocks["SOLID_SORT"] and "K=1,8" in blocks["SOLID"]
    assert "IXR(6,I)" in blocks["SPRING_SORT"] and "TWO*STR(I)" in blocks["SPRING"]
    blocks["INITIALIZE"] = lines("lectur.F", 8310, 8315)
    assert "STIFINT = ZERO" in blocks["INITIALIZE"]
    blocks["QUAD_INITIALIZE"] = lines("cinmas.F", 1750, 1759)
    triangle = lines("c3inmas.F", 1558, 1566)
    assert triangle.count("THK(I)") == 1
    # Boundary-only dummy rename: Q and T thickness arrays coexist in wrapper.
    blocks["TRIANGLE_INITIALIZE"] = triangle.replace("THK(I)", "THKT(I)")
    text = (ROOT / "Gather.F.in").read_text()
    text = text.replace("@CONSTANTS@", constants)
    for name, block in blocks.items():
        assert text.count("@" + name + "@") == 1
        text = text.replace("@" + name + "@", block)
    text = text.replace("I7STIFS,IGTYP,", "I7STIFS,IGTYP,NUMELS8,")
    text = text.replace("      I7STIFS=1\n", "      I7STIFS=1\n      NUMELS8=NUMELS\n")

    joint = (ROOT / "Joint.F.in").read_text().replace("@CONSTANTS@", constants)
    storage = lines("hm_read_prop45.F", 1074, 1076)
    store_kn = lines("hm_read_prop45.F", 1088, 1088)
    generic = lines("hm_read_prop_generic.F", 234, 234)
    assert "STIF  = PARGEO(2)" in generic
    geo_store = lines("hm_read_prop_generic.F", 243, 243)
    assert geo_store.count("GEO(3)=STIF") == 1
    geo_store = geo_store.replace("GEO(3)", "GEO(3,1)")
    spring = contribution_sources["Spring.F"]
    start = "      IF (I7STIFS /= 0) THEN\n"
    end = "      ENDIF ! IF (I7STIF /= 0)\n"
    assert spring.count(start) == spring.count(end) == 1
    spring = start + spring.split(start, 1)[1].split(end, 1)[0] + end
    for name, block in {"PROPERTY45": storage, "STORE_KN": store_kn,
                        "GENERIC_STIFFNESS": generic, "GEO_STIFFNESS": geo_store, "SPRING": spring}.items():
        assert joint.count("@" + name + "@") == 1
        joint = joint.replace("@" + name + "@", block)
    assert "@" not in text and "@" not in joint
    initialize = (ROOT / "Initialize.F.in").read_text().replace("@CONSTANTS@", constants)
    for tag, line in [("INITIALIZE_VNS", 8279), ("INITIALIZE_BNS", 8287)]:
        block = lines("lectur.F", line, line)
        assert ("VNS = ZERO" if tag == "INITIALIZE_VNS" else "BNS = ZERO") in block
        initialize = initialize.replace("@" + tag + "@", block)
    return {"Gather.F": text, "Joint.F": joint, "Initialize.F": initialize, "my_orders.c": sources["my_orders.c"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, text in generate().items():
        path = args.output / name
        if args.check:
            assert path.read_text() == text
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
    print("Pinned ordered SPMD channels, shell incidence and TYPE45 zero witness prepared")
