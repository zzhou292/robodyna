#!/usr/bin/env python3
"""Prepare only R1 sources with private constant-module names for CMake.

Borrow exact qualified QEPH constants/includes; never edit the owning originals.
Only the constant_mod identifier token changes, no arithmetic or precision.
This is source preparation, not a compiler or native runtime command.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
from verify_sources import verify

TOKEN = re.compile(r"\bconstant_mod\b", re.IGNORECASE)
PRIVATE_MODULE = "TL_T3_R1_CONSTANT_MOD"


def prepare(output, check=False):
    verify()
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / "source-manifest.json").read_text())
    originals = {entry["source"]: entry["path"] for entry in manifest["sources"]}
    paths = {"NativeT3Startup.F": "NativeT3Startup.F",
             "extracted/StarterC3evec3.F": "extracted/StarterC3evec3.F"}
    for source in ("common_source/modules/constant_mod.F", "engine/share/spe_inc/implicit_f.inc"):
        paths["original/" + source] = originals[source]
    records = []
    for relative, owned_source in paths.items():
        original = (root / owned_source).read_bytes()
        prepared = TOKEN.sub(PRIVATE_MODULE, original.decode("latin1")).encode("latin1")
        path = output / relative
        if check:
            if not path.is_file() or path.read_bytes() != prepared:
                raise RuntimeError("Stale T3 prepared source: " + relative)
        elif not path.is_file() or path.read_bytes() != prepared:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(prepared)
        records.append({"source_path": owned_source, "prepared_path": relative,
                        "source_sha256": hashlib.sha256(original).hexdigest(),
                        "prepared_sha256": hashlib.sha256(prepared).hexdigest(),
                        "prepared_bytes": len(prepared)})
    receipt = {"schema": "tl.t3-r1-prepared-sources.v1", "files": records,
               "module_mapping": {"constant_mod": PRIVATE_MODULE},
               "transformation": "case-insensitive whole-token module identifier rename only"}
    encoded = (json.dumps(receipt, indent=2) + "\n").encode()
    path = output / "prepared-sources.json"
    if check:
        if path.read_bytes() != encoded:
            raise RuntimeError("Stale T3 preparation receipt")
    elif not path.is_file() or path.read_bytes() != encoded:
        path.write_bytes(encoded)
    return {"status": "passed", "prepared_files": len(records), "check_only": check}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    print(json.dumps(prepare(args.output.resolve(), args.check), sort_keys=True))
