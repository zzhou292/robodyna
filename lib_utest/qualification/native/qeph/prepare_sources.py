#!/usr/bin/env python3
"""Make private module names visible to CMake's Fortran dependency scanner.

Only the four complete module identifier tokens change. Routines/COMMON retain
their existing compile-time isolation; no arithmetic or precision text changes.
Originals and exact extracts are read-only and verified before preparation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

from verify_sources import verify

MODULES = ("constant_mod", "precision_mod", "element_mod", "elbufdef_mod")
TOKEN = re.compile(r"\b(?:" + "|".join(MODULES) + r")\b", re.IGNORECASE)


def prepare(output, check=False):
    verify()
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / "source-manifest.json").read_text())
    paths = ["NativeQephStartup.F", "NativeQephKinematics.F"]
    paths += [entry["path"] for entry in manifest["extractions"]]
    paths += ["original/" + entry["path"] for entry in manifest["sources"]
              if entry["path"].startswith("common_source/modules/")]
    paths.append("original/engine/share/spe_inc/implicit_f.inc")
    records = []
    for relative in paths:
        source = (root / relative).read_bytes()
        # latin1 roundtrips every source byte, including untouched notices.
        transformed = TOKEN.sub(lambda match: "QE_Q1_" + match.group().upper(),
                                source.decode("latin1")).encode("latin1")
        destination = output / relative
        if check:
            if not destination.is_file() or destination.read_bytes() != transformed:
                raise RuntimeError("Stale Q1 prepared source: " + relative)
        elif not destination.is_file() or destination.read_bytes() != transformed:
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(transformed)
        records.append({"source_path": relative,
                        "source_sha256": hashlib.sha256(source).hexdigest(),
                        "prepared_path": relative,
                        "prepared_sha256": hashlib.sha256(transformed).hexdigest(),
                        "prepared_bytes": len(transformed)})
    receipt = {"schema": "tl.qeph-q1-prepared-sources.v1",
               "transformation": "case-insensitive whole-token module identifier rename only",
               "module_mapping": {name: "QE_Q1_" + name.upper() for name in MODULES},
               "files": records}
    encoded = (json.dumps(receipt, indent=2) + "\n").encode()
    path = output / "prepared-sources.json"
    if check:
        if path.read_bytes() != encoded:
            raise RuntimeError("Stale Q1 prepared-source receipt")
    elif not path.is_file() or path.read_bytes() != encoded:
        path.write_bytes(encoded)
    return {"status": "passed", "prepared_files": len(records), "check_only": check}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    print(json.dumps(prepare(args.output.resolve(), args.check), sort_keys=True))
