#!/usr/bin/env python3
"""Prepare selected T3 sources with private module names for CMake.

Borrow exact qualified QEPH constants/includes; never edit the owning originals.
Default startup mode preserves the R1 receipt. Explicit engine mode renames
the four complete native modules into a separate context; no arithmetic changes.
This is source preparation, not a compiler or native runtime command.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
from verify_sources import verify

PRIVATE_MODULE = "TL_T3_R1_CONSTANT_MOD"


def prepare(output, check=False, stage="startup"):
    verify()
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / "source-manifest.json").read_text())
    originals = {entry["source"]: entry["path"] for entry in manifest["sources"]}
    paths = {"NativeT3Startup.F": "NativeT3Startup.F",
             "extracted/StarterC3evec3.F": "extracted/StarterC3evec3.F"}
    for source in ("common_source/modules/constant_mod.F", "engine/share/spe_inc/implicit_f.inc"):
        paths["original/" + source] = originals[source]
    modules={"constant_mod": PRIVATE_MODULE}
    schema="tl.t3-r1-prepared-sources.v1"
    if stage in ("engine","force"):
        routines={"C3COOR3","C3EVEC3","C3DERI3","C3DEFO3","C3CURV3","CLSKEW3"}
        paths={name:name for name in ("T3NativeGeometry.F","NativeT3Kinematics.F")}
        for entry in manifest["extractions"]:
            if entry["routine"].upper() in routines and not entry["source"].startswith("starter/"):
                paths["extracted/"+Path(entry["path"]).name]=entry["path"]
        for source,owned in originals.items():
            if source.startswith("common_source/modules/"):
                paths["original/"+source]=owned
        paths["original/engine/share/spe_inc/implicit_f.inc"]=originals["engine/share/spe_inc/implicit_f.inc"]
        modules={name:"T3_ENGINE_"+name.upper() for name in
                 ("constant_mod","precision_mod","element_mod","elbufdef_mod")}
        schema="tl.t3-engine-prepared-sources.v1"
        if stage=="force":
            routines={"C3COEF3","C3STRA3","C3DT3","C3SROTO3","C3FINT3",
                      "C3FCUM3","C3MCUM3","C3UPDT3","SIGEPS01G","CSSP2A11"}
            paths={name:name for name in ("T3NativeHistory.F","T3NativeMaterial.F",
                "T3NativeLaw1.F","T3NativeStiffness.F","NativeT3Force.F","NativeT3Scatter.F")}
            for entry in manifest["extractions"]:
                if entry["routine"].upper() in routines:
                    paths["extracted/"+Path(entry["path"]).name]=entry["path"]
            source="engine/share/spe_inc/implicit_f.inc"
            paths["original/"+source]=originals[source]
            schema="tl.t3-force-prepared-sources.v1"
    token=re.compile(r"\b(?:"+"|".join(modules)+r")\b",re.IGNORECASE)
    records = []
    for relative, owned_source in paths.items():
        original = (root / owned_source).read_bytes()
        prepared = token.sub(lambda m:modules[m.group().lower()],original.decode("latin1")).encode("latin1")
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
    receipt = {"schema": schema, "files": records,
               "module_mapping": modules,
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
    parser.add_argument("--stage", choices=("startup","engine","force"), default="startup")
    args = parser.parse_args()
    print(json.dumps(prepare(args.output.resolve(), args.check, args.stage), sort_keys=True))
