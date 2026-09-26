#!/usr/bin/env python3
"""Read-only source contract check; native donors are qualification-only."""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--workspace", type=Path, required=True)
args = parser.parse_args()
here = Path(__file__).resolve().parent
root = here.parents[3]
raw = (here / "source-manifest.json").read_bytes()
assert hashlib.sha256(raw).hexdigest() == "3e9b09f8426d827653eaed2d180812656bc0dd93edb4ce30fa767f526613e8de"
manifest = json.loads(raw)
for row in manifest["app_files"]:
    path = Path(row["path"])
    assert not path.is_absolute() and ".." not in path.parts
    data = (root / path).read_bytes()
    assert len(data) == row["bytes"], path
    assert hashlib.sha256(data).hexdigest() == row["sha256"], path
proof_raw = (here / "source-defaults.json").read_bytes()
assert hashlib.sha256(proof_raw).hexdigest() == manifest["default_proof_sha256"]
proof = json.loads(proof_raw)
assert proof["revision"] == "a62b27e6baa555d222a580d6218867d0be4d70b5"
for row in proof["files"] + proof["converter_admesh_absence"]:
    path = Path(row["path"])
    assert not path.is_absolute() and ".." not in path.parts
    data = (args.workspace / path).read_bytes()
    assert len(data) == row["bytes"], path
    assert hashlib.sha256(data).hexdigest() == row["sha256"], path
    if "git_blob" in row:
        blob = b"blob " + str(len(data)).encode() + b"\0" + data
        assert hashlib.sha1(blob).hexdigest() == row["git_blob"], path
assert len(proof["files"]) == 14
assert len(proof["converter_admesh_absence"]) == 40
for row in proof["converter_admesh_absence"]:
    data = (args.workspace / row["path"]).read_text()
    assert "/ADMESH" not in data and '"/TRIA"' not in data
context = (here / "Context.cpp").read_text()
for token in ("part_sets::Read", "root.members.size() != 1", "list.members != selected.selected_part_ids",
              "OrdinaryHeader", "CompleteNoApplicableType24", "source_digest"):
    assert token in context
certificate = (here / "Certificate.cpp").read_text()
for token in ("a.arity < b.arity", "at->selected != first->selected", "NeedsNativeReaderOrder",
              "SameIdentity", "faces[first].raw_role != faces[i].raw_role"):
    assert token in certificate
owner = (here.parent / "InitialSurfaceSource.cpp").read_text()
for token in ("packed.Input(true)", "packed.Input(false)", "CertifyMembership", "CertifyOrder",
              "coated::detail::PrepareInputs"):
    assert token in owner
assert "coefficients()" not in owner
public = (here.parent / "InitialSurfaceSource.h").read_text()
assert "reader_row" not in public and "buffer_ordinal" not in public
assert "InitialClauseBeforeI25Classification" in public
print(json.dumps({"status":"passed", "scope":"initial source surface proof only",
                  "native_donors":14, "converter_units":40,
                  "app_records":len(manifest["app_files"]), "numerical_execution":False}))
