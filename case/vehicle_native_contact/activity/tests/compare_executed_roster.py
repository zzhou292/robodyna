"""Compare app source coverage against an authenticated existing native case.

This is a qualification reader, never a production source or native solver.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys

APP_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(APP_ROOT))
from modelio._legacy import fields, file_sha256, require, scan_vehicle


class Roster:
    def __init__(self):
        self.ids = {name: [] for name in ("shells", "beams", "solids")}

    def metadata(self, keyword, records, line):
        pass

    def record(self, keyword, line, number):
        names = {"*ELEMENT_SHELL": "shells", "*ELEMENT_BEAM": "beams", "*ELEMENT_SOLID": "solids"}
        if keyword not in names:
            return
        identity = fields(line[:8], [8], [int])[0][0]
        target = self.ids[names[keyword]]
        require(0 < identity < 2**64 and len(target) < 1048576, "Invalid native case identity or count")
        target.append(identity)


def compare(report, manifest_path, expected_hash):
    require(manifest_path.stat().st_size <= 4 << 20, "Native manifest exceeds cap")
    require(file_sha256(manifest_path) == expected_hash, "Native case manifest changed")
    manifest = json.loads(manifest_path.read_text())
    require(manifest["case_profile"] == report["executed_profile"] == "native_v6_raw8_heph_explicit_cin28", "Executed profiles differ")
    key = [entry for entry in manifest["artifacts"] if Path(entry["path"]).name == "case.key"]
    require(len(key) == 1, "Native source key is ambiguous")
    entry = key[0]
    path = manifest_path.parent / "case.key"
    require(path.stat().st_size == entry["bytes"] <= 64 << 20 and file_sha256(path) == entry["sha256"], "Native source key changed")
    roster = Roster()
    with path.open("rb") as stream:
        observed = scan_vehicle(stream, path.name, roster)
    require(observed["sha256"] == entry["sha256"], "Native source changed during scan")
    result = {}
    for name, ids in roster.ids.items():
        ordered = sorted(ids)
        require(len(ordered) == len(set(ordered)), "Duplicate native geometry identity")
        digest = hashlib.sha256(b"".join(struct.pack("<Q", value) for value in ordered)).hexdigest()
        require(len(ids) == report[name]["executed"] and digest == report[name]["executed_ids_sha256"], f"Executed {name} support roster differs")
        result[name] = {"count": len(ids), "ids_sha256": digest}
    for key in ("physical_nodes", "wall_shells", "type13", "beam18", "welds", "joints"):
        require(report[key] == manifest[key], f"Executed {key} census differs")
    return {"status": "matched_executed_source_rosters", "scope": "Source identities only; not runtime/native physics or performance qualification", "families": result, "native_manifest_sha256": expected_hash}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--native-manifest", type=Path, required=True)
    parser.add_argument("--native-manifest-sha256", required=True)
    args = parser.parse_args()
    require(args.report.stat().st_size <= 1 << 20, "Coverage report exceeds cap")
    report = json.loads(args.report.read_text())
    print(json.dumps(compare(report, args.native_manifest, args.native_manifest_sha256), sort_keys=True))


if __name__ == "__main__":
    main()
