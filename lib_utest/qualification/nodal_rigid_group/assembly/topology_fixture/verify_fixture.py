#!/usr/bin/env python3
"""Verify the frozen ID fixture; optionally reproduce it from hashed audits."""
import argparse
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
AUDIT_SHA = "6643c52d226ba3f2ee13fe51bd3c8fc6aef6b0db69c51e51fff769be6b72930b"
SCOPE_SHA = "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0"


def authenticated(path, expected):
    data = Path(path).read_bytes()
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError(f"Source audit hash changed: {path}")
    return json.loads(data)


def render(audit, scope):
    lines = ["// Frozen literal source IDs only; see source-manifest.json.",
             "#pragma once", '#include "lib_src/constraints/NodalRigidPartTopology.h"',
             "namespace yaris_rigid_topology_fixture {", "namespace r=tl::fea::rigid;"]

    def array(kind, name, values):
        lines.append(f"inline const {kind} {name}[]={{")
        for start in range(0, len(values), 10):
            lines.append("  " + ",".join(str(x) for x in values[start:start + 10]) + ",")
        lines.append("};")

    parts, extras, members = [], [], []
    for part in audit["parts"]:
        offset = len(members)
        members += part["source_nodes"]
        parts.append("{%d,Members+%d,%d}" % (part["part_id"], offset, len(part["source_nodes"])))
    for row in audit["source_extra_and_joint_records"]:
        if row["keyword"] != "*CONSTRAINED_EXTRA_NODES_SET":
            continue
        pid, sid = int(row["cards"][0][:10]), int(row["cards"][0][10:20])
        offset = len(members)
        members += row["node_ids"]
        extras.append("{%d,%d,Members+%d,%d}" % (pid, sid, offset, len(row["node_ids"])))
    resolved = audit["resolved_membership_without_generated_primaries"]
    expected = sorted(n for nodes in resolved.values() for n in nodes)
    other = [n for group in scope["connections"]["nodal_rigid_groups"] for n in group["source_node_ids"]]
    assert len(parts) == 22 and len(extras) == 20 and len(members) == 5452
    assert len(resolved) == 20 and len(expected) == len(set(expected)) == 5452
    assert len(other) == len(set(other)) == 7539 and set(other).isdisjoint(expected)
    assert set(expected) == set(members)
    array("r::SourceNodeId", "Members", members)
    array("r::PartTopologyPartInput", "Parts", parts)
    array("r::PartTopologyExtraInput", "Extras", extras)
    merges = []
    for row in audit["source_extra_and_joint_records"]:
        if row["keyword"] == "*CONSTRAINED_RIGID_BODIES":
            merges += ["{%d,%d}" % (int(card[:10]), int(card[10:20])) for card in row["cards"]]
    assert merges == ["{2000387,2000414}", "{2000397,2000399}"]
    array("r::PartTopologyMerge", "Merges", merges)
    array("r::SourceNodeId", "Expected", expected)
    array("r::SourceNodeId", "Other", other)
    array("std::uint64_t", "RootParts", [int(pid) for pid in resolved])
    array("std::size_t", "RootCounts", [len(nodes) for nodes in resolved.values()])
    lines.append("} // namespace yaris_rigid_topology_fixture")
    return ("\n".join(lines) + "\n").encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit")
    parser.add_argument("--scope")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    if bool(args.audit) != bool(args.scope) or (args.write and not args.audit):
        parser.error("Regeneration requires both --audit and --scope")
    path = HERE / "YarisRigidTopologyFixture.h"
    if args.audit:
        audit = authenticated(args.audit, AUDIT_SHA)
        scope = authenticated(args.scope, SCOPE_SHA)
        generated = render(audit, scope)
        if args.write:
            path.write_bytes(generated)
            manifest = {"audit_sha256": AUDIT_SHA, "scope_sha256": SCOPE_SHA,
                        "original_member_sha256": audit["original_member_sha256"],
                        "archive_sha256": audit["archive_sha256"], "fixture": path.name,
                        "fixture_sha256": hashlib.sha256(generated).hexdigest(),
                        "fixture_bytes": len(generated), "parts": 22, "roots": 20,
                        "original_members": 5452, "other_group_members": 7539,
                        "scope": "literal source topology only; no M/J or owner admission"}
            (HERE / "source-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        elif path.read_bytes() != generated:
            raise ValueError("Frozen fixture differs from authenticated source audits")
    manifest = json.loads((HERE / "source-manifest.json").read_text())
    data = path.read_bytes()
    if len(data) != manifest["fixture_bytes"] or hashlib.sha256(data).hexdigest() != manifest["fixture_sha256"]:
        raise ValueError("Frozen Yaris topology fixture changed")
    print("Verified Yaris literal topology: 22 PARTs, 20 roots, 5452 members, 7539 other rigid members")


if __name__ == "__main__":
    main()
