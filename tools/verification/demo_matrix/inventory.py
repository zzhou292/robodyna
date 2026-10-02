"""Join the immutable source denominator with current expanded Bazel declarations."""

import collections
import datetime
import json
from pathlib import Path

from tools.verification.chrono_inventory import digest, git
from .query import executable_sources, read_public_labels, read_query
from .roster import read_roster

SCHEMA = "robodyna.demo_inventory.v1"


def refresh(repository, query_xml, public_labels, query_receipt=None):
    root = Path(repository).resolve(strict=True)
    roster = read_roster(root)
    capture = json.loads(Path(query_receipt).read_text()) if query_receipt else None
    if capture:
        if capture.get("status") != "passed" or len(capture.get("queries", [])) != 2:
            raise ValueError("Query capture did not complete both snapshots")
        expected = {r["output"]: r["sha256"] for r in capture["queries"]}
        for path in (query_xml, public_labels):
            if expected.get(Path(path).name) != digest(Path(path).read_bytes()):
                raise ValueError("Query snapshot differs from its capture receipt")
    rules = read_query(query_xml)
    public = read_public_labels(public_labels)
    if not public.issubset(rules):
        raise ValueError("Public-label and expanded-rule snapshots do not describe the same graph")
    sources = collections.defaultdict(list)
    for label, record in executable_sources(rules).items():
        if label not in public or not label.startswith(("//examples/", "//apps/precice:")):
            continue
        for source in record["sources"]:
            location = record["location"]
            if location.startswith(str(root) + "/"):
                location = location[len(str(root)) + 1:]
            elif location.startswith("/"):
                location = "see captured query XML"
            sources[source].append({"label": label, "kind": record["kind"],
                                    "alias_for": record.get("alias_for"), "location": location})
    for row in roster["programs"]:
        family = row["family"]
        candidates = sources[row["source"]]
        if family in ("native_demo", "native_template", "cuda_demo", "application"):
            candidates = [r for r in candidates if r["kind"] in ("cc_binary", "cuda_binary") or
                          (r["kind"] == "py_binary" and family != "cuda_demo")]
        elif family == "python_demo":
            candidates = [r for r in candidates if r["kind"] == "py_binary"]
        else:
            candidates = [r for r in candidates if r["kind"] in ("managed_assembly", "py_binary")]
        row["public_targets"] = sorted(candidates, key=lambda r: r["label"])
        row["declaration_status"] = "declared" if candidates else "not_declared"
        declaration = row.pop("declared_manifest") or {}
        preferred = declaration.get("target")
        actual = {r["label"] for r in candidates}
        row["preferred_target"] = preferred if preferred in actual else None
        row["manifest_target"] = preferred
        row["required_profiles"] = declaration.get("required_profiles", declaration.get("required_configs", []))
        row["numpy_bridge"] = declaration.get("numpy_bridge", False)
        # Previous discovery manifests contain stale or wrapper-level statuses.
        # Preserve them as context; never reinterpret them as current demo gates.
        row["build_status"] = "no_target_scoped_evidence"
        row["runtime_status"] = "not_qualified_by_this_inventory"
    return {"schema": SCHEMA, "recorded_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "audited_head": git(root, "rev-parse", "HEAD").decode().strip(),
            "scope": "311 native demo mains +1 FMI template;26 CUDA,97 Python,17 C# separate; applications outside denominator",
            "query": {"xml_sha256": digest(Path(query_xml).read_bytes()),
                      "public_labels_sha256": digest(Path(public_labels).read_bytes()), "rule_count": len(rules)},
            "captured_build_metadata": capture.get("build_metadata") if capture else None,
            **roster, "declaration_counts": dict(collections.Counter(r["family"] for r in roster["programs"] if r["public_targets"]))}


def read_inventory(path):
    value = json.loads(Path(path).read_text())
    if value.get("schema") != SCHEMA:
        raise ValueError("Unsupported current demo inventory")
    seen = set()
    for row in value["programs"]:
        if row["source"] in seen:
            raise ValueError("Duplicate program source")
        seen.add(row["source"])
        if (row["declaration_status"] == "declared") != bool(row["public_targets"]):
            raise ValueError("Declaration status disagrees with the queried graph")
    return value
