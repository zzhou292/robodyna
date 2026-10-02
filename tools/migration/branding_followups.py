"""Authenticate reviewed later edits without repinning a historical brand snapshot."""

import hashlib
import json
from pathlib import Path

from tools.migration.source_transform import original_bytes, relative_file


def object_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()


def admit_followups(root, manifest_path, manifest, entries, registry_path):
    if registry_path is None:
        return {}
    registry = json.loads(Path(registry_path).read_text())
    if registry.get("schema") != "robodyna.branding_followups.v1":
        raise ValueError("Unsupported branding follow-up registry")
    if registry["historical_manifest_sha256"] != hashlib.sha256(Path(manifest_path).read_bytes()).hexdigest():
        raise ValueError("Historical branding manifest was changed or repinned")
    branded = {row["path"]: row for row in manifest["changed_text"]}
    admitted = {}
    for row in registry["files"]:
        name = row["path"]
        if name in admitted or name not in branded:
            raise ValueError("Duplicate or unlisted branding follow-up path")
        previous = branded[name]
        if row["historical_sha256"] != previous["after_sha256"] or row["current_sha256"] == row["historical_sha256"]:
            raise ValueError("Invalid historical branding boundary or stale follow-up")
        path = relative_file(root, name)
        if path.is_symlink() or not path.is_file():
            raise ValueError("Expected regular branding follow-up input")
        if hashlib.sha256(path.read_bytes()).hexdigest() != row["current_sha256"]:
            raise ValueError("Current branding follow-up identity differs: " + name)
        key = previous.get("transformation_original_path")
        if key is not None:
            if row["kind"] != "imported_source" or row["original_path"] != key:
                raise ValueError("Branding follow-up does not own this source history")
            entry = entries[key]
            count = row["historical_replacement_count"]
            if type(count) is not int or not 0 < count < len(entry["replacements"]):
                raise ValueError("Invalid historical source-history boundary")
            historical_entry = dict(entry, replacements=entry["replacements"][:count])
            if object_sha256(historical_entry) != row["historical_entry_sha256"]:
                raise ValueError("Historical source history was rewritten")
            if entry["canonical_path"] != name:
                raise ValueError("Branding follow-up requires an explicit relocation review")
            # The complete immutable import proof remains mandatory.
            original_bytes(root, entry)
            replacements = entry["replacements"][count:]
            encoding = entry.get("encoding", "utf-8")
        else:
            if row["kind"] != "owned_text":
                raise ValueError("Owned presentation follow-up has the wrong kind")
            replacements = row["replacements"]
            if not replacements:
                raise ValueError("Owned presentation follow-up lacks its exact inverse")
            encoding = row.get("encoding", "utf-8")
        # Reverse just the later suffix and require exactly the recorded branded
        # bytes. The existing verifier then also checks the pre-branding boundary.
        original_bytes(root, dict(original_path=name, canonical_path=name,
                                  original_sha256=previous["after_sha256"],
                                  replacements=replacements, encoding=encoding))
        admitted[name] = row["current_sha256"]
    return admitted
