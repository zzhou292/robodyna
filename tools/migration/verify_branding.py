"""Verify live presentation edits and retired assets without rewriting evidence."""

import argparse
import hashlib
import json
from pathlib import Path

from tools.migration.source_transform import index_entries, original_bytes, relative_file
from tools.migration.branding_followups import admit_followups


def _identity(root, name, expected):
    path = relative_file(root, name)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"Expected regular branding input: {name}")
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError(f"Branding input identity differs: {name}")


def verify(root, manifest_path, ledger_path, followups_path=None):
    root = Path(root).resolve(strict=True)
    manifest = json.loads(Path(manifest_path).read_text())
    if manifest.get("schema") != "robodyna.branding_replacements.v1":
        raise ValueError("Unsupported branding manifest")
    entries = index_entries(json.loads(Path(ledger_path).read_text()))
    followups = admit_followups(root, manifest_path, manifest, entries, followups_path)
    imported = 0
    for row in manifest["changed_text"]:
        _identity(root, row["path"], followups.get(row["path"], row["after_sha256"]))
        key = row.get("transformation_original_path")
        if key is None:
            if row["path"].startswith(("src/compatibility/chrono/", "src/compatibility/app/")) and not row["path"].endswith("/BUILD.bazel"):
                raise ValueError("Imported changed text lacks its exact inverse history")
            continue
        entry = entries[key]
        if entry["canonical_path"] != row["path"]:
            raise ValueError("Branding history belongs to a different source")
        original_bytes(root, entry)
        prefix = row["prior_replacement_count"]
        if type(prefix) is not int or not 0 <= prefix < len(entry["replacements"]):
            raise ValueError("Invalid pre-branding history boundary")
        # Reuse the same byte-exact verifier for the immediate qualified preimage.
        tail = dict(entry, original_path=row["path"], canonical_path=row["path"],
                    original_sha256=row["before_sha256"], replacements=entry["replacements"][prefix:])
        tail.pop("forwarding_sha256", None)
        original_bytes(root, tail)
        imported += 1
    for row in manifest["retired_assets"]:
        path = relative_file(root, row["path"])
        if path.exists() or path.is_symlink():
            raise ValueError(f"Retired branding asset remains in the checkout: {row['path']}")
    master = manifest["approved_generated_master"]
    _identity(root, master["path"], master["sha256"])
    for row in manifest["byte_identical_replacements"]:
        if row["sha256"] != master["sha256"]:
            raise ValueError("Branding package copy differs from the approved master")
        _identity(root, row["path"], row["sha256"])
    for row in manifest.get("new_presentation_files", []):
        _identity(root, row["path"], row["sha256"])
    historical = manifest["preserved_historical_screenshot"]
    _identity(root, historical["path"], historical["sha256"])
    if imported != manifest["source_proof"]["imported_changed_text_files"]:
        raise ValueError("Imported branding source count differs")
    return {"schema": "robodyna.branding_verification.v1", "passed": True,
            "changed_text_files": len(manifest["changed_text"]), "imported_source_histories": imported,
            "retired_assets": len(manifest["retired_assets"]),
            "replacement_copies": len(manifest["byte_identical_replacements"]),
            "reviewed_followup_files": len(followups),
            "scope": "Live file/hash and exact-source-inverse verification; no optional-backend runtime claim"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--followups", type=Path, help="Explicit reviewed later-edit registry; historical manifests remain unchanged")
    args = parser.parse_args()
    followups = args.followups or args.repository / "docs/verification/BRANDING_FOLLOWUPS.json"
    if args.followups is None and not followups.exists():
        followups = None
    print(json.dumps(verify(args.repository, args.repository / "docs/migration/BRANDING_REPLACEMENTS.json",
                            args.repository / "docs/migration/SOURCE_TRANSFORMATIONS.json", followups), indent=2))


if __name__ == "__main__":
    main()
