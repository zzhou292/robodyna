"""Authenticate narrow, reversible metadata overlays on imported BUILD files."""

import hashlib
from pathlib import Path


def verify_build_overlay(relative, current, entry):
    """Return the qualified hash only if the declared edits fully explain drift.

    Existing physics files cannot use this exception. Reversing the declared
    exact replacements must reconstruct the original qualified BUILD bytes.
    """
    if Path(relative).name not in ("BUILD", "BUILD.bazel"):
        raise ValueError(f"Only BUILD metadata can have an import overlay: {relative}")
    if hashlib.sha256(current).hexdigest() != entry["overlay_sha256"]:
        raise ValueError(f"Build overlay changed outside its reviewed record: {relative}")
    replacements = entry["replacements"]
    if not replacements:
        raise ValueError(f"Build overlay has no explicit changes: {relative}")
    baseline = current
    for replacement in reversed(replacements):
        before, after = replacement["before"].encode(), replacement["after"].encode()
        if not before or not after or before == after or baseline.count(after) != 1:
            raise ValueError(f"Build overlay is ambiguous or incomplete: {relative}")
        baseline = baseline.replace(after, before, 1)
    digest = hashlib.sha256(baseline).hexdigest()
    if digest != entry["baseline_sha256"]:
        raise ValueError(f"Build overlay does not reconstruct qualified bytes: {relative}")
    return digest
