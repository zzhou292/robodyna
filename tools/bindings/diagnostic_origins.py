"""Map SWIG view locations only after authenticating their exact original bytes."""

import hashlib
import json
from pathlib import Path
import re


# An independent format expectation: a changed producer prefix requires review.
GUARD_PREFIX = (b"// Generated parser metadata, authenticated against canonical source.\n"
                b"#ifndef SWIG\n#error This declaration view is for SWIG only; include the real Robodyna API.\n#endif\n")


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _relative(value):
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError("Declaration origin must stay inside its recorded snapshot")
    return path


def authenticated_views(candidate, baseline):
    """Return mappings backed by the saved view receipt and historical bytes."""
    candidate, baseline = Path(candidate).resolve(), Path(baseline).resolve()
    old_manifest = json.loads((baseline / "parser-inputs.json").read_text())
    new_manifest = json.loads((candidate / "parser-inputs.json").read_text())
    result = {}
    for output, recorded in new_manifest.get("generated_declaration_views", {}).items():
        view_path = candidate / "swig_generated" / _relative(output)
        receipt = json.loads(view_path.with_suffix(view_path.suffix + ".json").read_text())
        if receipt != recorded or receipt.get("schema") != "robodyna.swig_declaration_view.v1":
            raise ValueError("Declaration receipt differs from its recorded snapshot")
        original_name = str(_relative(receipt["original_path"]))
        original = (baseline / original_name).read_bytes()
        view = view_path.read_bytes()
        if (old_manifest["source_inputs"].get(original_name) != receipt["original_sha256"] or
                _digest(original) != receipt["original_sha256"] or _digest(view) != receipt["view_sha256"]):
            raise ValueError("Declaration view or historical source hash changed")
        if view != GUARD_PREFIX + original:
            raise ValueError("Declaration view is not the exact guard prefix and historical source")
        result[str(view_path)] = {"original": original_name, "prefix_lines": GUARD_PREFIX.count(b"\n"),
                                  "source_lines": len(original.splitlines()), "view_sha256": receipt["view_sha256"],
                                  "original_sha256": receipt["original_sha256"]}
    return result


def map_diagnostics(text, snapshot, views=None):
    """Preserve warning code/message and map only proven source coordinates."""
    snapshot = Path(snapshot).resolve()
    result = []
    for line in text.splitlines(keepends=True):
        match = re.match(r"(.+?):(\d+):(.*)", line)
        if not match:
            result.append(line)
            continue
        path, number, suffix = match.group(1), int(match.group(2)), match.group(3)
        resolved = str(Path(path).resolve())
        if views and resolved in views:
            origin = views[resolved]
            number -= origin["prefix_lines"]
            if number < 1 or number > origin["source_lines"]:
                raise ValueError("Diagnostic points outside the authenticated original header")
            path = origin["original"]
        else:
            try:
                path = str(Path(resolved).relative_to(snapshot))
            except ValueError:
                result.append(line)
                continue
        ending = "\n" if line.endswith("\n") else ""
        result.append(f"<parser-inputs>/{path}:{number}:{suffix}{ending}")
    return "".join(result)
