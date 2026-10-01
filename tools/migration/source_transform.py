"""Authenticate reviewed source transformations against immutable import hashes."""

import hashlib
import re
from pathlib import Path


def replace_code_token(text, before, after):
    """Replace one C++ token spelling, preserving comments and literal includes."""
    pattern = re.compile(
        r"^[ \t]*#[ \t]*include[^\n]*|//[^\n]*|/\*[\s\S]*?\*/"
        r'|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
        r"|(?P<token>\b" + re.escape(before) + r"\b)", re.MULTILINE)
    count = 0
    def replace(match):
        nonlocal count
        if match.group("token") is None:
            return match.group(0)
        count += 1
        return after
    return pattern.sub(replace, text), count


def relative_file(root, name):
    path = Path(name)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"Invalid repository-relative transformation path: {name}")
    return Path(root) / path


def original_bytes(root, entry):
    """Reverse only declared edits and require the exact original byte digest.

    Arithmetic edits, missing edits, extra edits and stale forwarding headers all
    fail. The original manifest is never repinned to accept transformed content.
    """
    current = relative_file(root, entry["canonical_path"])
    encoding = entry.get("encoding", "utf-8")
    if encoding not in ("utf-8", "latin-1"):
        raise ValueError("Unsupported reviewed source encoding")
    # A few retained source comments use Latin-1. Require that explicit recipe
    # choice and preserve bytes exactly; never decode with replacement/ignore.
    text = current.read_bytes().decode(encoding)
    for replacement in reversed(entry["replacements"]):
        before, after, count = (replacement[name] for name in ("before", "after", "count"))
        if not before or not after or type(count) is not int or count <= 0:
            raise ValueError("Invalid reviewed source replacement")
        if replacement.get("code_token", False):
            text, actual_count = replace_code_token(text, after, before)
            if actual_count != count:
                raise ValueError(f"Reviewed code token count differs: {current}: {after}")
        elif replacement.get("word", False):
            pattern = r"\b" + re.escape(after) + r"\b"
            if len(re.findall(pattern, text)) != count:
                raise ValueError(f"Reviewed token count differs: {current}: {after}")
            text = re.sub(pattern, lambda _: before, text)
        else:
            if text.count(after) != count:
                raise ValueError(f"Reviewed replacement count differs: {current}: {after}")
            text = text.replace(after, before)
    restored = text.encode(encoding)
    if hashlib.sha256(restored).hexdigest() != entry["original_sha256"]:
        raise ValueError(f"Source differs beyond its reviewed transformation: {current}")
    old = relative_file(root, entry["original_path"])
    if "forwarding_sha256" in entry:
        if hashlib.sha256(old.read_bytes()).hexdigest() != entry["forwarding_sha256"]:
            raise ValueError(f"Compatibility forwarding header differs: {old}")
    elif old != current and old.exists():
        raise ValueError(f"Relocated implementation still exists at its original path: {old}")
    return restored


def index_entries(document):
    if document.get("schema") != "robodyna.source_transformations.v1":
        raise ValueError("Unsupported source transformation schema")
    result = {}
    for entry in document["files"]:
        key = entry["original_path"]
        if key in result:
            raise ValueError(f"Duplicate transformed source: {key}")
        result[key] = entry
    return result
