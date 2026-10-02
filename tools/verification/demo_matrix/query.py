"""Consume Bazel's expanded XML graph; do not interpret Starlark source."""

from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET


QUERY_EXPRESSION = 'kind(".* rule", //examples/... union @legacy_fea//lib_bin/... union //apps/precice:run_adapter)'
PUBLIC_EXPRESSION = 'visible(//apps/cli:robodyna, ' + QUERY_EXPRESSION + ')'
EXECUTABLE_KINDS = {"cc_binary", "cuda_binary", "py_binary", "managed_assembly"}


def canonical_label(label):
    if label.startswith("@@"):
        label = label[1:]
    if label.startswith("@"):
        repository, rest = label[1:].split("//", 1)
        if repository in ("", "_main", "robodyna"):
            return "//" + rest
        if repository == "legacy_fea" or repository.endswith(("+legacy_fea", "~legacy_fea")):
            return "@legacy_fea//" + rest
    return label


def source_path(label):
    label = canonical_label(label)
    if label.startswith("@legacy_fea//"):
        prefix, label = "src/fea/legacy/", label[len("@legacy_fea"):]
    elif label.startswith("//"):
        prefix = ""
    else:
        return None
    package, name = label[2:].split(":", 1)
    path = PurePosixPath(prefix + package) / name
    if path.is_absolute() or ".." in path.parts:
        raise ValueError("Unsafe Bazel source label")
    return str(path)


def read_query(path):
    """Stream a bounded query snapshot containing already-expanded real rules."""
    path = Path(path)
    if path.stat().st_size > 128 * 1024**2:
        raise ValueError("Query snapshot exceeds the 128MiB inventory allowance")
    rules = {}
    for _, element in ET.iterparse(path, events=("end",)):
        if element.tag != "rule":
            continue
        label = canonical_label(element.attrib["name"])
        if label in rules:
            raise ValueError("Duplicate queried rule: " + label)
        attributes = {}
        for child in element:
            name = child.attrib.get("name")
            if not name:
                continue
            if child.tag == "list":
                attributes[name] = [canonical_label(item.attrib["value"]) for item in child if "value" in item.attrib]
            elif "value" in child.attrib:
                attributes[name] = canonical_label(child.attrib["value"])
        rules[label] = {"kind": element.attrib["class"], "attributes": attributes,
                        "location": element.attrib.get("location", "")}
        element.clear()
    if not rules:
        raise ValueError("Query snapshot contains no expanded rules")
    return rules


def resolve_sources(label, rules, stack=()):
    label = canonical_label(label)
    if label in stack:
        raise ValueError("Cycle in queried source/alias ownership: " + label)
    if label not in rules:
        path = source_path(label)
        return {path} if path else set()
    rule = rules[label]
    attributes = rule["attributes"]
    if rule["kind"] == "alias":
        values = [attributes.get("actual", "")]
    elif rule["kind"] in {"filegroup", "managed_assembly", "cc_binary", "cuda_binary"}:
        values = attributes.get("srcs", [])
    else:
        return set()
    result = set()
    for value in values:
        result.update(resolve_sources(value, rules, stack + (label,)))
    return result


def read_public_labels(path):
    labels = set()
    for value in Path(path).read_text().splitlines():
        value = canonical_label(value.strip())
        if not value or not value.startswith(("//", "@")):
            raise ValueError("Invalid public-label query output")
        labels.add(value)
    return labels


def compiled_sources(label, rules, stack=()):
    """Follow actual compiled backends carried by a launcher, not raw C++ data."""
    label = canonical_label(label)
    if label in stack:
        raise ValueError("Cycle in queried launcher backends")
    if label not in rules:
        return set()
    rule = rules[label]
    attributes = rule["attributes"]
    if rule["kind"] in ("cc_binary", "cuda_binary") or (rule["kind"] == "managed_assembly" and attributes.get("kind") == "exe"):
        return resolve_sources(label, rules)
    values = [attributes.get("actual", "")] if rule["kind"] == "alias" else attributes.get("srcs", []) if rule["kind"] == "filegroup" else []
    result = set()
    for value in values:
        result.update(compiled_sources(value, rules, stack + (label,)))
    return result


def executable_sources(rules):
    result = {}
    for label, rule in rules.items():
        kind, attributes = rule["kind"], rule["attributes"]
        if kind not in EXECUTABLE_KINDS or (kind == "managed_assembly" and attributes.get("kind") != "exe"):
            continue
        sources = set()
        for value in attributes.get("srcs", []):
            sources.update(resolve_sources(value, rules))
        # The retained Python/managed launcher declares its original program as
        # data. Follow only source-bearing filegroups/assemblies, never arbitrary
        # implementation dependencies that would misattribute another main.
        if kind == "py_binary":
            for value in attributes.get("data", []):
                sources.update(path for path in resolve_sources(value, rules) if path.endswith(".py"))
                sources.update(compiled_sources(value, rules))
        result[label] = {"kind": kind, "sources": sorted(sources), "location": rule["location"]}
    for label, rule in rules.items():
        if rule["kind"] == "alias":
            actual, seen = rule["attributes"].get("actual"), {label}
            while actual in rules and rules[actual]["kind"] == "alias":
                if actual in seen:
                    raise ValueError("Cycle in queried executable aliases")
                seen.add(actual)
                actual = rules[actual]["attributes"].get("actual")
            if actual in result:
                result[label] = dict(result[actual], alias_for=actual)
    return result
