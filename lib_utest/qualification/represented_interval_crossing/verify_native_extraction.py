#!/usr/bin/env python3
"""Verify every moved native arithmetic body against the frozen3ed predecessor."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import re

root = Path(__file__).resolve().parents[3]
directory = root / "lib_src/collision/represented_interval_crossing/native"
manifest = json.loads((directory / "ExtractionManifest.json").read_text())
assert manifest["baseline_commit"] == "3edbf971242ff3c011b5b0f6433b356609b09fc4"
pattern = re.compile(
    r"^[ \t]*(?:(?:static|inline)\s+)*(?:const\s+)?(?:int|bool|void|double|std::uint64_t|Dyadic|ExactVec3|ExactTriangle|StaticIntersection|RepresentedFeaturePathKey|RepresentedIntervalResult|CellEvaluation|ProjectionHull|DyadicTime)[ \t&]+(?P<name>\w+)\s*\(", re.M)

def bodies(text):
    clean = re.sub(r"//[^\n]*|/\*.*?\*/", "", text, flags=re.S)
    for match in pattern.finditer(clean):
        at = clean.find("{", match.end())
        stop = clean.find(";", match.end())
        if at < 0 or (stop >= 0 and stop < at):
            continue
        depth = 0
        for end in range(at, len(clean)):
            depth += (clean[end] == "{") - (clean[end] == "}")
            if depth == 0:
                break
        if depth:
            raise RuntimeError("Unclosed extracted native function")
        normalized = re.sub(r"\s+", "", clean[at:end + 1])
        yield match.group("name"), hashlib.sha256(normalized.encode()).hexdigest()

actual = Counter()
for name in ("Modes.h", "Identity.h", "Arithmetic.h", "Geometry.h", "CellKernel.h"):
    actual.update(bodies((directory / name).read_text()))
# One name-lookup bridge is required because a class's inherited dyadic
# Compare hides namespace overloads. Its body may only forward the same keys.
geometry = (directory / "Geometry.h").read_text()
assert "static int Compare(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept" in geometry
bridge = ("Compare", hashlib.sha256(b"{returnnative::Compare(a,b);}").hexdigest())
assert actual[bridge] == 1
actual.subtract([bridge])
actual += Counter()  # Remove the now-zero bridge entry before exact comparison.
expected = Counter((record["name"], record["sha256"]) for record in manifest["functions"])
assert len(manifest["functions"]) == 68
if actual != expected:
    raise RuntimeError("Native body extraction changed: missing=" + str(expected - actual) +
                       "; unexpected=" + str(actual - expected))
print(json.dumps({"native_bodies_unchanged": sum(expected.values()),
                  "baseline_commit": manifest["baseline_commit"],
                  "numerical_execution": False}, sort_keys=True))
