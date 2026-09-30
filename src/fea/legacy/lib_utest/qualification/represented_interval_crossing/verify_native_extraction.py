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
    r"^[ \t]*(?:(?:static|inline)\s+)*(?:const\s+)?(?:int|unsigned|bool|void|double|std::uint64_t|Dyadic|ExactVec3|ExactTriangle|StaticIntersection|RepresentedFeaturePathKey|RepresentedIntervalResult|CellEvaluation|ProjectionHull|DyadicTime)[ \t&]+(?P<name>\w+)\s*\(", re.M)

def policy_baseline(name, value):
    # Exact, enumerated adapter spellings only. No generic statement stripping.
    replacements = {
        # Execution-space standard-library equivalents; never geometry changes.
        "portable::memcpy": "std::memcpy",
        "portable::fill_n": "std::fill_n",
        "portable::tie": "std::tie",
        "portable::swap": "std::swap",
        "portable::min": "std::min",
        "constexprunsignedendpoint_indices[]{0,2};for(unsignedendpoint:endpoint_indices)": "for(unsignedendpoint:{0u,2u})",
        "integers_.Assign(result.numerator,fraction);": "result.numerator=fraction;",
        "integers_.Assign(result.numerator,(std::uint64_t{1}<<52)|fraction);": "result.numerator=(std::uint64_t{1}<<52)|fraction;",
        "integers_.Negate(result.numerator)": "-result.numerator",
        "integers_.Negate(value.numerator)": "-value.numerator",
        "integers_.IsZero(a.numerator)": "a.numerator==0",
        "integers_.IsZero(b.numerator)": "b.numerator==0",
        "constautoshift=[this]": "constautoshift=[]",
        "integers_.IsNegative(*value)": "*value<0",
        "integers_.Negate(*value)": "-*value",
        "integers_.Shift(*value,amount);": "*value<<=amount;",
        "integers_.Add(a.numerator,b.numerator)": "a.numerator+b.numerator",
        "integers_.Multiply(a.numerator,b.numerator)": "a.numerator*b.numerator",
        "integers_.Scale(value.numerator,factor);": "value.numerator*=factor;",
        "integers_.Sign(value.numerator)": "value.numerator<0?-1:(value.numerator>0?1:0)",
        "normal=kernel.Normal(": "normal=Normal(",
        "if(!kernel.Healthy())returnnormal;": "",
        "NormalAt(kernel,second,": "NormalAt(second,",
        "returnkernel.RegularCell(": "returnRegularCell(",
        "[this,coordinate]": "[coordinate]", "[this,&axis]": "[&axis]",
        "scratch->NormalAt(*this,": "scratch->NormalAt(",
        "scratch->Regular(*this,": "scratch->Regular(",
    }
    for before, after in replacements.items():
        value = value.replace(before, after)
    if name == "CertifyPair":
        failure = "returnUnresolved(key,RepresentedIntervalReason::ExactArithmeticRange,work);"
        assert value.count("context_.BeginPair();") == 1
        assert value.count("if(!Healthy())" + failure) == 3
        assert "constboolcommon_translation=CommonTranslation(a,b);if(!Healthy())" + failure + "if(common_translation)" in value
        assert value.count("if(!Healthy())" + failure + "if(evaluation.disposition==CellDisposition::Crossing)") == 2
        value = value.replace("context_.BeginPair();", "")
        value = value.replace("returnintegers_.Protect([&]()->RepresentedIntervalResult{", "")
        value = value.replace("},[&](){" + failure + "});", "")
        value = value.replace("if(!Healthy())" + failure, "")
        value = value.replace("constboolcommon_translation=CommonTranslation(a,b);if(common_translation)", "if(CommonTranslation(a,b))")
        value = value.replace("dfs[dfs_size++]={};", "dfs[dfs_size++]={};try{")
        value = value.replace("if(all_leaves_separated)", "}catch(...){" + failure + "}if(all_leaves_separated)")
    return value

def bodies(text):
    clean = re.sub(r"//[^\n]*|/\*.*?\*/", "", text, flags=re.S)
    # Execution-space annotation only; constructors are outside the frozen
    # numerical-body set and pinned by the owning source check.
    clean = clean.replace("TL_MATH_HOST_DEVICE ", "")
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
        yield match.group("name"), hashlib.sha256(policy_baseline(match.group("name"), normalized).encode()).hexdigest()

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
healthy = ("Healthy", hashlib.sha256(b"{ifconstexpr(IntegerPolicy::TracksErrors)returncontext_.valid();elsereturntrue;}").hexdigest())
assert actual[healthy] == 1
actual.subtract([healthy])
actual += Counter()  # Remove the now-zero bridge entry before exact comparison.
expected = Counter((record["name"], record["sha256"]) for record in manifest["functions"])
assert len(manifest["functions"]) == 69
if actual != expected:
    raise RuntimeError("Native body extraction changed: missing=" + str(expected - actual) +
                       "; unexpected=" + str(actual - expected))
print(json.dumps({"native_baseline_bodies_verified_after_explicit_policy_adapters": sum(expected.values()),
                  "baseline_commit": manifest["baseline_commit"],
                  "numerical_execution": False}, sort_keys=True))
