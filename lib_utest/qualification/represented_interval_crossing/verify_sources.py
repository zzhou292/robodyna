#!/usr/bin/env python3
"""Verify the isolated represented-linear interval certificate wiring."""
from pathlib import Path
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
collision = root / "lib_src/collision"
source = (collision / "RepresentedIntervalCrossing.cpp").read_text()
public = (collision / "RepresentedIntervalCrossing.h").read_text()
types = (collision / "RepresentedIntervalCrossingTypes.h").read_text()
cmake = (collision / "RepresentedIntervalCrossing.cmake").read_text()
bazel = (collision / "BUILD.bazel").read_text()
test_cmake = (here / "CMakeLists.txt").read_text()
test_bazel = (here / "BUILD.bazel").read_text()
tests = "\n".join(
    (here / name).read_text()
    for name in ("GeometryTest.cpp", "InvarianceTest.cpp", "FailureTest.cpp")
)

for name in (
    "RepresentedIntervalCrossing.cpp",
    "RepresentedIntervalCrossing.h",
    "RepresentedIntervalCrossingTypes.h",
):
    assert name in cmake or name in bazel, name
assert 'name = "represented_interval_crossing"' in bazel
assert "tl_represented_interval_crossing" in cmake
assert "represented_interval_crossing_host" in test_cmake
assert 'name = "host_check"' in test_bazel

body_start = source.index("RepresentedIntervalResult CertifyPair")
body_end = source.index("bool KnownMotion", body_start)
pair_body = source[body_start:body_end]
assert pair_body.index("RepresentedMotion::LinearNodalV1") < pair_body.index(
    "PairContext context"
)
assert "RepresentedIntervalReason::UnsupportedMotion" in pair_body
assert "SweptBoxesSeparated" in source
assert "CertifiedCrossingContact" not in source[
    source.index("bool SweptBoxesSeparated"):
    source.index("int ReasonPriority")
]
assert "swept AABB overlap is never called a crossing" in public
assert "RigidArc" in types and "Nonlinear" in types
assert "max_work_per_pair" in source and "max_total_work" in source
assert "storage.published.swap(storage.staging)" in source

required = (
    "PassThroughWithSeparatedEndpointsCertifiesMiddleContact",
    "EnterAndExitBetweenEndpointsCertifiesTransverseIntersection",
    "NearGrazingRepresentableGapIsCertifiedSeparated",
    "VertexFaceWitnessUsesImmutableVertexAndFaceKeys",
    "EdgeEdgeWitnessUsesSortedImmutableEdgeKeys",
    "TransverseTriangleIntersectionIsNotReducedToEndpointDistances",
    "CoplanarPassThroughIsCertifiedAtInteriorDyadicTime",
    "ExactBoundaryAndNextafterOnBothSidesRemainDistinct",
    "SourcePairOrderAndWindingPermutationLeaveCertificatesInvariant",
    "DegenerateRepresentedTriangleIsExplicitlyUnresolved",
    "UnsupportedRigidArcIsUnresolvedBeforeEndpointBoxReasoning",
    "PerPairWorkExhaustionPublishesExplicitUnresolvedRecord",
    "CheckedTotalWorkExhaustionPreservesPublicationAndAllowsRetry",
    "ResultCapMinusOneFailureIsAtomicAndSubsetRetrySucceeds",
)
found = set(re.findall(r"TEST\(RepresentedIntervalCrossing,\s*(\w+)\)", tests))
assert set(required) <= found

for forbidden in (
    "SelfContactBroadphase",
    "ShellPhysicalOwner",
    "active_use",
    "stiffness_per_area",
    "adaptive_timestep",
):
    assert forbidden not in public + types + source

print(json.dumps({
    "status": "passed",
    "production_translation_units": 1,
    "host_functions": len(found),
    "motion": "explicit represented LinearNodalV1 only",
    "numerical_execution": False,
}, sort_keys=True))
