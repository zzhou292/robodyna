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
test_sources = (
    "GeometryTest.cpp", "InvarianceTest.cpp", "FailureTest.cpp",
    "IdentityTest.cpp", "DeepTest.cpp", "OracleTest.cpp", "Oracle.cpp",
)
tests = "\n".join((here / name).read_text() for name in test_sources)

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
assert 'name = "source_identity"' in test_bazel
assert "represented_interval_crossing_source_proof" in bazel
for name in test_sources:
    assert name in test_cmake, ("CMake", name)
    assert name in test_bazel, ("Bazel", name)

body_start = source.index("RepresentedIntervalResult CertifyPair")
body_end = source.index("bool KnownMotion", body_start)
pair_body = source[body_start:body_end]
assert pair_body.index("RepresentedMotion::LinearNodalV1") < pair_body.index(
    "dfs->clear()"
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
assert "RepresentedIntervalResult Visit(" not in source
assert "std::vector<Cell>* dfs" in source
assert "std::unique_ptr<ExactScratch>" in source
assert "vertex_ledger.reserve" in source and "dfs.reserve" in source
assert "vertex_ledger_capacity" in types and "exact_scratch_bytes" in types
assert "limits.max_paths > UINT32_MAX" in source
assert "inconsistent vertex trajectory identity" in source
assert "storage.staging.data()" in source and "storage.dfs.data()" in source
assert "view expires on the next successful Certify" in public
assert "boost::multiprecision::cpp_rational" in (
    here / "Oracle.cpp").read_text()

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
    "DeepDyadicAffineContactUsesOwnedIterativeStack",
    "StaticDisjointTouchingBoxesUseExactGeometryCertificate",
    "NondyadicIsolatedContactRemainsUnresolvedNeverSeparated",
    "ExtremeBinary64ExponentsInterpolateWithoutFalseRangeResult",
    "IndependentExactRationalOracleChecksStaticSatAndFeatures",
    "ProductionCrossingWitnessesPassIndependentExactOracle",
    "CoplanarContainmentAndBoundaryAreClosedContactNotSeparation",
    "PreflightRejectsPathCountThatWouldTruncateIndices",
    "CanonicalFixedFacetIdentityRejectsMalformedVertexAndEdges",
    "SharedVertexLedgerRejectsDifferentTrajectoryOrMotion",
    "CompatibleDuplicatePathAndEdgesCanonicalizeToOnePair",
    "AllOwnedRangesRejectAliasesAndFailedCallsPreserveCurrentView",
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
