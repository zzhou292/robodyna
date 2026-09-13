#!/usr/bin/env python3
"""Verify the focused fixed-triangle production and qualification boundary."""

from pathlib import Path
import json
import os

HERE = Path(__file__).resolve().parent
if os.environ.get("TEST_SRCDIR") and os.environ.get("TEST_WORKSPACE"):
    ROOT = (Path(os.environ["TEST_SRCDIR"]) /
            os.environ["TEST_WORKSPACE"])
    HERE = ROOT / "lib_utest/qualification/fixed_triangle_features"
else:
    ROOT = HERE.parents[2]
COLLISION = ROOT / "lib_src/collision"

production = [
    COLLISION / "FixedTriangleFeatureTypes.h",
    COLLISION / "FixedTriangleFeatureDiscovery.h",
    COLLISION / "fixed_triangle_features/Geometry.h",
    COLLISION / "fixed_triangle_features/ExactPredicates.h",
    COLLISION / "fixed_triangle_features/ExactPredicates.cpp",
    COLLISION / "fixed_triangle_features/Geometry.cpp",
    COLLISION / "fixed_triangle_features/Discovery.cpp",
]
for path in production:
    assert path.is_file(), path

text = "\n".join(path.read_text() for path in production)
for forbidden in (
    "cuda",
    "CUDA",
    "__global__",
    "same_pid",
    "SamePid",
    "effective_mass",
    "stiffness_per_area",
):
    assert forbidden not in text, forbidden

geometry = production[5].read_text()
assert "for (unsigned vertex = 0; vertex < 3; ++vertex)" in geometry
assert geometry.count("AddVertexFace(") >= 3
assert "for (unsigned edge_a = 0; edge_a < 3; ++edge_a)" in geometry
assert "for (unsigned edge_b = 0; edge_b < 3; ++edge_b)" in geometry
assert geometry.count("++result->feature_tasks;") == 2
assert "if (Same(a->key, b->key))\n    return" not in geometry
for required in (
    "CoplanarOverlap",
    "CoplanarTouch",
    "Transverse",
    "SharedVertexOnly",
    "SharedEdgeOnly",
    "IdenticalFace",
):
    assert required in text, required

discovery = production[6].read_text()
assert discovery.index("report.raw_feature_candidates +=") < discovery.index(
    "EvaluatePairFeaturesOnce(")
assert discovery.count("EvaluatePairFeaturesOnce(") == 1
assert discovery.index("report.raw_feature_candidates +=") < discovery.index(
    "impl_->features[i] = impl_->raw_features[i]")
assert "long double" not in geometry
assert "closest.weights[i] == 0" not in geometry
assert "closest.weights[i] == 1" not in geometry
assert "exact::ClosestStratum(" in geometry
assert "on_edge.parameter > 0 && on_edge.parameter < 1" in geometry
assert "weight > 0 && weight < 1" in geometry
assert "SameFeatureKey" in discovery
assert "std::sort(impl_->raw_features.get()" in discovery
assert "impl_->complete = true" in discovery
assert "preserves the last complete publication" in discovery
assert discovery.index("unique_features >") < discovery.index(
    "impl_->features[i] = impl_->raw_features[i]")

cmake = (COLLISION / "FixedTriangleFeatureDiscovery.cmake").read_text()
bazel = (COLLISION / "BUILD.bazel").read_text()
for source in ("ExactPredicates.cpp", "Geometry.cpp", "Discovery.cpp"):
    assert source in cmake and source in bazel
assert "fixed_triangle_feature_discovery" in bazel

tests = "\n".join(path.read_text() for path in HERE.glob("*Test.cpp"))
qualification_bazel = (HERE / "BUILD.bazel").read_text()
qualification_cmake = (HERE / "CMakeLists.txt").read_text()
for path in HERE.glob("*Test.cpp"):
    assert f'"{path.name}"' in qualification_bazel, path.name
    if path.name != "SourceProofTest.cpp":
        assert path.name in qualification_cmake, path.name
assert 'name = "source_proof"' in qualification_bazel
assert '"SourceProofTest.cpp"' in qualification_bazel
assert "sh_test(" not in qualification_bazel
for required in (
    "CompleteVFBothOrientationsAndEEAreDeterministic",
    "TransversePiercingIsExplicitWhenAllBoundaryQueriesArePositive",
    "SharedCornerDoesNotHideRemoteTransverseIntersection",
    "ExactFeatureCapPassesAndMinusOneRejectsNoPrefix",
    "EverySmallMeshTaskEqualsIndependentLocalIncidenceEnumeration",
    "EveryVFAndEEValueMatchesIndependentDecimal100Geometry",
    "AllTrianglePermutationsAndPairReversalsMatchDecimal100Geometry",
    "ExhaustiveSmallLatticeIntersectionsMatchExactDecimal100Clipping",
    "ExhaustiveTransverseGridMatchesIndependentDecimal100PlaneClipping",
    "ParallelCollinearAndNearParallelEdgeRecordsSurviveAllReversals",
    "ExactBoundaryAndAdjacentRepresentableCoordinatesHaveExplicitClasses",
    "DegeneracyBoundaryIsClosedAndNextafterAboveRetriesSuccessfully",
    "ExactIntersectionCapPassesAndMinusOneRejectsThenRetries",
    "RawIntersectionStagingMinusOneCountsAllThenRetries",
    "CanonicalVertexFaceStratumDeduplicatesProducingFacetSeam",
    "CanonicalTargetEdgeAndEdgePairDeduplicateAcrossFacetSeam",
    "GlobalLedgerRejectsConflictsOutsideDirectPairContexts",
    "OwnedPublicationAndControlAliasesRejectAndPreservePublication",
    "ExactEdgeIdentityIgnoresRoundedPositiveBoundaryWeight",
    "ExactVertexIdentityIgnoresRoundedSmallNonzeroWeight",
    "AdversarialExactEdgeStillDeduplicatesAcrossTargetSeam",
    "ClosedBoundaryNextafterAndOutsideVoronoiRegionsStayExact",
    "UnrepresentableFaceAndEdgeInteriorsRejectAllPermutations",
    "UnrepresentableInteriorFailurePreservesPublicationAndRetries",
):
    assert required in tests, required

print(json.dumps({
    "status": "passed",
    "production_files": len(production),
    "cuda_or_gpu_execution": False,
    "whole_parent_exclusions": False,
    "feature_tasks_per_pair": 15,
    "candidate_publication": "complete-count/sort/deduplicate/no-prefix",
}, sort_keys=True))
