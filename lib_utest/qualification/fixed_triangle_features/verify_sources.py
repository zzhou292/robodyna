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
    COLLISION / "fixed_triangle_features/ExactInteger.h",
    COLLISION / "fixed_triangle_features/ExactPredicateKernel.h",
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
public_header = production[1].read_text()
assert "for (unsigned vertex = 0; vertex < 3; ++vertex)" in geometry
assert geometry.count("AddVertexFace(") >= 3
assert "for (unsigned edge_a = 0; edge_a < 3; ++edge_a)" in geometry
assert "for (unsigned edge_b = 0; edge_b < 3; ++edge_b)" in geometry
assert geometry.count("++result->feature_tasks;") == 3
assert "PairLocalFeatureTaskMask(" in geometry
assert "BuildFixedTriangleFeatureTaskMask(" in geometry
assert "BuildFixedTriangleFeatureTaskMask(" in public_header
assert "EvaluatePairFeaturesMaskedOnce(" in geometry
assert "FixedTriangleVertexFaceTaskSlot(0, vertex)" in geometry
assert "FixedTriangleEdgeEdgeTaskSlot(edge_a, edge_b)" in geometry
assert "if (Same(a->key, b->key))\n    return" not in geometry
transverse_cone = geometry[
    geometry.index("bool CoplanarIncidentEdgeHasPositiveOverlap("):
    geometry.index("bool OnlySharedCoplanarVertex(")]
assert transverse_cone.count("exact::Orient2D(") == 2
for forbidden in ("epsilon", "nextafter", "fabs", "tolerance"):
    assert forbidden not in transverse_cone, forbidden
assert "CoplanarIncidentEdgeHasPositiveOverlap(" in geometry[
    geometry.index("bool OnlySharedTransverseVertex("):
    geometry.index("FixedTriangleDiscoveryStatus ClassifyIntersection(")]
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
discover_impl = discovery[discovery.index(
    "\nFixedTriangleDiscoveryReport "
    "FixedTriangleFeatureDiscovery::DiscoverImpl("):]
assert discover_impl.index("report.raw_feature_candidates +=") < (
    discover_impl.index("impl_->RunWorkers("))
assert discovery.count("EvaluatePairFeaturesMaskedOnce(") == 1
assert discovery.count("pthread_create(") == 1
assert "std::async" not in discovery
assert "std::thread" not in discovery
for forbidden in ("pthread_create(", "mmap(", "new (", "malloc("):
    assert forbidden not in discover_impl, forbidden
assert "Feature task mask omits a nonlocal or nonexistent task" in discovery
assert "ft::PairLocalFeatureTaskMask(" in discovery
assert "report.exact_executed_tasks +=" in discovery
assert discover_impl.index("report.raw_feature_candidates +=") < (
    discover_impl.index(
    "impl_->features[i] = impl_->raw_features[i]")
)
assert "RunWorkers(triangles, pairs, pair_count)" in discover_impl
assert "next_pair.fetch_add(" in discovery
assert "stage.raw_feature_offset" in discovery
assert "stage.expected_feature_count" in discovery
assert "busy.compare_exchange_strong(" in discovery
assert "Impl::Phase::Warm" in discovery
assert "pthread_join(" in discovery
assert "FixedTriangleFeatureMaximumWorkerCount" in text
assert "kWorkerStackBytes" in discovery
assert "long double" not in geometry
assert "closest.weights[i] == 0" not in geometry
assert "closest.weights[i] == 1" not in geometry
assert "exact::ClosestStratum(" in geometry
assert "on_edge.parameter > 0 && on_edge.parameter < 1" in geometry
assert "RepresentedFaceWeights" in geometry
assert "representation_error_m" in geometry
assert "SameFeatureKey" in discovery
assert "std::sort(impl_->raw_features.get()" in discovery
assert "impl_->complete = true" in discovery
assert "preserves the last complete publication" in discovery
assert discovery.index("unique_features >") < discovery.index(
    "impl_->features[i] = impl_->raw_features[i]")

cmake = (COLLISION / "FixedTriangleFeatureDiscovery.cmake").read_text()
bazel = (COLLISION / "BUILD.bazel").read_text()
for source in ("ExactPredicates.cpp", "ExactInteger.h", "ExactPredicateKernel.h", "Geometry.cpp", "Discovery.cpp"):
    assert source in cmake and source in bazel
assert "fixed_triangle_feature_discovery" in bazel
assert "Threads::Threads" in cmake
assert '"-pthread"' in bazel

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
    "AllFifteenSlotsMapAndIntersectionRemainsComplete",
    "SharedVertexAndSharedEdgePreserveEveryNonlocalObservation",
    "CoincidentDistinctIdsAndSameParentRemoteFeaturesStayUnmasked",
    "ExecutedNonlocalCapsPassExactlyAndMinusOnePublishesNoPrefix",
    "EverySmallMeshTaskEqualsIndependentLocalIncidenceEnumeration",
    "EveryVFAndEEValueMatchesIndependentDecimal100Geometry",
    "AllTrianglePermutationsAndPairReversalsMatchDecimal100Geometry",
    "ExhaustiveSmallLatticeIntersectionsMatchExactDecimal100Clipping",
    "ExhaustiveTransverseGridMatchesIndependentDecimal100PlaneClipping",
    "ParallelCollinearAndNearParallelEdgeRecordsSurviveAllReversals",
    "ExactBoundaryAndAdjacentRepresentableCoordinatesHaveExplicitClasses",
    "AuthenticatedCouponSharedVertexOnlyIsPermutationInvariant",
    "AuthenticatedBoundaryToParentDiagonalIsSharedEdgeOnly",
    "ExactNearSharedSegmentAndNoncanonicalCoincidenceStayNonlocal",
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
    "RoundedBoundaryCollapsePublishesEnclosedExactStratumMapping",
    "RoundedInteriorRepresentationPublishesAndBoundaryRetrySucceeds",
    "WorkerCountsMatchAcrossMasksCapsPermutationsAndRetries",
    "EarliestExactFailureMatchesSerialAndPreservesPublication",
    "PreflightBoundsMetadataStacksAndRepeatedDestruction",
):
    assert required in tests, required


# The public four-function API is independent of backend selection. Only the
# source-derived private domain may choose smaller storage; zeroing is retained.
integer = (COLLISION / "fixed_triangle_features/ExactInteger.h").read_text()
entry = (COLLISION / "fixed_triangle_features/ExactPredicates.cpp").read_text()
integer_core = (ROOT / "lib_src/math/FixedInteger.h").read_text()
assert "std::uint64_t limbs[kLimbs]{};" in integer_core
assert "tl::math::fixed_integer::Arithmetic<LimbCount, Sign>" in integer
assert "WideLimbs = 144" in integer and "SmallLimbs = 8" in integer
assert "SmallCoordinateBits = 125" in integer
assert "finite && coordinate_bits <= SmallCoordinateBits" in integer
assert "if (decoded.significand)" in integer
assert entry.count("Storage::Adaptive") == 4
assert "Storage::Wide" not in entry
assert "WideAdapter.cpp" not in cmake
for forbidden in ("thread_local", "getenv(", "malloc("):
    assert forbidden not in integer + integer_core + entry

print(json.dumps({
    "status": "passed",
    "production_files": len(production),
    "cuda_or_gpu_execution": False,
    "whole_parent_exclusions": False,
    "unmasked_feature_tasks_per_pair": 15,
    "masked_tasks": "exact-local-incidence-only",
    "candidate_publication": "complete-count/sort/deduplicate/no-prefix",
}, sort_keys=True))
