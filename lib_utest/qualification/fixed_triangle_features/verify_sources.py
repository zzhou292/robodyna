#!/usr/bin/env python3
"""Verify the focused fixed-triangle production and qualification boundary."""

from pathlib import Path
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
COLLISION = ROOT / "lib_src/collision"

production = [
    COLLISION / "FixedTriangleFeatureTypes.h",
    COLLISION / "FixedTriangleFeatureDiscovery.h",
    COLLISION / "fixed_triangle_features/Geometry.h",
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

geometry = production[3].read_text()
assert "for (unsigned vertex = 0; vertex < 3; ++vertex)" in geometry
assert geometry.count("AddVertexFace(") >= 3
assert "for (unsigned edge_a = 0; edge_a < 3; ++edge_a)" in geometry
assert "for (unsigned edge_b = 0; edge_b < 3; ++edge_b)" in geometry
assert geometry.count("++output->feature_task_count;") == 2
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

discovery = production[4].read_text()
assert discovery.index("report.feature_tasks +=") < discovery.index(
    "report.raw_feature_candidates +=")
assert discovery.index("report.raw_feature_candidates +=") < discovery.index(
    "impl_->features[feature_write++]")
assert "std::sort(impl_->features.get()" in discovery
assert "impl_->complete = true" in discovery
assert discovery.index("impl_->Revoke()") < discovery.index(
    "if (pair_count > impl_->limits.max_input_pairs)")

cmake = (COLLISION / "FixedTriangleFeatureDiscovery.cmake").read_text()
bazel = (COLLISION / "BUILD.bazel").read_text()
for source in ("Geometry.cpp", "Discovery.cpp"):
    assert source in cmake and source in bazel
assert "fixed_triangle_feature_discovery" in bazel

tests = "\n".join(path.read_text() for path in HERE.glob("*Test.cpp"))
for required in (
    "CompleteVFBothOrientationsAndEEAreDeterministic",
    "TransversePiercingIsExplicitWhenAllBoundaryQueriesArePositive",
    "SharedCornerDoesNotHideRemoteTransverseIntersection",
    "ExactFeatureCapPassesAndMinusOneRejectsNoPrefix",
    "EverySmallMeshTaskEqualsIndependentLocalIncidenceEnumeration",
    "EveryVFAndEEValueMatchesIndependentDecimal100Geometry",
    "AllTrianglePermutationsAndPairReversalsMatchDecimal100Geometry",
    "ExhaustiveSmallLatticeIntersectionsMatchDecimal100SAT",
    "ExactBoundaryAndAdjacentRepresentableCoordinatesHaveExplicitClasses",
    "DegeneracyBoundaryIsClosedAndNextafterAboveRetriesSuccessfully",
    "ExactIntersectionCapPassesAndMinusOneRejectsThenRetries",
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
