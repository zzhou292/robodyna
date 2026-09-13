#!/usr/bin/env python3
"""Prove the isolated current-regularity boundary without running mechanics."""

from pathlib import Path
import json
import os

HERE = Path(__file__).resolve().parent
if os.environ.get("TEST_SRCDIR") and os.environ.get("TEST_WORKSPACE"):
    ROOT = Path(os.environ["TEST_SRCDIR"]) / os.environ["TEST_WORKSPACE"]
    HERE = ROOT / "lib_utest/qualification/self_contact_current_regularity"
else:
    ROOT = HERE.parents[2]
COLLISION = ROOT / "lib_src/collision"

production = [
    COLLISION / "SelfContactCurrentRegularity.h",
    COLLISION / "SelfContactCurrentRegularityTypes.h",
    COLLISION / "SelfContactCurrentRegularityValues.h",
    COLLISION / "SelfContactCurrentRegularity.cpp",
    COLLISION / "self_contact_current_regularity/Storage.h",
    COLLISION / "self_contact_current_regularity/Layout.cpp",
    COLLISION / "self_contact_current_regularity/Source.cpp",
    COLLISION / "self_contact_current_regularity/Values.cpp",
    COLLISION / "self_contact_current_regularity/Query.cpp",
]
for path in production:
    assert path.is_file(), path

public = production[0].read_text()
types = production[1].read_text()
lifecycle = production[3].read_text()
layout = production[5].read_text()
source = production[6].read_text()
values = production[7].read_text()
query = production[8].read_text()
all_production = "\n".join(path.read_text() for path in production)

assert "SelfContactActiveUseBinding binding" in production[4].read_text()
assert "ValidateSource(binding, layout.forecast)" in lifecycle
assert "strict deterministic source-EID order" in source
assert "facet_cursor != forecast.facets" in source
assert "publication_records = 2 *" in layout
assert "facet_staging_records = next.forecast.facets" in layout
assert "CurrentFixedTriangle" in layout
assert "PrepareQ4MaterialMeasure" in query
assert "CertifiedQ4FixedDirection" in query
assert "PrepareT3MaterialMeasure" in query
assert "activity.current[p] > activity.base[p]" in query
assert "LongInactiveSkipped" in query
assert "fixed->Approximation" in query
assert "EvaluateCurrentFacetRegularity" in query
assert "DirectedTriangle" in values
assert "64 * DBL_EPSILON" in values
assert "FacetOrientationMismatch" in query
assert "FacetDegenerate" in query
assert "std::swap(impl_->publication, impl_->staging)" in lifecycle
assert lifecycle.index("RunQuery(") < lifecycle.index(
    "std::swap(impl_->publication, impl_->staging)")
assert "owner_identity_" in types and "generation_" in types
assert "MatchesInputs" in types
assert "SameParentNeedsCurrentRegularity" in lifecycle
assert "pair.parent[0] != pair.parent[1]" in lifecycle
assert "ExcludedRegularOwnParent" in lifecycle
assert "candidate_directed_area_m2 = {}" in lifecycle
assert "admitted_force_area_m2 = {}" in lifecycle
assert "Authenticates(pair)" in lifecycle
assert "activity_base_identity" in lifecycle

for text in (lifecycle, query):
    assert "std::vector" not in text
for forbidden in (
    "ApplyPenalty",
    "AssembleAccepted",
    "SelfContactBroadphase",
    "RepresentedIntervalCrossing",
    "AdvanceStaggeredCin",
    "cudaMalloc",
    "cudaFree",
    "__global__",
):
    assert forbidden not in all_production, forbidden
assert "half_thickness" not in all_production
assert "approximation" not in lifecycle
assert "reference_half_thickness" not in query

cmake = (COLLISION / "SelfContactCurrentRegularity.cmake").read_text()
bazel = (COLLISION / "BUILD.bazel").read_text()
for source_name in (
    "SelfContactCurrentRegularity.cpp",
    "Layout.cpp",
    "Source.cpp",
    "Values.cpp",
    "Query.cpp",
):
    assert source_name in cmake and source_name in bazel
assert 'name = "self_contact_current_regularity"' in bazel
assert 'name = "self_contact_current_regularity_source_proof"' in bazel

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
    "FlatSkewWarpedQ4AndT3PassAtLevelsZeroOneTwo",
    "SaddleHasFixedDirectionChartAndPositiveApproximationMetadataOnly",
    "CollinearReversedAndNearDegenerateFacetsRejectExactly",
    "ZeroApproximationDoesNotAdmitDegenerateCurrentParent",
    "BaseActiveRemovalIsCertifiedAndLongInactiveGeometryIsExplicitlySkipped",
    "InvalidActivityAndReactivationPreserveCompleteQueryAndReceipt",
    "ExactFreshReceiptAloneExcludesCertifiedOwnParentWithZeroArea",
    "DifferentParentAndForeignBindingPairsAreNeverChangedByOwnParentReceipt",
    "ExactParentFacetAndByteCapsRejectOneShortThenRetry",
    "QueryAliasesAndLateParentFailurePreserveCompletePublicationThenRetry",
    "PermutedSelectionPublishesIdenticalSourceEidOrderedSummary",
):
    assert required in tests, required
assert "cpp_dec_float<100>" in (HERE / "Oracle.h").read_text()
assert "Q4ChartPositive" in tests
assert "DirectedTriangle" in tests

print(json.dumps({
    "status": "passed",
    "profile": "fixed-current-parent-chart-v1",
    "parents": "all-base-active-removals-included",
    "long_inactive": "explicit-skip-no-contact-ownership",
    "allocation_per_query": False,
    "force_or_clock": False,
    "receipt": "binding-generation-exact-input-identity",
}, sort_keys=True))
