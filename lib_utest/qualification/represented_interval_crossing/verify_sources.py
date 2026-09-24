#!/usr/bin/env python3
"""Verify the isolated represented-linear interval certificate wiring."""
from pathlib import Path
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
collision = root / "lib_src/collision"
numerical_headers = ("Modes.h", "Identity.h", "Arithmetic.h", "Geometry.h", "CellKernel.h", "ArithmeticContext.h", "BoostIntegerPolicy.h", "FixedIntegerPolicy.h")
source = "\n".join((collision / "represented_interval_crossing/native" / name).read_text()
                   for name in numerical_headers) + "\n" + (collision / "RepresentedIntervalCrossing.cpp").read_text()
public = (collision / "RepresentedIntervalCrossing.h").read_text()
types = (collision / "RepresentedIntervalCrossingTypes.h").read_text()
cmake = (collision / "RepresentedIntervalCrossing.cmake").read_text()
bazel = (collision / "BUILD.bazel").read_text()
test_cmake = (here / "CMakeLists.txt").read_text()
test_bazel = (here / "BUILD.bazel").read_text()
test_sources = (
    "BatchRosterTest.cpp", "NormalReuseTest.cpp", "RelativeSeparationTest.cpp", "ExactPathReuseTest.cpp", "CommonPointReuseTest.cpp", "NativeStorageTest.cpp", "FixedPolicyTest.cpp", "ResultAssertions.h", "GeometryTest.cpp", "InvarianceTest.cpp", "FailureTest.cpp",
    "IdentityTest.cpp", "DeepTest.cpp", "OracleTest.cpp",
    "ParallelTest.cpp", "TranslationProofTest.cpp", "Oracle.cpp",
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
    "dfs[dfs_size++]"
)
assert "RepresentedIntervalReason::UnsupportedMotion" in pair_body
assert "SweptBoxesSeparated" in source
assert "bool CommonTranslation(" in source
assert "Compare(displacement, reference[component])" in source
assert pair_body.index("CommonTranslation(a, b)") < pair_body.index(
    "while (dfs_size)"
)
translation_body = pair_body[pair_body.index("if (common_translation)"):
                             pair_body.index("while (dfs_size)")]
for tag in ("ExactCommonTranslationTransverse", "ExactCommonTranslationCoplanar"):
    assert tag in types
    assert tag in translation_body
    assert source.count(tag) == 1
assert "HasExactCommonTranslationProof" in types
assert "BaseIntersectionGeometry" in types
geometry_source = (collision / "represented_interval_crossing/native/Geometry.h").read_text()
assert "CertifiedCrossingContact" not in geometry_source[geometry_source.index("bool SweptBoxesSeparated"): ]
assert "swept AABB overlap is never called a crossing" in public
assert "RigidArc" in types and "Nonlinear" in types
assert "max_work_per_pair" in source and "max_total_work" in source
assert "storage.published.swap(storage.staging)" in source
assert "RepresentedIntervalResult Visit(" not in source
assert "Cell* dfs, std::size_t dfs_capacity" in source
assert "std::unique_ptr<ExactScratch[]>" in source
assert "std::unique_ptr<Cell[]>" in source
assert "vertex_ledger.reserve" in source
assert "vertex_ledger_capacity" in types and "exact_scratch_bytes" in types
assert "limits.max_paths > UINT32_MAX" in source
assert "inconsistent vertex trajectory identity" in source
assert "storage.staging.data()" in source and "storage.dfs_frames.get()" in source
assert "view expires on the next successful Certify" in public
assert "persistent worker pool" in public
assert "find_package(Threads REQUIRED)" in cmake
assert "Threads::Threads" in cmake and '"-pthread"' in bazel
assert "mmap(" in source and "mprotect(" in source
assert "pthread_attr_setstack" in source and "pthread_create" in source
assert "pthread_join" in source
assert "worker_stack_bytes" in types and "pair_status_bytes" in types
assert "RepresentedIntervalMaximumWorkerCount = 8" in types
certify_start = source.index(
    "RepresentedIntervalReport RepresentedIntervalCrossing::Certify(")
certify_end = source.index(
    "RepresentedIntervalForecast RepresentedIntervalCrossing::forecast()",
    certify_start)
certify_body = source[certify_start:certify_end]
for forbidden in ("pthread_create", "pthread_join", "mmap(", "new (",
                  "make_unique", ".reserve("):
    assert forbidden not in certify_body, forbidden
assert certify_body.index("storage.pairs.size() > storage.limits.max_results") < (
    certify_body.index("storage.staging.resize(storage.pairs.size())"))
assert "storage.RunWorkers" in certify_body
assert "storage.busy.compare_exchange_strong" in certify_body
assert "impl_->busy.load(std::memory_order_acquire)" in source
batch_header = (collision / "represented_interval_crossing/Batch.h").read_text()
batch_execution = (collision / "represented_interval_crossing/BatchExecution.h").read_text()
for name in ("represented_interval_crossing/Batch.h",
             "represented_interval_crossing/BatchExecution.h"):
    assert name in cmake and name in bazel, name
assert "struct PathRoster" in source
assert "if (!roster.authenticated)" in source
assert "roster.authenticated = true" in source
assert "PathRosterWork" in batch_header
assert "BatchAccess::Certify(" in source
assert "storage.DisjointFromOwned(scratch, scratch_bytes)" in source
assert "report.path_roster_work = roster.work" in source
assert "report.results = {scratch, pair_count, true}" in batch_execution
assert "scratch[report.batch_offset + pair] = current.data[pair]" in batch_execution
for forbidden in ("malloc(", ".reserve(", "pthread_create", "new "):
    assert forbidden not in batch_execution, forbidden
assert "boost::multiprecision::cpp_rational" in (
    here / "Oracle.cpp").read_text()
for required_parallel in (
    "WorkerCountsMatchMixedCertificatesAndVariedPairOrder",
    "CanonicalTotalWorkFailureMatchesSerialAndRollsBackExactly",
    "InvalidInputReportsAndPriorPublicationMatchExactly",
    "PreflightBoundsPersistentWorkerStorageAndDestruction",
):
    assert required_parallel in tests

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
    "CommonTranslationMinimalTotalCapRollsBackAndRetriesExactly",
    "RoundedResidualMinimalCapFailureRollsBackExactly",
    "PersistentProxyMinimalCapFailureRollsBackExactly",
    "ResultCapMinusOneFailureIsAtomicAndSubsetRetrySucceeds",
    "DeepDyadicAffineContactUsesOwnedIterativeStack",
    "StaticDisjointTouchingBoxesUseExactGeometryCertificate",
    "ExactCommonTranslationCertifiesSeparatedYarisGeometryInOneVisit",
    "ExactCommonTranslationKeepsActualContactAndFeatureRepresented",
    "TranslationProofPreservesPriorEnumOrdinalsAndWitnessShape",
    "ExactTranslationProofRetainsStaticAndMovingSharedTopologyWitnesses",
    "TranslationProofDoesNotConvertNonlocalOverlapIntoLocalAuthority",
    "EqualRoundedDisplacementsCannotForgeExactTranslationProof",
    "UnsupportedCurvedMotionCannotBorrowTranslationProofFromEndpoints",
    "TranslationProofPreservesSeparatedAndDegenerateClassifications",
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
batch_found = set(re.findall(
    r"TEST\(RepresentedIntervalBatchRoster,\s*(\w+)\)", tests))
required_batch = (
    "LegacySlicesMatchAtOneTwoAndFivePairs",
    "WholeRosterAuthenticatesOnceAt256And257Boundary",
    "EmptyPairsStillAuthenticateEveryUnusedPath",
    "DistantIdentityConflictsPreserveNativeDiagnosticOrder",
    "MalformedPairsPrecedeUnusedPathAuthentication",
    "CompatibleUnusedDuplicateDoesNotChangeResults",
    "LateWorkFailurePreservesLastNativeSliceAndLimits",
    "SeparateCallsReauthenticateMutationFailureAndRetry",
    "ExpiredNativeScratchIsRejectedWithoutPublicationChange",
    "PerPairWorkExhaustionRemainsAnExplicitUnresolvedResult",
)
assert set(required_batch) <= batch_found

for forbidden in (
    "SelfContactBroadphase",
    "ShellPhysicalOwner",
    "active_use",
    "stiffness_per_area",
    "adaptive_timestep",
):
    assert forbidden not in public + types + source



# The retained exact normal cache is private, cell-local and fully forecast.
normal_implementation = source
for token in ("ExactVec3 normal_a[3]", "ExactVec3 normal_b[3]",
              "bool ready_a[3]", "bool ready_b[3]", "scratch->BeginCell()",
              "normal = kernel.Normal(second ? b[sample] : a[sample], counters)",
              "ready = true", "template <NormalReuse reuse = NormalReuse::Memoize,",
              "CertifyPair<NormalReuse::Recompute, SeparationProof::RelativeFaces, ExactPathReuse::Original, CommonPointReuse::Original, NativeStorage::Wide>", "sizeof(ExactScratch)"):
    if token not in normal_implementation:
        raise RuntimeError(f"Missing lazy normal cache contract: {token}")
normal_cell = normal_implementation[normal_implementation.index("CellEvaluation EvaluateCell("):
                                    normal_implementation.index("void RaiseReason(")]
if normal_cell.index("scratch->BeginCell()") > normal_cell.index("scratch->a[sample] = At("):
    raise RuntimeError("Exact normal readiness survives cell coordinate replacement")



projection_header = (collision / "represented_interval_crossing/ExactProjectionDomain.h").read_text()
for token in ("BOOST_VERSION == 107400", "encoded ? static_cast<int>(encoded) - 1023 - 52 : -1074",
              "(report.karatsuba_cutoff - 1) * report.limb_bits", "maximum_cell_depth + 1",
              "ExactProjectionDomain() = default"):
    assert token in projection_header, token
for token in ("SeparationProof::RelativeFaces", "SeparationProof::LegacyAabb",
              "EvaluateCell<reuse, SeparationProof::LegacyAabb, ExactPathReuse::Original, false, point_reuse>",
              "ProjectionDomain::FromPaths(a, b, limits.max_depth)",
              "CanonicalAnchor(a)", "RelativeCoordinatesSeparated", "RelativeAxisSeparated"):
    assert token in source, token
relative_cell = source[source.index("CellEvaluation EvaluateCell("):source.index("void RaiseReason(")]
assert relative_cell.index("if (degenerate)") < relative_cell.index("if (regular(false) && regular(true))")
assert relative_cell.index("SweptBoxesSeparated(scratch->a, scratch->b)") < relative_cell.index("!domain->eligible()")
assert relative_cell.index("!domain->eligible()") < relative_cell.index("RelativeCoordinatesSeparated(*scratch, anchor)")
assert "scratch->NormalAt(*this, side != 0, 0, counters)" in relative_cell

# Exact path reuse stays behind native-derived arithmetic and translation
# proofs; no caller profile or early arbitrary feature witness is introduced.
for token in ("ExactPathReuse path_reuse = ExactPathReuse::Optimized",
              "constexpr unsigned sample_count = single_sample ? 1 : 3",
              "domain && domain->eligible()", "CompareExactPathReuse",
              "RepresentedFeatureKind::VertexFace < RepresentedFeatureKind::EdgeEdge"):
    assert token in source, token
assert "ExactPathReuse" not in public + types
assert translation_body.index("domain.eligible()") < translation_body.index(
    "EvaluateCell<reuse, SeparationProof::LegacyAabb, path_reuse, true, point_reuse>")
feature_body = source[source.index("RepresentedFeaturePathKey IntersectionFeature("):
                      source.index("bool RegularCell(")]
assert feature_body.index("PointInClosedTriangle(b.vertex[vertex]") < feature_body.index(
    "if (skip_dominated_edges && have") < feature_body.index("for (unsigned edge_a")

# Common-point reuse is a private value fact derived from actual endpoint
# coordinates after unchanged sample regularity and arithmetic-domain gates.
assert "CommonPointReuse" not in public + types
assert "CommonEndpointPoint(path_a, path_b, times[sample]" in relative_cell
assert relative_cell.index("if (degenerate_at(false, sample)") < relative_cell.index("CommonEndpointPoint(")
assert relative_cell.index("if (domain && domain->eligible())") < relative_cell.index("CommonEndpointPoint(")
static_body = source[source.index("StaticIntersection Intersects("):source.index("bool PointInClosedTriangle(")]
assert static_body.index("result.coplanar &&") < static_body.index("if (common_endpoint)")
assert static_body.index("if (common_endpoint)") < static_body.index("if (SeparatedOnAxis(")
assert "CompareCommonPointReuse" in source
assert "SameFiniteCoordinate(p.x, q.x)" in source
assert "first == second || ((first & magnitude) == 0 && (second & magnitude) == 0)" in source

# Full native storage specialization retains immutable per-pair eligibility,
# original wide arenas and a separate noinline automatic narrow frame.
storage_domain = (collision / "represented_interval_crossing/NativeStorageDomain.h").read_text()
for token in ("proof.limb_bits != 64", "proof.karatsuba_cutoff != 40",
              "proof.coordinate_bits > 125", "maximum_product_limbs <= 8"):
    assert token in storage_domain, token
for token in ("using ExactScratch = WideKernel::ExactScratch", "__attribute__((noinline))",
              "sizeof(NarrowKernel::ExactScratch) <= 8192", "NarrowKernel::ExactScratch scratch",
              "NativeStorage::Wide", "NativeStorage::Adaptive"):
    assert token in source, token
assert "NativeStorage" not in public + types
# Per-pair instance state is explicit and cannot be cleared by a predicate.
assert "FixedIntegerPolicy" not in public + types
context = (collision / "represented_interval_crossing/native/ArithmeticContext.h").read_text()
fixed = (collision / "represented_interval_crossing/native/FixedIntegerPolicy.h").read_text()
boost = (collision / "represented_interval_crossing/native/BoostIntegerPolicy.h").read_text()
assert "template <unsigned, class> friend struct CellKernel" in context
assert "failed_ = failed_ || !valid" in context
assert source.count("context_.BeginPair()") == 1
assert "if (!kernel.Healthy()) return normal" in source
assert "context_.Observe(result.valid)" in fixed
assert "context_.Observe(!value.overflow)" in fixed
assert "TracksErrors = false" in boost and "TracksErrors = true" in fixed
boost_compact = re.sub(r"\s+", "", boost)
fixed_compact = re.sub(r"\s+", "", fixed)
for method in (
    "voidAssign(Integer&output,std::uint64_tvalue){output=value;}",
    "boolIsZero(constInteger&value)constnoexcept{returnvalue==0;}",
    "boolIsNegative(constInteger&value)constnoexcept{returnvalue<0;}",
    "IntegerAdd(constInteger&a,constInteger&b){returna+b;}",
    "IntegerNegate(constInteger&value){return-value;}",
    "IntegerMultiply(constInteger&a,constInteger&b){returna*b;}",
    "voidShift(Integer&value,unsignedamount){value<<=amount;}",
    "voidScale(Integer&value,std::uint64_tfactor){value*=factor;}",
    "intSign(constInteger&value)constnoexcept{returnvalue<0?-1:(value>0?1:0);}",
):
    assert method in boost_compact, method
for method in (
    "IntegerAdd(constInteger&a,constInteger&b)noexcept{returnChecked(Core::Add(a,b));}",
    "IntegerNegate(constInteger&value)noexcept{returnChecked(Core::Negate(value));}",
    "IntegerMultiply(constInteger&a,constInteger&b)noexcept{returnChecked(Core::Multiply(a,b));}",
    "voidShift(Integer&value,unsignedamount)noexcept{value=Checked(Core::ShiftLeft(value,amount));}",
    "voidScale(Integer&value,std::uint64_tfactor)noexcept{value=Checked(Core::Multiply(value,Core::FromU64(factor)));}",
):
    assert method in fixed_compact, method
assert "try { return body(); }" in boost and "catch (...)" in boost
assert "catch" not in fixed
assert "FixedKernel kernel(context)" in source
assert "WideKernel kernel(context)" in source and "NarrowKernel kernel(context)" in source
assert "thread_local" not in context + fixed + boost
import runpy
runpy.run_path(str(here / "verify_native_extraction.py"), run_name="__main__")

print(json.dumps({
    "status": "passed",
    "production_translation_units": 1,
    "host_functions": len(found),
    "batch_roster_functions": len(batch_found),
    "motion": "explicit represented LinearNodalV1 only",
    "numerical_execution": False,
}, sort_keys=True))
