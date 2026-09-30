#include "CandidateFailureFixture.h"

#include "CandidateFailureValues.h"
#include "FailureBaselineScope.h"
#include "case/vehicle_run/SelfContactDocument.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"
#include "output/BoundedArrayJson.h"

#include <cmath>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {
using namespace output;

Document EndpointCounts(const nonlinear_fixture::Pair& pair) {
    Document result;
    result.SetObject();
    for (unsigned side = 0; side < 2; ++side) {
        Document endpoint;
        endpoint.SetObject();
        const auto& features = side ? pair.prepared_features : pair.accepted_features;
        const auto& intersections = side ? pair.prepared_intersections : pair.accepted_intersections;
        std::size_t local = 0, nonlocal = 0;
        for (const auto& intersection : intersections) {
            if (contact::RequiresIntersectionAdmission(intersection))
                ++nonlocal;
            else
                ++local;
        }
        Integer(endpoint, "native_features", features.size());
        Integer(endpoint, "local_intersections", local);
        Integer(endpoint, "nonlocal_intersections", nonlocal);
        array_json::Child(result, side ? "prepared" : "accepted", endpoint);
    }
    return result;
}

Document Residual(const sct::LinearResidualSeparationResult& residual) {
    Document result;
    result.SetObject();
    Integer(result, "status", unsigned(residual.status));
    Boolean(result, "exact_common_translation", residual.exact_common_translation);
    String(result, "scope", "native masked residual result; thickness tasks only, not topology authority");
    const double values[]{residual.reference_translation.x, residual.reference_translation.y,
                          residual.reference_translation.z, residual.first_residual_upper_m,
                          residual.second_residual_upper_m, residual.prepared_distance_lower_m,
                          residual.strict_gap_lower_m};
    Value readable(rapidjson::kArrayType);
    bool all_finite = true;
    for (const auto value : values) {
        Value number;
        if (std::isfinite(value))
            number.SetDouble(value);
        else
            all_finite = false;
        readable.PushBack(number, result.GetAllocator());
    }
    Boolean(result, "all_finite", all_finite);
    if (all_finite)
        result.AddMember("reference_xyz_residuals_distance_gap", readable, result.GetAllocator());
    Value bits(rapidjson::kArrayType);
    for (const auto value : values)
        bits.PushBack(Bits(value), result.GetAllocator());
    result.AddMember("binary64_bits", bits, result.GetAllocator());
    return result;
}

}  // namespace

std::string CandidateFailureFixture::Export(const std::filesystem::path& directory) const {
    using namespace output;
    const auto& frozen = pair();
    Require(!std::filesystem::exists(directory), "Failure output directory must be absent");
    nonlinear_fixture::Writer encoded;
    nonlinear_fixture::AppendPair(&encoded, frozen);
    const auto bytes_upper = encoded.bytes().size() + 26 * sizeof(std::uint64_t);
    Require(bytes_upper <= FailureFixtureArchiveCap - FailureFixtureManifestCap,
            "Failure fixture exceeds fixed archive capacity");
    Require(std::filesystem::create_directory(directory), "Cannot create failure fixture directory");
    const auto profile_hash = failure_detail::ProfileHash(work_, depth_, nonlinear_work_, nonlinear_depth_);
    const auto dt_hash = failure_detail::DtHash(duration_, kick_dt_, phase_.prepared_trajectory);
    nonlinear_fixture::Write((directory / "pair.bin").string(), pairs_, profile_hash,
                             dt_hash, 0, phase_, false);
    const auto bytes = ReadBounded(directory / "pair.bin", FailureFixtureArchiveCap);
    Require(bytes.size() == bytes_upper, "Failure fixture encoding size differs from forecast");
    const auto checked = nonlinear_fixture::Read((directory / "pair.bin").string(), false);
    Require(checked.pairs.size() == 1, "Failure fixture readback differs from one pair");

    Document manifest;
    manifest.SetObject();
    String(manifest, "schema", failure_detail::FailureManifestSchema);
    Boolean(manifest, "baseline_observed", compact_.observed_failure_baseline);
    String(manifest, "scope", "one authenticated rejected pair; diagnostic only; no complete census or physics acceptance; capture storage budget excludes native proof stack and allocator overhead, governed by outer RSS guard");
    String(manifest, "baseline_scope", compact_.observed_failure_baseline
        ? failure_detail::ObservedBaselineScope : failure_detail::UnreportedBaselineScope);
    Boolean(manifest, "physics_accepted", false);
    Boolean(manifest, "owners_equivalent", owners_equivalent_);
    Integer(manifest, "capture_storage_budget_bytes", FailureCaptureHostCap);
    Integer(manifest, "archive_byte_cap", FailureFixtureArchiveCap);
    Integer(manifest, "owner_id", owner_);
    Integer(manifest, "attempt", attempt_);
    Integer(manifest, "accepted_epoch", phase_.accepted_epoch);
    Number(manifest, "duration_s", duration_);
    Number(manifest, "kick_dt_s", kick_dt_);
    Integer(manifest, "duration_bits", Bits(duration_));
    Integer(manifest, "kick_dt_bits", Bits(kick_dt_));
    Integer(manifest, "crossing_work", work_);
    Integer(manifest, "crossing_depth", depth_);
    Integer(manifest, "nonlinear_work", nonlinear_work_);
    Integer(manifest, "nonlinear_depth", nonlinear_depth_);
    Integer(manifest, "full_owner_count", full_owners_);
    Integer(manifest, "compact_owner_count", frozen.accepted_owners.size());
    String(manifest, "file", "pair.bin");
    String(manifest, "file_sha256", Sha256(bytes));
    Integer(manifest, "file_bytes", bytes.size());
    Integer(manifest, "schema_hash", checked.schema_hash);
    Integer(manifest, "source_hash", checked.source_hash);
    Integer(manifest, "profile_hash", checked.profile_hash);
    Integer(manifest, "dt_hash", checked.dt_hash);
    Integer(manifest, "payload_hash", checked.payload_hash);
    Integer(manifest, "roster_digest", checked.roster_digest);
    Document activity;
    activity.SetObject();
    Integer(activity, "selected", activity_.selected);
    Integer(activity, "accepted_active", activity_.accepted_active);
    Integer(activity, "prepared_active", activity_.prepared_active);
    Integer(activity, "removing", activity_.removing);
    Integer(activity, "inactive", activity_.inactive);
    Boolean(activity, "complete", activity_.complete);
    array_json::Child(manifest, "activity", activity);
    array_json::Child(manifest, "native_report", vehicle_run::detail::SelfContactErrorDocument(
        vehicle_self_contact::SelfContactStageError(report_,
            vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal, 0)));
    array_json::Child(manifest, "compact_replay", prepared_replay::PairDocument(compact_));
    array_json::Child(manifest, "full_ledger_replay", prepared_replay::PairDocument(full_));
    array_json::Child(manifest, "endpoint_counts", EndpointCounts(frozen));
    array_json::Child(manifest, "native_residual", Residual(residual_));
    array_json::Child(manifest, "geometry", failure_detail::Geometry(frozen));
    const auto manifest_bytes = failure_detail::JsonBytes(manifest);
    Require(bytes.size() + manifest_bytes.size() <= FailureFixtureArchiveCap,
            "Failure artifact exceeds total archive capacity");
    WriteBytes(directory / "failure.json", manifest_bytes);
    return Sha256(manifest_bytes);
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
