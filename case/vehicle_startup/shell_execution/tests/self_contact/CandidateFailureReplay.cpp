#include "CandidateFailureFixture.h"

#include "CandidateFailureValues.h"
#include "FailureBaselineScope.h"
#include "output/BoundedArrayJson.h"

#include <cmath>
#include <cstring>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {
using namespace output;
using namespace output::array_json;

void CheckFile(const std::filesystem::path& path) {
    const auto status = std::filesystem::symlink_status(path);
    Require(std::filesystem::is_regular_file(status) && !std::filesystem::is_symlink(status),
            "Failure artifact must be a regular nonsymlink file");
}

double FromBits(std::uint64_t value) {
    double result;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

void CheckActivity(const Value& value) {
    Keys(value, {"selected", "accepted_active", "prepared_active", "removing", "inactive", "complete"});
    Require(value["complete"].IsBool() && value["complete"].GetBool(),
            "Failure activity was incomplete");
    const auto selected = UInt(value["selected"]);
    const auto accepted = UInt(value["accepted_active"]);
    const auto prepared = UInt(value["prepared_active"]);
    const auto removing = UInt(value["removing"]);
    const auto inactive = UInt(value["inactive"]);
    Require(prepared <= accepted && removing == accepted - prepared &&
                accepted <= selected && inactive == selected - accepted,
            "Failure activity partition is inconsistent");
}

}  // namespace

output::Document ReplayCandidateFailure(const std::filesystem::path& manifest_path,
                                       const std::string& expected_sha256) {
    using namespace output;
    using namespace output::array_json;
    Require(expected_sha256.size() == 64, "Pin the failure manifest SHA-256 explicitly");
    CheckFile(manifest_path);
    const auto manifest_bytes = ReadBounded(manifest_path, FailureFixtureManifestCap);
    Require(Sha256(manifest_bytes) == expected_sha256, "Failure manifest SHA-256 mismatch");
    auto manifest = Parse(manifest_bytes, FailureFixtureManifestCap);
    const bool observed_baseline = failure_detail::ConsumeBaselineObserved(manifest);
    Keys(manifest, {"schema", "scope", "baseline_scope", "physics_accepted", "owners_equivalent",
                   "capture_storage_budget_bytes", "archive_byte_cap", "owner_id", "attempt", "accepted_epoch",
                   "duration_s", "kick_dt_s", "duration_bits", "kick_dt_bits", "crossing_work",
                   "crossing_depth", "nonlinear_work", "nonlinear_depth", "full_owner_count",
                   "compact_owner_count", "file", "file_sha256", "file_bytes", "schema_hash",
                   "source_hash", "profile_hash", "dt_hash", "payload_hash", "roster_digest",
                   "activity", "native_report", "compact_replay", "full_ledger_replay",
                   "endpoint_counts", "native_residual", "geometry"});
    Require((Text(manifest["schema"]) == failure_detail::LegacyFailureManifestSchema ||
             Text(manifest["schema"]) == failure_detail::FailureManifestSchema) &&
                Text(manifest["file"]) == "pair.bin" &&
                manifest["physics_accepted"].IsBool() && !manifest["physics_accepted"].GetBool() &&
                manifest["owners_equivalent"].IsBool() && manifest["owners_equivalent"].GetBool(),
            "Invalid failure fixture identity or acceptance claim");
    Require(UInt(manifest["capture_storage_budget_bytes"]) == FailureCaptureHostCap &&
                UInt(manifest["archive_byte_cap"]) == FailureFixtureArchiveCap,
            "Failure fixture bounds differ from supported profile");
    CheckActivity(manifest["activity"]);
    const auto work = UInt(manifest["crossing_work"]);
    const auto depth = UInt(manifest["crossing_depth"]);
    const auto nonlinear_work = UInt(manifest["nonlinear_work"]);
    const auto nonlinear_depth = UInt(manifest["nonlinear_depth"]);
    Require(work && work <= (1u << 20) && nonlinear_work && nonlinear_work <= (1u << 20) &&
                depth <= 52 && nonlinear_depth <= 52,
            "Failure replay work/depth exceed bounded profile");
    const auto duration = Real(manifest["duration_s"]);
    const auto kick = Real(manifest["kick_dt_s"]);
    Require(duration > 0 && kick > 0 && Bits(duration) == UInt(manifest["duration_bits"]) &&
                Bits(kick) == UInt(manifest["kick_dt_bits"]),
            "Failure replay time fields disagree");
    const auto file_path = manifest_path.parent_path() / "pair.bin";
    CheckFile(file_path);
    const auto bytes = ReadBounded(file_path, FailureFixtureArchiveCap - manifest_bytes.size());
    Require(bytes.size() >= 26 * sizeof(std::uint64_t) &&
                bytes.size() == UInt(manifest["file_bytes"]) &&
                Sha256(bytes) == Text(manifest["file_sha256"]),
            "Failure pair size or SHA-256 mismatch");
    // The legacy codec reserves pair_count before decoding. Authenticate its
    // fixed header and one-pair cap before allowing that allocation.
    nonlinear_fixture::Reader header(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
    for (unsigned i = 0; i < 9; ++i)
        header.U64();
    Require(header.U64() == 1, "Failure fixture must contain exactly one pair");
    const auto file = nonlinear_fixture::Read(file_path.string(), false);
    Require(file.pairs.size() == 1 && file.nonlinear_roster_digest == 0 &&
                file.schema_hash == UInt(manifest["schema_hash"]) &&
                file.source_hash == UInt(manifest["source_hash"]) &&
                file.payload_hash == UInt(manifest["payload_hash"]) &&
                file.roster_digest == UInt(manifest["roster_digest"]) &&
                file.profile_hash == UInt(manifest["profile_hash"]) &&
                file.profile_hash == failure_detail::ProfileHash(work, depth, nonlinear_work, nonlinear_depth) &&
                file.dt_hash == UInt(manifest["dt_hash"]) &&
                file.dt_hash == failure_detail::DtHash(duration, kick, file.phase.prepared_trajectory),
            "Failure fixture source/profile/codec identity mismatch");
    Require(file.phase.accepted_epoch == UInt(manifest["accepted_epoch"]) &&
                Bits(FromBits(file.phase.prepared_time_bits) -
                     FromBits(file.phase.prepared_base_time_bits)) == Bits(duration),
            "Failure fixture phase duration mismatch");
    const auto& pair = file.pairs.front();
    Require(pair.accepted_owners.size() == UInt(manifest["compact_owner_count"]) &&
                pair.accepted_owners.size() <= UInt(manifest["full_owner_count"]),
            "Failure fixture owner counts disagree");
    Require(failure_detail::Geometry(pair) == manifest["geometry"],
            "Failure fixture geometry document differs from exact blob");
    bool affine = true;
    for (const auto& coefficients : pair.quadratic)
        for (const auto& vertex : coefficients.q)
            for (const auto& component : vertex)
                affine = affine && component.lower == 0 && component.upper == 0;
    const auto replay = failure_detail::Evaluate(pair, duration,
        affine ? work : nonlinear_work, affine ? depth : nonlinear_depth,
        pair.accepted_owners.data(), pair.accepted_owners.size(),
        observed_baseline);
    const auto actual = prepared_replay::PairDocument(replay);
    Document result;
    result.SetObject();
    String(result, "schema", "robo_dyna.self_contact_failure_replay.v1");
    String(result, "manifest_sha256", expected_sha256);
    Boolean(result, "physics_accepted", false);
    Boolean(result, "matches_captured_compact_result", actual == manifest["compact_replay"]);
    // A later source revision may reclassify the exact pair. Preserve both
    // results; diagnostic replay must not assert that a correction is invalid.
    Child(result, "replay", actual);
    Value original;
    original.CopyFrom(manifest["native_report"], result.GetAllocator());
    result.AddMember("original_rejection", original, result.GetAllocator());
    Child(result, "geometry", failure_detail::Geometry(pair));
    return result;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
