#include "CensusFixtureExport.h"
#include "CoverageFixtureCapture.h"

#include <array>
#include <atomic>
#include <exception>
#include <iostream>
#include <numeric>
#include <thread>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {
namespace fixture = nonlinear_fixture;
using Snapshot = vehicle_self_contact::AcceptedAssemblyCouponSnapshot;
constexpr std::size_t PairsPerShard = 128;
constexpr std::size_t ArchiveCap = std::size_t{2} << 30;
constexpr std::size_t ManifestReserve = std::size_t{1} << 20;
constexpr std::size_t HeaderBytes = 26 * sizeof(std::uint64_t);

void HashKey(const contact::RepresentedTrianglePathKey& key,
             std::uint64_t* digest) {
    fixture::HashUnsigned(key.source_instance_id, digest);
    fixture::HashUnsigned(key.parent_eid, digest);
    fixture::HashUnsigned(key.level, digest);
    fixture::HashUnsigned(key.local_facet, digest);
}

std::uint64_t CensusDigest(const Snapshot& snapshot) {
    std::uint64_t digest = 1469598103934665603ull;
    for (const auto& row : snapshot.nonlinear_roster) {
        HashKey(row.key.paths[0], &digest);
        HashKey(row.key.paths[1], &digest);
        fixture::HashUnsigned(static_cast<unsigned>(row.separation.status), &digest);
        fixture::HashUnsigned(row.separation.work, &digest);
        fixture::HashUnsigned(row.separation.deepest, &digest);
    }
    for (const auto& row : snapshot.linear_roster) {
        HashKey(row.key.paths[0], &digest);
        HashKey(row.key.paths[1], &digest);
        fixture::HashUnsigned(row.task_mask.local_tasks, &digest);
        fixture::HashUnsigned(static_cast<unsigned>(row.crossing.classification), &digest);
        fixture::HashUnsigned(static_cast<unsigned>(row.crossing.reason), &digest);
        fixture::HashUnsigned(row.crossing.work, &digest);
    }
    return digest;
}

class Shards {
  public:
    Shards(const Snapshot& snapshot, const std::filesystem::path& destination,
           output::Document& manifest, std::uint64_t profile_hash,
           std::uint64_t dt_hash, std::uint64_t census_digest)
        : snapshot_(snapshot), destination_(destination), manifest_(manifest),
          profile_hash_(profile_hash), dt_hash_(dt_hash), census_digest_(census_digest) {
        pending_.reserve(PairsPerShard);
        manifest_.AddMember("files", output::Value(rapidjson::kArrayType),
                            manifest_.GetAllocator());
    }

    void Append(const char* family, fixture::Pair pair) {
        if (!pending_.empty() && family_ != family) Flush();
        family_ = family;
        pending_.push_back(std::move(pair));
        if (pending_.size() == PairsPerShard) Flush();
    }

    void Flush() {
        if (pending_.empty()) return;
        FreezeAcceptedPolicies(snapshot_, &pending_);
        std::size_t encoded_bytes = HeaderBytes;
        {
            fixture::Writer size_probe;
            for (const auto& pair : pending_) fixture::AppendPair(&size_probe, pair);
            output::Require(size_probe.bytes().size() <= fixture::MaximumBytes - HeaderBytes,
                            "Second-candidate fixture shard exceeds codec cap");
            encoded_bytes += size_probe.bytes().size();
        }
        output::Require(encoded_bytes <= ArchiveCap - ManifestReserve - bytes_,
                        "Complete second-candidate fixture archive exceeds 2 GiB cap");
        const auto filename = family_ + "-" + std::to_string(files_) + ".bin";
        const auto path = destination_ / filename;
        output::Require(!std::filesystem::exists(path),
                        "Diagnostic fixture output already exists");
        fixture::Write(path.string(), pending_, profile_hash_, dt_hash_,
                       census_digest_, FixturePhaseIdentity(snapshot_), false);
        const auto decoded = fixture::Read(path.string(), false);
        output::Require(decoded.pairs.size() == pending_.size() &&
                            decoded.phase.accepted_epoch == snapshot_.accepted_stamp.epoch &&
                            decoded.phase.prepared_base_epoch == snapshot_.accepted_stamp.epoch &&
                            decoded.profile_hash == profile_hash_ && decoded.dt_hash == dt_hash_,
                        "Written second-candidate fixture failed its complete readback");
        const auto bytes = output::ReadBounded(path, fixture::MaximumBytes + HeaderBytes);
        output::Require(bytes.size() == encoded_bytes,
                        "Second-candidate fixture byte accounting differs");
        output::Document file;
        file.SetObject();
        output::String(file, "file", filename);
        output::String(file, "sha256", output::Sha256(bytes));
        output::String(file, "family", family_);
        output::Integer(file, "bytes", bytes.size());
        output::Integer(file, "pairs", pending_.size());
        output::Integer(file, "source_hash", decoded.source_hash);
        output::Integer(file, "payload_hash", decoded.payload_hash);
        output::Integer(file, "roster_digest", decoded.roster_digest);
        output::Value entry;
        entry.CopyFrom(file, manifest_.GetAllocator());
        manifest_["files"].PushBack(entry, manifest_.GetAllocator());
        bytes_ += bytes.size();
        ++files_;
        pending_.clear();
    }

    std::size_t files() const noexcept { return files_; }
    std::size_t bytes() const noexcept { return bytes_; }

  private:
    const Snapshot& snapshot_;
    const std::filesystem::path& destination_;
    output::Document& manifest_;
    std::uint64_t profile_hash_, dt_hash_, census_digest_;
    std::string family_;
    std::vector<fixture::Pair> pending_;
    std::size_t files_ = 0, bytes_ = 0;
};
}  // namespace

CensusFixtureExportResult ExportPreparedCensus(
    const Snapshot& snapshot, const std::filesystem::path& destination,
    const output::Document& profile, std::uint64_t profile_hash,
    std::uint64_t dt_hash) {
    output::Require(snapshot.prepared_census && snapshot.prepared_census_receipt.valid() &&
                        snapshot.nonlinear_summary.complete &&
                        snapshot.nonlinear_summary.roster_complete &&
                        snapshot.linear_summary.complete && snapshot.linear_summary.roster_complete &&
                        snapshot.motion_certificates.complete &&
                        snapshot.nonlinear_roster.size() == snapshot.nonlinear_summary.nonlinear_pairs &&
                        snapshot.linear_roster.size() == snapshot.linear_summary.represented_work_exhausted,
                    "Second-candidate export requires a complete actual prepared-activity census");
    output::Require(!std::filesystem::exists(destination),
                    "Second-candidate export must preserve existing destinations");
    output::Require(std::filesystem::create_directories(destination),
                    "Cannot create second-candidate export destination");
    output::Document manifest;
    manifest.SetObject();
    output::String(manifest, "schema", "robo_dyna.prepared_self_contact_census.v1");
    output::String(manifest, "scope",
        "one committed interval; complete next-candidate diagnostic census; second interval not committed");
    output::Boolean(manifest, "second_interval_committed", false);
    output::Boolean(manifest, "actual_prepared_activity", true);
    output::Value profile_copy;
    profile_copy.CopyFrom(profile, manifest.GetAllocator());
    manifest.AddMember("profile", profile_copy, manifest.GetAllocator());
    output::Integer(manifest, "owner_id", snapshot.accepted_stamp.owner_id);
    output::Integer(manifest, "accepted_epoch", snapshot.accepted_stamp.epoch);
    output::Integer(manifest, "prepared_attempt", snapshot.prepared_view.attempt);
    output::Number(manifest, "accepted_time_s", snapshot.accepted_stamp.time);
    output::Number(manifest, "prepared_base_time_s", snapshot.prepared_view.base_time);
    output::Number(manifest, "prepared_time_s", snapshot.prepared_view.proposed_time);
    output::Number(manifest, "prepared_kick_dt_s", snapshot.prepared_view.kick_dt);
    output::Integer(manifest, "profile_hash", profile_hash);
    output::Integer(manifest, "dt_hash", dt_hash);
    output::Integer(manifest, "diagnostic_host_upper_bound", snapshot.diagnostic_host_upper_bound);
    output::Integer(manifest, "archive_cap_bytes", ArchiveCap);
    const auto digest = CensusDigest(snapshot);
    output::Integer(manifest, "complete_census_digest", digest);
    const auto activity = snapshot.prepared_census_receipt.activity_summary();
    output::Require(activity.complete, "Prepared activity accounting is incomplete");
    output::Integer(manifest, "selected_parents", activity.selected);
    output::Integer(manifest, "accepted_active_parents", activity.accepted_active);
    output::Integer(manifest, "prepared_active_parents", activity.prepared_active);
    output::Integer(manifest, "removing_parents", activity.removing);
    output::Integer(manifest, "inactive_parents", activity.inactive);

    Shards shards(snapshot, destination, manifest, profile_hash, dt_hash, digest);
    CensusFixtureExportResult result;
    for (const auto& entry : snapshot.linear_roster) {
        output::Require(entry.crossing.classification == contact::RepresentedIntervalClassification::Unresolved &&
                            entry.crossing.reason == contact::RepresentedIntervalReason::WorkExhausted,
                        "Linear failure roster contains an unexpected crossing outcome");
        shards.Append("linear", FreezeLinearPair(snapshot, entry));
        ++result.linear_pairs;
    }
    shards.Flush();

    std::vector<NonlinearPairEvidence> evidence(snapshot.nonlinear_roster.size());
    std::atomic<std::size_t> next{0};
    const auto inspect = [&] {
        for (;;) {
            const auto index = next.fetch_add(1, std::memory_order_relaxed);
            if (index >= snapshot.nonlinear_roster.size()) return;
            const auto& entry = snapshot.nonlinear_roster[index];
            if (entry.separation.status != sct::NonlinearSeparationStatus::CertifiedSeparated)
                evidence[index] = InspectNonlinearPair(snapshot, entry);
        }
    };
    // Two workers total; completion order cannot affect source-ordered folding.
    std::exception_ptr worker_error, caller_error;
    std::thread worker([&] {
        try { inspect(); }
        catch (...) { worker_error = std::current_exception(); }
    });
    try { inspect(); }
    catch (...) { caller_error = std::current_exception(); }
    worker.join();
    if (worker_error) std::rethrow_exception(worker_error);
    if (caller_error) std::rethrow_exception(caller_error);
    std::array<std::size_t, static_cast<unsigned>(NonlinearAfterClass::Count)> classes{};
    std::uint64_t inspection_digest = 1469598103934665603ull;
    for (std::size_t index = 0; index < snapshot.nonlinear_roster.size(); ++index) {
        const auto& entry = snapshot.nonlinear_roster[index];
        if (entry.separation.status == sct::NonlinearSeparationStatus::CertifiedSeparated) continue;
        const auto& inspected = evidence[index];
        output::Require(inspected.valid, "Second-candidate nonlinear classification is incomplete");
        ++classes[static_cast<unsigned>(inspected.after)];
        HashKey(entry.key.paths[0], &inspection_digest);
        HashKey(entry.key.paths[1], &inspection_digest);
        fixture::HashUnsigned(static_cast<unsigned>(inspected.after), &inspection_digest);
        const bool remaining = inspected.after == NonlinearAfterClass::PossibleCurvedCrossing ||
            (inspected.local && inspected.ledger && inspected.endpoint_contact);
        fixture::HashUnsigned(remaining, &inspection_digest);
        if (remaining) {
            shards.Append("nonlinear", FreezeNonlinearPair(snapshot, entry));
            ++result.nonlinear_pairs;
        }
    }
    output::Require(std::accumulate(classes.begin(), classes.end(), std::size_t{0}) ==
                        snapshot.nonlinear_summary.unresolved,
                    "Second-candidate nonlinear classification partition is incomplete");
    shards.Flush();
    result.files = shards.files();
    result.bytes = shards.bytes();
    output::Integer(manifest, "linear_affine_pairs", snapshot.linear_summary.affine_pairs);
    output::Integer(manifest, "linear_swept_bounds_separated", snapshot.linear_summary.swept_bounds_separated);
    output::Integer(manifest, "linear_prism_separated", snapshot.linear_summary.prism_separated);
    output::Integer(manifest, "linear_exact_geometry_pairs", snapshot.linear_summary.exact_geometry_pairs);
    output::Integer(manifest, "linear_common_translation", snapshot.linear_summary.common_translation);
    output::Integer(manifest, "linear_residual_separated", snapshot.linear_summary.residual_separated);
    output::Integer(manifest, "linear_persistent_accepted", snapshot.linear_summary.persistent_accepted);
    output::Integer(manifest, "linear_represented_pairs", snapshot.linear_summary.represented_pairs);
    output::Integer(manifest, "linear_represented_separated", snapshot.linear_summary.represented_separated);
    output::Integer(manifest, "linear_represented_crossing", snapshot.linear_summary.represented_crossing);
    output::Integer(manifest, "linear_represented_degenerate", snapshot.linear_summary.represented_degenerate);
    output::Integer(manifest, "linear_work_exhausted_pairs", result.linear_pairs);
    output::Integer(manifest, "linear_represented_arithmetic_range", snapshot.linear_summary.represented_arithmetic_range);
    output::Integer(manifest, "linear_represented_work", snapshot.linear_summary.represented_work);
    output::Boolean(manifest, "linear_complete", snapshot.linear_summary.complete);
    output::Boolean(manifest, "linear_roster_complete", snapshot.linear_summary.roster_complete);
    output::String(manifest, "linear_crossing_status_scope", "successful complete census; no per-pair engine status in retained roster");
    output::Integer(manifest, "linear_fixture_crossing_classification", static_cast<unsigned>(contact::RepresentedIntervalClassification::Unresolved));
    output::Integer(manifest, "linear_fixture_crossing_reason", static_cast<unsigned>(contact::RepresentedIntervalReason::WorkExhausted));
    output::String(manifest, "linear_fixture_depth_scope", "baseline_depth zero means unknown; profile records configured crossing_depth");
    output::Integer(manifest, "nonlinear_pairs", snapshot.nonlinear_summary.nonlinear_pairs);
    output::Integer(manifest, "nonlinear_initial_separated", snapshot.nonlinear_summary.certified_separated);
    output::Integer(manifest, "nonlinear_initial_unresolved", snapshot.nonlinear_summary.unresolved);
    output::Integer(manifest, "nonlinear_exact_local", classes[0]);
    output::Integer(manifest, "nonlinear_residual_separated", classes[1]);
    output::Integer(manifest, "nonlinear_persistent_accepted", classes[2]);
    output::Integer(manifest, "nonlinear_remaining", classes[3]);
    output::Integer(manifest, "nonlinear_fixture_pairs", result.nonlinear_pairs);
    output::Integer(manifest, "inspection_digest", inspection_digest);
    output::Integer(manifest, "fixture_bytes", result.bytes);
    output::Require(result.bytes <= ArchiveCap - ManifestReserve,
                    "Diagnostic fixture archive lacks its manifest reserve");
    output::WriteJson(destination / "census.json", manifest);
    output::Require(std::filesystem::file_size(destination / "census.json") <= ManifestReserve,
                    "Diagnostic census manifest exceeds its fixed reserve");
    std::cout << "V5_SECOND_CENSUS_EXPORT linear=" << result.linear_pairs
              << " nonlinear=" << result.nonlinear_pairs << " files=" << result.files
              << " bytes=" << result.bytes << " census_digest=" << digest << std::endl;
    return result;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
