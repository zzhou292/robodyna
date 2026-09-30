#include "PreparedCensusReplayManifest.h"
#include "output/BoundedArrayJson.h"

#include <algorithm>
#include <set>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay::detail {
namespace {
namespace json = output::array_json;
namespace fixture = nonlinear_fixture;
using output::Require;

bool HashText(const std::string& value) {
    return value.size() == 64 && std::all_of(value.begin(), value.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}
std::size_t Add(std::size_t a, std::uint64_t b) {
    Require(b <= SIZE_MAX - a, "Prepared census count overflows");
    return a + static_cast<std::size_t>(b);
}
std::uint64_t HashProfile(const output::Value& profile) {
    std::uint64_t hash = 1469598103934665603ull;
    for (auto it = profile.MemberBegin(); it != profile.MemberEnd(); ++it) {
        fixture::HashBytes(it->name.GetString(), it->name.GetStringLength(), &hash);
        if (it->value.IsString())
            fixture::HashBytes(it->value.GetString(), it->value.GetStringLength(), &hash);
        else if (it->value.IsUint64()) fixture::HashUnsigned(it->value.GetUint64(), &hash);
        else fixture::HashUnsigned(output::Bits(json::Real(it->value)), &hash);
    }
    return hash;
}
}

Manifest ReadManifest(const std::filesystem::path& path, const std::string& expected_sha256) {
    Require(HashText(expected_sha256), "Prepared census needs an explicit SHA-256 authority");
    const auto bytes = output::ReadBounded(path, ManifestByteCap);
    Require(output::Sha256(bytes) == expected_sha256, "Prepared census manifest SHA-256 differs");
    const auto d = json::Parse(bytes, ManifestByteCap);
    json::Keys(d, {"schema", "scope", "second_interval_committed", "actual_prepared_activity", "profile",
        "owner_id", "accepted_epoch", "prepared_attempt", "accepted_time_s", "prepared_time_s",
        "prepared_base_time_s", "prepared_kick_dt_s", "profile_hash", "dt_hash", "diagnostic_host_upper_bound", "archive_cap_bytes",
        "complete_census_digest", "selected_parents", "accepted_active_parents", "prepared_active_parents",
        "removing_parents", "inactive_parents", "files", "linear_affine_pairs", "linear_swept_bounds_separated",
        "linear_prism_separated", "linear_exact_geometry_pairs", "linear_common_translation",
        "linear_residual_separated", "linear_persistent_accepted", "linear_represented_pairs",
        "linear_represented_separated", "linear_represented_crossing", "linear_represented_degenerate",
        "linear_work_exhausted_pairs", "linear_represented_arithmetic_range", "linear_represented_work",
        "linear_complete", "linear_roster_complete", "linear_crossing_status_scope",
        "linear_fixture_crossing_classification", "linear_fixture_crossing_reason", "linear_fixture_depth_scope",
        "nonlinear_pairs", "nonlinear_initial_separated", "nonlinear_initial_unresolved", "nonlinear_exact_local", "nonlinear_exact_local_scope",
        "nonlinear_residual_separated", "nonlinear_persistent_accepted", "nonlinear_remaining",
        "nonlinear_fixture_pairs", "inspection_digest", "fixture_bytes"});
    Require(json::Text(d["schema"]) == "robo_dyna.prepared_self_contact_census.v1" &&
        d["second_interval_committed"].IsBool() && !d["second_interval_committed"].GetBool() &&
        d["actual_prepared_activity"].IsBool() && d["actual_prepared_activity"].GetBool(),
        "Prepared census scope or activity authority differs");
    const auto& p = d["profile"];
    json::Keys(p, {"physical_profile", "contact_profile", "scope", "leading_gap_m", "initial_speed_mps",
        "declared_duration_s", "physical_step_s", "source_instance_id", "configuration_id", "qualification_id",
        "self_contact_source_id", "event_capacity", "parent_pair_capacity", "facet_pair_capacity", "facet_chunk",
        "crossing_work_per_pair", "crossing_depth", "discovery_workers", "crossing_workers"});
    Manifest m;
    m.bytes = bytes.size(); m.profile_hash = json::UInt(d["profile_hash"]);
    m.dt_hash = json::UInt(d["dt_hash"]); m.census_digest = json::UInt(d["complete_census_digest"]);
    m.epoch = json::UInt(d["accepted_epoch"]);
    m.accepted_time = json::Real(d["accepted_time_s"]);
    m.prepared_time = json::Real(d["prepared_time_s"]);
    const auto base_time = json::Real(d["prepared_base_time_s"]);
    Require(output::Bits(base_time) == output::Bits(m.accepted_time), "Prepared census base endpoint differs");
    m.duration = m.prepared_time - base_time;
    m.kick_dt = json::Real(d["prepared_kick_dt_s"]);
    Require(m.epoch && json::UInt(d["owner_id"]) && json::UInt(d["prepared_attempt"]) &&
        m.accepted_time >= 0 && std::isfinite(m.duration) && m.duration > 0 && m.kick_dt > 0 &&
        json::Real(p["physical_step_s"]) > 0 && HashProfile(p) == m.profile_hash,
        "Prepared census time or profile authentication differs");
    const auto work = json::UInt(p["crossing_work_per_pair"]), depth = json::UInt(p["crossing_depth"]);
    Require(work && work <= (1u << 20) && depth <= 52, "Prepared replay proof budget exceeds hard bounds");
    m.work = work; m.depth = static_cast<unsigned>(depth);
    Require(json::UInt(d["archive_cap_bytes"]) == ArchiveByteCap, "Prepared census archive cap differs");
    const auto active = json::UInt(d["prepared_active_parents"]), removing = json::UInt(d["removing_parents"]);
    Require(Add(active, removing) == json::UInt(d["accepted_active_parents"]) &&
        Add(Add(active, removing), json::UInt(d["inactive_parents"])) == json::UInt(d["selected_parents"]),
        "Prepared census activity partition differs");
    const auto unresolved = json::UInt(d["nonlinear_initial_unresolved"]);
    Require(Add(json::UInt(d["nonlinear_initial_separated"]), unresolved) == json::UInt(d["nonlinear_pairs"]) &&
        Add(Add(json::UInt(d["nonlinear_exact_local"]), json::UInt(d["nonlinear_residual_separated"])),
            Add(json::UInt(d["nonlinear_persistent_accepted"]), json::UInt(d["nonlinear_remaining"]))) == unresolved,
        "Prepared census nonlinear partition differs");
    m.linear = json::UInt(d["linear_work_exhausted_pairs"]);
    m.nonlinear = json::UInt(d["nonlinear_fixture_pairs"]);
    const auto represented = json::UInt(d["linear_represented_pairs"]);
    Require(d["linear_complete"].IsBool() && d["linear_complete"].GetBool() &&
        d["linear_roster_complete"].IsBool() && d["linear_roster_complete"].GetBool() &&
        json::UInt(d["linear_fixture_crossing_classification"]) ==
            static_cast<unsigned>(tlfea::contact::RepresentedIntervalClassification::Unresolved) &&
        json::UInt(d["linear_fixture_crossing_reason"]) ==
            static_cast<unsigned>(tlfea::contact::RepresentedIntervalReason::WorkExhausted) &&
        Add(Add(json::UInt(d["linear_swept_bounds_separated"]), json::UInt(d["linear_prism_separated"])),
            json::UInt(d["linear_exact_geometry_pairs"])) == json::UInt(d["linear_affine_pairs"]) &&
        Add(Add(json::UInt(d["linear_residual_separated"]), json::UInt(d["linear_persistent_accepted"])),
            represented) == json::UInt(d["linear_exact_geometry_pairs"]) &&
        Add(Add(Add(json::UInt(d["linear_represented_separated"]), json::UInt(d["linear_represented_crossing"])),
            Add(json::UInt(d["linear_represented_degenerate"]), m.linear)),
            json::UInt(d["linear_represented_arithmetic_range"])) == represented,
        "Prepared census affine partition is incomplete");
    Require(json::Text(d["nonlinear_exact_local_scope"]) ==
        "endpoint-only descriptive classification; all such rows retained for continuous replay" &&
        m.linear <= json::UInt(d["linear_affine_pairs"]) && m.nonlinear <= unresolved &&
        m.nonlinear >= Add(json::UInt(d["nonlinear_exact_local"]), json::UInt(d["nonlinear_remaining"])),
        "Prepared census fixture coverage differs");
    const auto residual = json::UInt(d["nonlinear_residual_separated"]);
    Require(residual <= unresolved && m.nonlinear <= unresolved - residual,
        "Prepared census frozen and separated nonlinear partitions overlap");
    m.omitted_nonlinear_persistent = unresolved - residual - m.nonlinear;
    Require(m.omitted_nonlinear_persistent <= json::UInt(d["nonlinear_persistent_accepted"]),
        "Prepared census omits an unproved nonlinear class");
    m.omitted_linear_persistent = json::UInt(d["linear_persistent_accepted"]);
    const auto& files = d["files"];
    Require(files.IsArray() && files.Size() <= 8192, "Prepared census file roster exceeds metadata cap");
    std::set<std::string> seen;
    std::size_t linear = 0, nonlinear = 0;
    bool nonlinear_started = false;
    for (const auto& value : files.GetArray()) {
        json::Keys(value, {"file", "sha256", "family", "bytes", "pairs", "source_hash", "payload_hash", "roster_digest"});
        Shard s;
        s.file = json::Text(value["file"]); s.sha256 = json::Text(value["sha256"]); s.family = json::Text(value["family"]);
        s.bytes = json::UInt(value["bytes"]); s.pairs = json::UInt(value["pairs"]);
        s.source_hash = json::UInt(value["source_hash"]); s.payload_hash = json::UInt(value["payload_hash"]);
        s.roster_digest = json::UInt(value["roster_digest"]);
        const std::filesystem::path relative(s.file);
        Require(!s.file.empty() && s.file.find('\0') == std::string::npos && relative == relative.filename() &&
            s.file != "." && s.file != ".." && seen.insert(s.file).second && HashText(s.sha256) &&
            s.bytes >= 208 && s.bytes <= ShardByteCap && s.pairs && s.pairs <= PairsPerShard,
            "Prepared census shard path, hash or shape is invalid");
        Require(s.family == "linear" || s.family == "nonlinear", "Prepared census shard family is invalid");
        if (s.family == "linear") {
            Require(!nonlinear_started, "Prepared census shard family order differs"); linear = Add(linear, s.pairs);
        } else { nonlinear_started = true; nonlinear = Add(nonlinear, s.pairs); }
        m.fixture_bytes = Add(m.fixture_bytes, s.bytes);
        Require(m.fixture_bytes <= ArchiveByteCap - m.bytes, "Prepared census archive exceeds 2 GiB");
        m.shards.push_back(std::move(s));
    }
    Require(linear == m.linear && nonlinear == m.nonlinear && m.fixture_bytes == json::UInt(d["fixture_bytes"]),
        "Prepared census shard inventory does not cover the declared roster");
    return m;
}

nonlinear_fixture::File ReadShard(const std::filesystem::path& root, const Shard& s, const Manifest& m) {
    const auto path = root / s.file;
    Require(std::filesystem::is_regular_file(std::filesystem::symlink_status(path)),
        "Prepared census shard must be a regular owned file");
    // Validate the untrusted header count before the legacy codec reserves its
    // pair vector. Release these bytes before the codec reads and decodes.
    {
        const auto bytes = output::ReadBounded(path, ShardByteCap);
        Require(bytes.size() == s.bytes && output::Sha256(bytes) == s.sha256, "Prepared census shard SHA-256 differs");
        fixture::Reader r(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
        for (unsigned field = 0; field < 9; ++field) r.U64();
        Require(r.U64() == s.pairs, "Prepared census shard header count differs");
    }
    auto file = fixture::Read(path.string(), false);
    const auto& phase = file.phase;
    std::uint64_t dt_hash = 1469598103934665603ull;
    fixture::HashUnsigned(output::Bits(m.duration), &dt_hash);
    fixture::HashUnsigned(output::Bits(m.kick_dt), &dt_hash);
    fixture::HashUnsigned(phase.prepared_trajectory, &dt_hash);
    Require(file.pairs.size() == s.pairs && file.source_hash == s.source_hash && file.payload_hash == s.payload_hash &&
        file.roster_digest == s.roster_digest && file.profile_hash == m.profile_hash && file.dt_hash == m.dt_hash &&
        file.nonlinear_roster_digest == m.census_digest && dt_hash == m.dt_hash &&
        phase.accepted_epoch == m.epoch && phase.prepared_base_epoch == m.epoch &&
        phase.accepted_time_bits == output::Bits(m.accepted_time) &&
        phase.prepared_time_bits == output::Bits(m.prepared_time), "Prepared census shard source or phase differs");
    return file;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay::detail
