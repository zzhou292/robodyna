#include "PreparedCensusReplayManifest.h"
#include "CoverageFixtureCapture.h"

#include <tuple>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay {
namespace {
namespace contact = tlfea::contact;
namespace sct = contact::self_contact_transaction;
using Status = sct::NonlinearSeparationStatus;
using output::Require;
auto Key(const contact::FixedTriangleKey& k) {
    return std::make_tuple(k.source_instance_id, k.parent_eid, k.level, k.local_facet);
}
auto Phase(const nonlinear_fixture::PhaseIdentity& p) {
    return std::make_tuple(p.accepted_label, p.prepared_label, p.accepted_epoch, p.prepared_base_epoch,
        p.accepted_time_bits, p.prepared_base_time_bits, p.prepared_time_bits, p.accepted_velocity_time_bits,
        p.prepared_velocity_time_bits, p.accepted_temporal_scheme, p.prepared_temporal_scheme,
        p.accepted_velocity_phase, p.prepared_velocity_phase, p.prepared_trajectory);
}
void HashResult(const sct::NonlinearSeparationResult& r, std::uint64_t* digest) {
    nonlinear_fixture::HashUnsigned(static_cast<unsigned>(r.status), digest);
    nonlinear_fixture::HashUnsigned(r.work, digest);
    nonlinear_fixture::HashUnsigned(r.deepest, digest);
    nonlinear_fixture::HashUnsigned(r.proof_digest, digest);
    nonlinear_fixture::HashUnsigned(r.work_exhausted, digest);
    nonlinear_fixture::HashUnsigned(r.depth_exhausted, digest);
}
PairResult Evaluate(const nonlinear_fixture::Pair& pair, const detail::Manifest& manifest) {
    PairResult result;
    result.affine = true;
    for (unsigned side = 0; side < 2; ++side) {
        Require(Key(pair.accepted[side].key) == Key(pair.prepared[side].key) &&
            pair.prepared[side].key.source_instance_id && pair.prepared[side].key.parent_eid &&
            pair.quadratic[side].complete, "Prepared census pair source or trajectory is incomplete");
        result.facets[side] = pair.prepared[side].key;
        for (const auto& vertex : pair.quadratic[side].q)
            for (const auto& q : vertex) result.affine &= q.lower == 0 && q.upper == 0;
    }
    Require(Key(result.facets[0]) < Key(result.facets[1]), "Prepared census pair is not in canonical order");
    result.baseline_status = pair.baseline_status; result.baseline_work = pair.baseline_work;
    result.baseline_depth = pair.baseline_depth; result.owners = pair.accepted_owners.size();
    const auto exclusions = SameRigidExclusions(pair);
    result.exclusions = exclusions.size();
    // Preserve the original ledger result: the policy wrapper can replace its
    // failure with an exclusion result. Both identities matter diagnostically.
    result.policy = sct::CertifyQuadraticFacetPolicyCoverage(
        pair.accepted[0], pair.prepared[0], pair.quadratic[0], pair.half_thickness[0],
        pair.accepted[1], pair.prepared[1], pair.quadratic[1], pair.half_thickness[1], manifest.duration,
        pair.accepted_owners.data(), pair.accepted_owners.size(), exclusions.data(), exclusions.size(),
        manifest.work, manifest.depth);
    result.ledger = exclusions.empty() ? result.policy : sct::CertifyQuadraticFacetCoverage(
        pair.accepted[0], pair.prepared[0], pair.quadratic[0], pair.half_thickness[0],
        pair.accepted[1], pair.prepared[1], pair.quadratic[1], pair.half_thickness[1], manifest.duration,
        pair.accepted_owners.data(), pair.accepted_owners.size(), manifest.work, manifest.depth);
    return result;
}
}

bool Certified(Status status) noexcept {
    return status == Status::CertifiedSeparated || status == Status::CertifiedAcceptedCoverage ||
        status == Status::CertifiedExactExclusion;
}
const char* StatusName(Status status) noexcept {
    switch (status) {
        case Status::CertifiedSeparated: return "certified_separated";
        case Status::PotentialContact: return "potential_contact";
        case Status::WorkExhausted: return "work_exhausted";
        case Status::DepthExhausted: return "depth_exhausted";
        case Status::InvalidInput: return "invalid_input";
        case Status::CertifiedAcceptedCoverage: return "certified_accepted_coverage";
        case Status::MissingAcceptedOwner: return "missing_accepted_owner";
        case Status::OwnerAmbiguity: return "owner_ambiguity";
        case Status::PossibleGeometricCrossing: return "possible_geometric_crossing";
        case Status::CertifiedExactExclusion: return "certified_exact_exclusion";
    }
    return "unknown";
}
std::vector<sct::AcceptedFeatureExclusionCertificate> SameRigidExclusions(const nonlinear_fixture::Pair& pair) {
    Require(pair.accepted_features.size() == pair.accepted_policy.size(), "Prepared census accepted policy is incomplete");
    for (std::size_t i = 0; i < pair.accepted_features.size(); ++i) {
        const auto& evidence = pair.accepted_policy[i];
        if (evidence.pair_status != contact::SelfContactPairStatus::ExcludedSameRigidGroup) continue;
        const auto& a = evidence.endpoint_support[0]; const auto& b = evidence.endpoint_support[1];
        Require(evidence.report_status == contact::SelfContactTransactionStatus::Ok &&
            a.status == contact::SelfContactSupportStatus::CompleteRigidGroup &&
            b.status == contact::SelfContactSupportStatus::CompleteRigidGroup &&
            a.complete_rigid_group == b.complete_rigid_group && a.complete_rigid_group < UINT32_MAX,
            "Prepared census same-rigid exclusion lacks matching authenticated support");
    }
    return FrozenSameRigidExclusions(pair);
}

Summary Replay(const std::filesystem::path& path, const std::string& expected_sha256,
               const std::function<void(const PairResult&)>& observe) {
    const auto manifest = detail::ReadManifest(path, expected_sha256);
    Summary summary; summary.bytes = manifest.bytes;
    nonlinear_fixture::PhaseIdentity phase;
    contact::FixedTriangleKey previous[2];
    bool have_phase = false, have_previous = false;
    std::string family;
    for (const auto& shard : manifest.shards) {
        const auto data = detail::ReadShard(path.parent_path(), shard, manifest);
        if (have_phase) Require(Phase(phase) == Phase(data.phase), "Prepared census shard phases differ");
        else { phase = data.phase; have_phase = true; }
        if (family != shard.family) { family = shard.family; have_previous = false; }
        for (std::size_t i = 0; i < data.pairs.size(); ++i) {
            auto result = Evaluate(data.pairs[i], manifest);
            if (have_previous)
                Require(std::make_tuple(Key(previous[0]), Key(previous[1])) <
                    std::make_tuple(Key(result.facets[0]), Key(result.facets[1])),
                    "Prepared census family repeats or reorders a source pair");
            previous[0] = result.facets[0]; previous[1] = result.facets[1]; have_previous = true;
            result.file = shard.file; result.family = shard.family; result.ordinal = i;
            const auto status = static_cast<unsigned>(result.policy.status);
            Require(status < summary.status_counts.size(), "Prepared replay result status is unknown");
            ++summary.status_counts[status]; ++summary.pairs;
            if (shard.family == "linear") ++summary.linear; else ++summary.nonlinear;
            summary.noncertified += !Certified(result.policy.status);
            Require(result.policy.work <= SIZE_MAX - summary.total_work, "Prepared replay work accounting overflows");
            summary.total_work += result.policy.work;
            nonlinear_fixture::HashPath(result.facets[0], &summary.result_digest);
            nonlinear_fixture::HashPath(result.facets[1], &summary.result_digest);
            HashResult(result.ledger, &summary.result_digest); HashResult(result.policy, &summary.result_digest);
            if (observe) observe(result);
        }
        ++summary.files; summary.bytes += shard.bytes;
    }
    Require(summary.linear == manifest.linear && summary.nonlinear == manifest.nonlinear,
        "Prepared replay did not visit every declared pair");
    summary.complete = true;
    return summary;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay
