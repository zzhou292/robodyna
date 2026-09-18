#include "CandidateFailureFixture.h"

#include "CandidateFailureValues.h"
#include "CoverageFixtureCapture.h"
#include "lib_src/collision/self_contact_transaction/ResidualTasks.h"
#include "lib_utils/BoundedArena.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {

template <std::size_t N>
void Message(std::array<char, N>& destination, const char* source) noexcept {
    if (!source)
        source = "No diagnostic message";
    const auto length = std::min(std::strlen(source), N - 1);
    std::memcpy(destination.data(), source, length);
    destination[length] = '\0';
}

}  // namespace

CandidateFailureFixture::CandidateFailureFixture(
    const contact::SelfContactTransactionLimits& limits)
    : work_(limits.crossing.max_work_per_pair), depth_(limits.crossing.max_depth),
      nonlinear_work_(limits.max_nonlinear_subdivision_work_per_pair),
      nonlinear_depth_(limits.max_nonlinear_subdivision_depth) {
    output::Require(work_ && work_ <= (1u << 20) && depth_ <= 52 &&
                        nonlinear_work_ && nonlinear_work_ <= (1u << 20) &&
                        nonlinear_depth_ <= 52,
                    "Failure replay proof limits exceed bounded diagnostic profile");
    // Account for retained state, temporary owner copies, native feature/policy
    // buffers, codec copies and JSON capacity before any simulation allocation.
    // Native proof stack and allocator overhead remain governed by the outer
    // runtime RSS guard; this budget covers capture/codec/JSON storage only.
    tl::util::BoundedArenaLayout budget(FailureCaptureHostCap);
    tl::util::ArenaRegion region;
    output::Require(
        budget.Append<CandidateFailureFixture>(1, region) &&
        budget.Append<sct::AcceptedEventCertificate>(4 * nonlinear_fixture::MaximumOwners, region) &&
        budget.Append<contact::FixedTriangleFeatureCandidate>(6 * nonlinear_fixture::MaximumFeatures, region) &&
        budget.Append<sct::AcceptedFeaturePolicyEvidence>(4 * nonlinear_fixture::MaximumFeatures, region) &&
        budget.Append<std::byte>(3 * FailureFixtureArchiveCap, region) &&
        budget.Append<std::byte>(4 * FailureFixtureManifestCap, region),
        "Failure capture exceeds its fixed extra host allowance");
    pairs_.reserve(1);
}

sct::CandidateFailureObserver CandidateFailureFixture::observer() noexcept {
    return {this, sizeof(*this), &Capture};
}

const nonlinear_fixture::Pair& CandidateFailureFixture::pair() const {
    output::Require(complete_ && pairs_.size() == 1, "Failure fixture is incomplete");
    return pairs_.front();
}

void CandidateFailureFixture::Capture(
    void* context, const sct::CandidateFailureCapture& input) noexcept {
    auto& state = *static_cast<CandidateFailureFixture*>(context);
    if (state.seen_)
        return;  // Preserve the first rejection, even if a caller retries.
    state.seen_ = true;
    state.report_ = input.report;
    Message(state.message_, input.report.message);
    state.report_.message = state.message_.data();
    try {
        output::Require(input.report.message &&
                            std::strlen(input.report.message) < state.message_.size(),
                        "Native rejection message exceeds exact capture capacity");
        state.Freeze(input);
        state.complete_ = true;
    } catch (const std::exception& error) {
        Message(state.error_, error.what());
        state.exception_ = std::current_exception();
    } catch (...) {
        Message(state.error_, "Unknown exception while freezing native rejection");
        state.exception_ = std::current_exception();
    }
}

void CandidateFailureFixture::Freeze(const sct::CandidateFailureCapture& input) {
    output::Require(input.report.status != contact::SelfContactTransactionStatus::Ok &&
                        input.transaction && input.accepted_assembly && input.activity.valid() &&
                        input.activity.activity_summary().complete && input.motion.complete &&
                        input.motion.descriptors && input.motion.quadratic &&
                        input.motion.accepted_triangles && input.motion.prepared_triangles &&
                        input.accepted_events.complete &&
                        (!input.accepted_events.count || input.accepted_events.data),
                    "Failure observer did not supply complete authenticated evidence");
    output::Require(input.facets.first < input.motion.facet_count &&
                        input.facets.second < input.motion.facet_count &&
                        input.accepted.epoch == input.prepared.kinematics.base_epoch &&
                        output::Bits(input.accepted.time) == output::Bits(input.prepared.base_time),
                    "Failure geometry has a stale phase or invalid inventory index");
    duration_ = input.prepared.proposed_time - input.prepared.base_time;
    kick_dt_ = input.prepared.kick_dt;
    output::Require(std::isfinite(duration_) && duration_ > 0 &&
                        std::isfinite(kick_dt_) && kick_dt_ > 0,
                    "Failure interval must have finite positive duration");
    owner_ = input.prepared.owner_id;
    attempt_ = input.prepared.attempt;
    activity_ = input.activity.activity_summary();

    vehicle_self_contact::AcceptedAssemblyCouponSnapshot snapshot;
    snapshot.prepared_census = true;
    snapshot.transaction = input.transaction;
    snapshot.accepted_receipt = input.accepted_assembly;
    snapshot.prepared_census_receipt = input.activity;
    snapshot.accepted_stamp = input.accepted;
    snapshot.prepared_view = input.prepared;
    snapshot.motion_certificates = input.motion;
    snapshot.accepted_certificates.reserve(nonlinear_fixture::MaximumOwners);
    const std::array<contact::CurrentFixedTriangle, 2> prepared{
        input.motion.prepared_triangles[input.facets.first],
        input.motion.prepared_triangles[input.facets.second]};
    for (std::size_t i = 0; i < input.accepted_events.count; ++i) {
        const auto& owner = input.accepted_events.data[i];
        if (!CouldOwnFacetPairConservatively(prepared, owner))
            continue;
        output::Require(snapshot.accepted_certificates.size() < nonlinear_fixture::MaximumOwners,
                        "Relevant accepted owners exceed failure fixture cap; no truncation");
        snapshot.accepted_certificates.push_back(owner);
    }
    sct::NonlinearCandidateRosterEntry entry;
    entry.facets = input.facets;
    // Native report has a crossing reason, not the final nonlinear result.
    // Preserve that distinction; baseline InvalidInput means not reported.
    pairs_.push_back(FreezeNonlinearPair(snapshot, entry));
    FreezeAcceptedPolicies(snapshot, &pairs_);
    phase_ = FixturePhaseIdentity(snapshot);
    const auto& frozen = pairs_.front();
    bool affine = true;
    for (const auto& coefficients : frozen.quadratic)
        for (const auto& vertex : coefficients.q)
            for (const auto& component : vertex)
                affine = affine && component.lower == 0 && component.upper == 0;
    const auto replay_work = affine ? work_ : nonlinear_work_;
    const auto replay_depth = affine ? depth_ : nonlinear_depth_;
    compact_ = failure_detail::Evaluate(frozen, duration_, replay_work, replay_depth,
                                      frozen.accepted_owners.data(), frozen.accepted_owners.size());
    full_owners_ = input.accepted_events.count;
    full_ = failure_detail::Evaluate(frozen, duration_, replay_work, replay_depth,
                                   input.accepted_events.data, input.accepted_events.count);
    owners_equivalent_ = failure_detail::Equivalent(full_, compact_);
    output::Require(owners_equivalent_, "Compact owner replay differs from authenticated full ledger");
    residual_ = sct::CertifyQuadraticUnmaskedSeparation(
        frozen.accepted[0], frozen.prepared[0], frozen.quadratic[0], frozen.half_thickness[0],
        frozen.accepted[1], frozen.prepared[1], frozen.quadratic[1], frozen.half_thickness[1],
        duration_, {frozen.prepared_features.data(), frozen.prepared_features.size(), true},
        contact::FixedTriangleFeatureTaskMask{frozen.prepared_mask});
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
