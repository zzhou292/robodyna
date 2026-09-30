#include "CoverageFixtureCapture.h"

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/FixedContactFacetValues.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace contact = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;
nonlinear_fixture::Pair FreezeNonlinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    const sct::NonlinearCandidateRosterEntry& entry) {
    nonlinear_fixture::Pair result;
    result.facet[0] = entry.facets.first;
    result.facet[1] = entry.facets.second;
    result.baseline_status = entry.separation.status;
    result.baseline_work = entry.separation.work;
    result.baseline_depth = entry.separation.deepest;
    const std::uint32_t facets[2]{
        entry.facets.first, entry.facets.second};
    for (unsigned side = 0; side < 2; ++side) {
        if (facets[side] >=
            snapshot.motion_certificates.facet_count)
            throw std::runtime_error(
                "Nonlinear fixture facet is out of range");
        result.accepted[side] =
            snapshot.motion_certificates.accepted_triangles[
                facets[side]];
        result.prepared[side] =
            snapshot.motion_certificates.prepared_triangles[
                facets[side]];
        result.quadratic[side] =
            snapshot.motion_certificates.quadratic[facets[side]];
        result.half_thickness[side] =
            snapshot.motion_certificates.descriptors[
                facets[side]].reference_half_thickness_m;
    }
    const std::array<contact::CurrentFixedTriangle, 2> accepted{
        result.accepted[0], result.accepted[1]};
    const std::array<contact::CurrentFixedTriangle, 2> prepared{
        result.prepared[0], result.prepared[1]};
    contact::FixedTriangleFeatureTaskMask accepted_mask;
    contact::FixedTriangleFeatureTaskMask prepared_mask;
    if (contact::BuildFixedTriangleFeatureTaskMask(
            accepted[0], accepted[1], &accepted_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok ||
        contact::BuildFixedTriangleFeatureTaskMask(
            prepared[0], prepared[1], &prepared_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(
            "Nonlinear fixture local mask failed");
    result.accepted_mask = accepted_mask.local_tasks;
    result.prepared_mask = prepared_mask.local_tasks;
    auto accepted_geometry =
        DiscoverDirect(accepted, accepted_mask);
    auto prepared_geometry =
        DiscoverDirect(prepared, prepared_mask);
    result.accepted_features =
        std::move(accepted_geometry.features);
    result.prepared_features =
        std::move(prepared_geometry.features);
    result.accepted_intersections =
        std::move(accepted_geometry.intersections);
    result.prepared_intersections =
        std::move(prepared_geometry.intersections);
    for (const auto& certificate :
         snapshot.accepted_certificates)
        if (snapshot.prepared_census
                ? CouldOwnFacetPairConservatively(prepared, certificate)
                : CouldOwnQuadraticPair(prepared, certificate))
            result.accepted_owners.push_back(certificate);
    if (result.accepted_owners.size() >
        nonlinear_fixture::MaximumOwners)
        throw std::runtime_error(
            "Nonlinear fixture owner roster exceeds hard cap");
    return result;
}

linear_fixture::Pair FreezeLinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    const sct::LinearWorkExhaustedRosterEntry& entry) {
    linear_fixture::Pair result;
    result.facet[0] = entry.facets.first;
    result.facet[1] = entry.facets.second;
    result.baseline_status =
        sct::NonlinearSeparationStatus::WorkExhausted;
    result.baseline_work = entry.crossing.work;
    // The linear report does not expose the depth reached. Preserve the
    // historical fixture value, but use zero (unknown) for new captures.
    result.baseline_depth = snapshot.prepared_census ? 0 : 20;
    const std::uint32_t facets[2]{
        entry.facets.first, entry.facets.second};
    for (unsigned side = 0; side < 2; ++side) {
        if (facets[side] >=
            snapshot.motion_certificates.facet_count)
            throw std::runtime_error(
                "Linear fixture facet is out of range");
        result.accepted[side] =
            snapshot.motion_certificates.accepted_triangles[
                facets[side]];
        result.prepared[side] =
            snapshot.motion_certificates.prepared_triangles[
                facets[side]];
        result.quadratic[side] =
            snapshot.motion_certificates.quadratic[facets[side]];
        result.half_thickness[side] =
            snapshot.motion_certificates.descriptors[
                facets[side]].reference_half_thickness_m;
    }
    contact::FixedTriangleFeatureTaskMask accepted_mask;
    contact::FixedTriangleFeatureTaskMask prepared_mask;
    if (contact::BuildFixedTriangleFeatureTaskMask(
            result.accepted[0], result.accepted[1],
            &accepted_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok ||
        contact::BuildFixedTriangleFeatureTaskMask(
            result.prepared[0], result.prepared[1],
            &prepared_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(
            "Linear fixture local mask failed");
    result.accepted_mask = accepted_mask.local_tasks;
    result.prepared_mask = prepared_mask.local_tasks;
    const std::array<contact::CurrentFixedTriangle, 2> accepted{
        result.accepted[0], result.accepted[1]};
    const std::array<contact::CurrentFixedTriangle, 2> prepared{
        result.prepared[0], result.prepared[1]};
    auto accepted_geometry =
        DiscoverDirect(accepted, accepted_mask);
    auto prepared_geometry =
        DiscoverDirect(prepared, prepared_mask);
    result.accepted_features =
        std::move(accepted_geometry.features);
    result.prepared_features =
        std::move(prepared_geometry.features);
    result.accepted_intersections =
        std::move(accepted_geometry.intersections);
    result.prepared_intersections =
        std::move(prepared_geometry.intersections);
    for (const auto& certificate :
         snapshot.accepted_certificates)
        if (snapshot.prepared_census
                ? CouldOwnFacetPairConservatively(prepared, certificate)
                : CouldOwnQuadraticPair(prepared, certificate))
            result.accepted_owners.push_back(certificate);
    if (result.accepted_owners.size() >
        nonlinear_fixture::MaximumOwners)
        throw std::runtime_error(
            "Linear fixture owner roster exceeds hard cap");
    return result;
}

void FreezeAcceptedPolicies(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    std::vector<nonlinear_fixture::Pair>* pairs) {
    if (!pairs || !snapshot.transaction ||
        !snapshot.accepted_receipt)
        throw std::runtime_error(
            "Nonlinear fixture accepted policy owner is absent");
    std::vector<contact::FixedTriangleFeatureCandidate> features;
    std::vector<std::size_t> offsets;
    offsets.reserve(pairs->size() + 1);
    offsets.push_back(0);
    for (const auto& pair : *pairs) {
        features.insert(
            features.end(), pair.accepted_features.begin(),
            pair.accepted_features.end());
        offsets.push_back(features.size());
    }
    std::vector<sct::AcceptedFeaturePolicyEvidence>
        policy(features.size());
    std::size_t policy_count = 0;
    const auto report = snapshot.prepared_census ?
        sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
            *snapshot.transaction, snapshot.prepared_census_receipt,
            {features.data(), features.size(), true},
            policy.data(), policy.size(), &policy_count) :
        sct::QualificationAccess::
            ClassifyAcceptedFeaturePolicies(
                *snapshot.transaction,
                *snapshot.accepted_receipt,
                {features.data(), features.size(), true},
                policy.data(), policy.size(), &policy_count);
    if (report.status !=
            contact::SelfContactTransactionStatus::Ok ||
        policy_count != policy.size())
        throw std::runtime_error(report.message);
    for (std::size_t pair = 0; pair < pairs->size(); ++pair)
        (*pairs)[pair].accepted_policy.assign(
            policy.begin() + offsets[pair],
            policy.begin() + offsets[pair + 1]);
}

std::vector<sct::AcceptedFeatureExclusionCertificate>
FrozenSameRigidExclusions(
    const nonlinear_fixture::Pair& pair) {
    std::vector<sct::AcceptedFeatureExclusionCertificate> result;
    for (std::size_t feature = 0;
         feature < pair.accepted_features.size(); ++feature) {
        const auto& evidence = pair.accepted_policy[feature];
        if (evidence.pair_status !=
                contact::SelfContactPairStatus::
                    ExcludedSameRigidGroup ||
            evidence.endpoint_support[0].status !=
                contact::SelfContactSupportStatus::
                    CompleteRigidGroup ||
            evidence.endpoint_support[1].status !=
                contact::SelfContactSupportStatus::
                    CompleteRigidGroup ||
            evidence.endpoint_support[0].complete_rigid_group !=
                evidence.endpoint_support[1].
                    complete_rigid_group)
            continue;
        result.push_back({
            pair.accepted_features[feature],
            static_cast<std::uint32_t>(
                evidence.endpoint_support[0].
                    complete_rigid_group)});
    }
    return result;
}

nonlinear_fixture::PhaseIdentity FixturePhaseIdentity(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot) {
    const auto bits = [](double value) {
        std::uint64_t result = 0;
        std::memcpy(&result, &value, sizeof(result));
        return result;
    };
    nonlinear_fixture::PhaseIdentity result;
    result.accepted_epoch = snapshot.accepted_stamp.epoch;
    result.prepared_base_epoch =
        snapshot.prepared_view.kinematics.base_epoch;
    result.accepted_time_bits =
        bits(snapshot.accepted_stamp.time);
    result.prepared_base_time_bits =
        bits(snapshot.prepared_view.base_time);
    result.prepared_time_bits =
        bits(snapshot.prepared_view.proposed_time);
    result.accepted_velocity_time_bits =
        bits(snapshot.accepted_stamp.velocity_time);
    result.prepared_velocity_time_bits =
        bits(snapshot.prepared_view.velocity_time);
    result.accepted_temporal_scheme =
        static_cast<unsigned>(
            snapshot.accepted_stamp.temporal_scheme);
    result.prepared_temporal_scheme =
        static_cast<unsigned>(
            snapshot.prepared_view.temporal_scheme);
    result.accepted_velocity_phase =
        static_cast<unsigned>(
            snapshot.accepted_stamp.velocity_phase);
    result.prepared_velocity_phase =
        static_cast<unsigned>(
            snapshot.prepared_view.velocity_phase);
    result.prepared_trajectory =
        static_cast<unsigned>(
            snapshot.prepared_view.rigid_member_trajectory);
    return result;
}


}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
