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
bool RequiresFrozenNonlinearReplay(const NonlinearPairEvidence& evidence) noexcept {
    if (!evidence.valid) return true;
    if (evidence.after == NonlinearAfterClass::CertifiedQuadraticResidualSeparation &&
        evidence.residual_status == sct::LinearResidualSeparationStatus::CertifiedSeparated)
        return false;
    if (evidence.after == NonlinearAfterClass::CertifiedPersistentAcceptedContact &&
        evidence.persistent_status == sct::PersistentLinearContactStatus::CertifiedContact &&
        evidence.ledger && evidence.endpoint_contact && !evidence.local)
        return false;
    return true;
}

NonlinearPairEvidence InspectNonlinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    const sct::NonlinearCandidateRosterEntry& entry) {
    NonlinearPairEvidence result;
    if (entry.facets.first >=
            snapshot.motion_certificates.facet_count ||
        entry.facets.second >=
            snapshot.motion_certificates.facet_count)
        return result;
    const std::array<contact::CurrentFixedTriangle, 2> accepted{
        snapshot.motion_certificates.accepted_triangles[
            entry.facets.first],
        snapshot.motion_certificates.accepted_triangles[
            entry.facets.second]};
    const std::array<contact::CurrentFixedTriangle, 2> prepared{
        snapshot.motion_certificates.prepared_triangles[
            entry.facets.first],
        snapshot.motion_certificates.prepared_triangles[
            entry.facets.second]};
    for (unsigned side = 0; side < 2; ++side)
        if (contact::fixed_triangle_features::Compare(
                accepted[side].key, prepared[side].key) != 0)
            return result;
    const auto& first_descriptor =
        snapshot.motion_certificates.descriptors[
            entry.facets.first];
    const auto& second_descriptor =
        snapshot.motion_certificates.descriptors[
            entry.facets.second];
    result.thickness_m =
        first_descriptor.reference_half_thickness_m +
        second_descriptor.reference_half_thickness_m;
    contact::FixedTriangleFeatureTaskMask accepted_mask;
    contact::FixedTriangleFeatureTaskMask prepared_mask;
    if (contact::BuildFixedTriangleFeatureTaskMask(
            accepted[0], accepted[1], &accepted_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok ||
        contact::BuildFixedTriangleFeatureTaskMask(
            prepared[0], prepared[1], &prepared_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok)
        return result;
    result.accepted_mask = accepted_mask.local_tasks;
    result.prepared_mask = prepared_mask.local_tasks;
    try {
        const auto accepted_geometry =
            DiscoverDirect(accepted, accepted_mask);
        const auto prepared_geometry =
            DiscoverDirect(prepared, prepared_mask);
        for (const auto& first : first_descriptor.vertex_keys)
            for (const auto& second :
                 second_descriptor.vertex_keys)
                result.shared_vertices +=
                    contact::SameFacetVertexKey(first, second);
        for (const auto& first : first_descriptor.edge_keys)
            for (const auto& second :
                 second_descriptor.edge_keys)
                result.shared_edges +=
                    contact::SameFacetEdgeKey(first, second);
        result.local =
            HasLocalIntersection(accepted_geometry) ||
            HasLocalIntersection(prepared_geometry);
        result.admitted =
            HasAdmittedIntersection(accepted_geometry) ||
            HasAdmittedIntersection(prepared_geometry);
        result.endpoint_separated =
            StrictlySeparated(
                accepted_geometry, accepted_mask,
                result.thickness_m) &&
            StrictlySeparated(
                prepared_geometry, prepared_mask,
                result.thickness_m);
        result.endpoint_contact =
            StrictlyWithinThickness(
                accepted_geometry, result.thickness_m) ||
            StrictlyWithinThickness(
                prepared_geometry, result.thickness_m);
        sct::LinearResidualSeparationResult residual;
        if (!result.local && result.endpoint_separated)
            residual = sct::CertifyQuadraticResidualSeparation(
                accepted[0], prepared[0],
                snapshot.motion_certificates.quadratic[
                    entry.facets.first],
                first_descriptor.reference_half_thickness_m,
                accepted[1], prepared[1],
                snapshot.motion_certificates.quadratic[
                    entry.facets.second],
                second_descriptor.reference_half_thickness_m,
                snapshot.prepared_view.proposed_time -
                    snapshot.prepared_view.base_time,
                {prepared_geometry.features.data(),
                 prepared_geometry.features.size(), true},
                {prepared_geometry.intersections.data(),
                 prepared_geometry.intersections.size(), true});
        sct::PersistentLinearContactResult persistent;
        if (result.endpoint_contact)
            persistent = sct::CertifyPersistentQuadraticContact(
                accepted[0], prepared[0],
                snapshot.motion_certificates.quadratic[
                    entry.facets.first],
                first_descriptor.reference_half_thickness_m,
                accepted[1], prepared[1],
                snapshot.motion_certificates.quadratic[
                    entry.facets.second],
                second_descriptor.reference_half_thickness_m,
                snapshot.prepared_view.proposed_time -
                    snapshot.prepared_view.base_time,
                {prepared_geometry.features.data(),
                 prepared_geometry.features.size(), true},
                snapshot.accepted_certificates.data(),
                snapshot.accepted_certificates.size());
        result.residual_status = residual.status;
        result.persistent_status = persistent.status;
        result.ledger =
            persistent.status ==
            sct::PersistentLinearContactStatus::CertifiedContact;
        if (result.local && !result.admitted &&
            result.endpoint_separated)
            result.classification =
                NonlinearEvidenceClass::ExactLocalIntersection;
        else if (result.ledger && result.endpoint_contact)
            result.classification =
                NonlinearEvidenceClass::PersistentAcceptedLedger;
        else if (result.endpoint_contact)
            result.classification =
                NonlinearEvidenceClass::EndpointThicknessContact;
        else if (result.endpoint_separated)
            result.classification =
                NonlinearEvidenceClass::EndpointSeparated;
        if (result.local && !result.admitted &&
            result.endpoint_separated)
            result.after =
                NonlinearAfterClass::ExactLocalIntersection;
        else if (residual.status ==
                 sct::LinearResidualSeparationStatus::
                     CertifiedSeparated)
            result.after = NonlinearAfterClass::
                CertifiedQuadraticResidualSeparation;
        else if (persistent.status ==
                 sct::PersistentLinearContactStatus::
                     CertifiedContact)
            result.after = NonlinearAfterClass::
                CertifiedPersistentAcceptedContact;
        result.accepted_minimum_m =
            MinimumDistance(accepted_geometry);
        result.prepared_minimum_m =
            MinimumDistance(prepared_geometry);
        result.valid = true;
        return result;
    } catch (...) {
        return result;
    }
}


}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
