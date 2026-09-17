#pragma once

#include "CandidateCapture.h"
#include "NonlinearCoverageFixture.h"
#include "LinearCoverageFixture.h"

#include <array>
#include <vector>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace contact = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;
struct EndpointGeometry {
    std::vector<contact::FixedTriangleFeatureCandidate> features;
    std::vector<contact::FixedTriangleIntersection> intersections;
    contact::FixedTriangleDiscoveryReport report;
};

enum class NonlinearEvidenceClass : unsigned {
    ExactLocalIntersection,
    EndpointSeparated,
    PersistentAcceptedLedger,
    EndpointThicknessContact,
    PossibleCurvedCrossing,
    Count,
};

enum class NonlinearAfterClass : unsigned {
    ExactLocalIntersection,
    CertifiedQuadraticResidualSeparation,
    CertifiedPersistentAcceptedContact,
    PossibleCurvedCrossing,
    Count,
};

struct NonlinearPairEvidence {
    NonlinearEvidenceClass classification =
        NonlinearEvidenceClass::PossibleCurvedCrossing;
    NonlinearAfterClass after =
        NonlinearAfterClass::PossibleCurvedCrossing;
    sct::LinearResidualSeparationStatus residual_status =
        sct::LinearResidualSeparationStatus::InvalidInput;
    sct::PersistentLinearContactStatus persistent_status =
        sct::PersistentLinearContactStatus::InvalidInput;
    unsigned shared_vertices = 0;
    unsigned shared_edges = 0;
    std::uint16_t accepted_mask = 0;
    std::uint16_t prepared_mask = 0;
    bool local = false;
    bool admitted = false;
    bool endpoint_separated = false;
    bool endpoint_contact = false;
    bool ledger = false;
    bool valid = false;
    double accepted_minimum_m = 0;
    double prepared_minimum_m = 0;
    double thickness_m = 0;
};


EndpointGeometry DiscoverDirect(
    const std::array<tlfea::contact::CurrentFixedTriangle, 2>&,
    tlfea::contact::FixedTriangleFeatureTaskMask);
double MinimumDistance(const EndpointGeometry&);
std::size_t ExpectedFeatureCount(tlfea::contact::FixedTriangleFeatureTaskMask);
bool StrictlySeparated(const EndpointGeometry&,
    tlfea::contact::FixedTriangleFeatureTaskMask, double thickness);
bool StrictlyWithinThickness(const EndpointGeometry&, double thickness);
bool HasLocalIntersection(const EndpointGeometry&);
bool HasAdmittedIntersection(const EndpointGeometry&);
bool CouldOwnQuadraticPair(
    const std::array<tlfea::contact::CurrentFixedTriangle, 2>&,
    const tlfea::contact::self_contact_transaction::AcceptedEventCertificate&);
bool CouldOwnFacetPairConservatively(
    const std::array<tlfea::contact::CurrentFixedTriangle, 2>&,
    const tlfea::contact::self_contact_transaction::AcceptedEventCertificate&);
nonlinear_fixture::Pair FreezeNonlinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot&,
    const tlfea::contact::self_contact_transaction::NonlinearCandidateRosterEntry&);
linear_fixture::Pair FreezeLinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot&,
    const tlfea::contact::self_contact_transaction::LinearWorkExhaustedRosterEntry&);
void FreezeAcceptedPolicies(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot&,
    std::vector<nonlinear_fixture::Pair>*);
std::vector<tlfea::contact::self_contact_transaction::AcceptedFeatureExclusionCertificate>
FrozenSameRigidExclusions(const nonlinear_fixture::Pair&);
nonlinear_fixture::PhaseIdentity FixturePhaseIdentity(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot&);
NonlinearPairEvidence InspectNonlinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot&,
    const tlfea::contact::self_contact_transaction::NonlinearCandidateRosterEntry&);
// Endpoint-local classification is descriptive. Only an actual continuous
// certificate permits omission from the frozen unknown roster.
bool RequiresFrozenNonlinearReplay(const NonlinearPairEvidence&) noexcept;

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
