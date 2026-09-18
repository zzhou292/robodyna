#pragma once

#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "case/vehicle_dynamics/SelfContactContribution.h"
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include "lib_src/collision/self_contact_transaction/CandidateFailureCapture.h"

#include <vector>

namespace crash::cases::vehicle_self_contact {

struct CandidateRigidCouponSnapshot {
    const vehicle_dynamics::Fields* accepted = nullptr;
    const vehicle_dynamics::Fields* prepared = nullptr;
    tl::fea::NodalPreparedView prepared_view;
    std::vector<tl::fea::NodalRigidGroupSnapshot> accepted_groups;
    std::vector<tl::fea::NodalRigidGroupSnapshot> prepared_groups;
};

struct AcceptedAssemblyCouponSnapshot {
    std::vector<double> accepted_positions;
    std::vector<double> prepared_positions;
    std::vector<
        tlfea::contact::self_contact_transaction::
            AcceptedEventCertificate> accepted_certificates;
    std::vector<
        tlfea::contact::self_contact_transaction::
            NonlinearCandidateRosterEntry> nonlinear_roster;
    std::vector<
        tlfea::contact::self_contact_transaction::
            LinearWorkExhaustedRosterEntry> linear_roster;
    tlfea::contact::self_contact_transaction::
        NonlinearCandidateRosterSummary nonlinear_summary;
    tlfea::contact::self_contact_transaction::
        LinearCandidateCensusSummary linear_summary;
    tlfea::contact::self_contact_transaction::
        PreparedMotionCertificateView motion_certificates;
    tl::fea::NodalStamp accepted_stamp;
    tl::fea::NodalPreparedView prepared_view;
    tlfea::contact::SelfContactTransaction* transaction = nullptr;
    const tlfea::contact::SelfContactAcceptedAssemblyReceipt*
        accepted_receipt = nullptr;
    tlfea::contact::self_contact_transaction::
        QualificationPreparedCensusReceipt prepared_census_receipt;
    bool prepared_census = false;
    std::size_t diagnostic_host_upper_bound = 0;
};

// Qualification access only: source snapshots never grant publication authority.
class CandidateRigidCouponAccess {
  public:
    // Test-only controller composition. The decorator remains inside the
    // native dynamics owner and changes only its candidate-seal dispatch.
    static void InstallFailureObserver(
        vehicle_dynamics::VehiclePhysicalDynamics&,
        const tlfea::contact::self_contact_transaction::CandidateFailureObserver&);
    static void SealWithFailureObserver(
        vehicle_dynamics::detail::SelfContactContribution&,
        tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
        const tl::fea::ShellPhysicalDiagnostics&,
        const tl::fea::NodalPreparedView&, vehicle_dynamics::SelfContactObservation&,
        const tlfea::contact::self_contact_transaction::CandidateFailureObserver&);
    static CandidateRigidCouponSnapshot Prepare(
        vehicle_dynamics::VehiclePhysicalDynamics&);
    static AcceptedAssemblyCouponSnapshot PrepareAcceptedAssembly(
        vehicle_dynamics::VehiclePhysicalDynamics&);
    // Reuses actual structural and wall candidate stages, then captures the
    // second candidate census without sealing or committing self contact.
    static AcceptedAssemblyCouponSnapshot PrepareCandidateCensus(
        vehicle_dynamics::VehiclePhysicalDynamics&);
    // Executes the real accepted/structural/wall stages and one production
    // self seal with a bounded failure observer. Success remains a prepared
    // step for CommitStep; failure preserves the native report and rolls back.
    static tlfea::contact::SelfContactTransactionReport PrepareWithFailureObserver(
        vehicle_dynamics::VehiclePhysicalDynamics&,
        const tlfea::contact::self_contact_transaction::CandidateFailureObserver&);
  private:
    static AcceptedAssemblyCouponSnapshot CaptureCensus(
        vehicle_dynamics::VehiclePhysicalDynamics&, bool prepared_activity);
};

}  // namespace crash::cases::vehicle_self_contact
