#include "CandidateCapture.h"

#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_self_contact/RuntimeData.h"
#include "case/vehicle_self_contact/runtime/Stages.h"
#include "case/vehicle_self_contact/runtime/Operations.h"
#include "output/ArtifactIO.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace crash::cases::vehicle_self_contact {
tlfea::contact::SelfContactTransactionReport
CandidateRigidCouponAccess::PrepareWithFailureObserver(
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics,
    const tlfea::contact::self_contact_transaction::CandidateFailureObserver& observer) {
    namespace c = tlfea::contact;
    auto& storage = *dynamics.storage_;
    output::Require(!storage.pending, "Discard or commit the existing diagnostic step first");
    c::SelfContactTransactionReport result;
    try {
        storage.Prepare();
        // Same candidate-stage reuse as CaptureCensus, without a census or
        // full-node host copy. Only the final self-seal call is substituted.
        auto retained_self = std::move(storage.self_contact);
        try {
            storage.Evaluate();
        } catch (...) {
            storage.self_contact = std::move(retained_self);
            throw;
        }
        storage.self_contact = std::move(retained_self);
        auto* stages = dynamic_cast<detail::SelfContactStages*>(storage.self_contact.get());
        output::Require(stages != nullptr, "Failure observer requires actual self-contact stages");
        stages->receipt_ = {};
        runtime::CheckCandidateObservation(storage.candidate().self_contact, storage.prepared);
        result = c::self_contact_transaction::QualificationAccess::SealCandidateWithFailureObserver(
            stages->contact.data_->transaction, storage.state().owner, storage.token,
            storage.candidate().mechanics, storage.prepared, stages->accepted_,
            &stages->receipt_, observer);
        if (result.status != c::SelfContactTransactionStatus::Ok) {
            storage.Discard();
            return result;
        }
        runtime::ObserveCandidate(storage.candidate().self_contact, storage.prepared,
                                  stages->receipt_);
        stages->accepted_ = {};
        storage.Capture();
        storage.pending = true;
        return result;
    } catch (...) {
        storage.Discard();
        throw;
    }
}

CandidateRigidCouponSnapshot CandidateRigidCouponAccess::Prepare(
        vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
        dynamics.PrepareStep();
        auto& storage = *dynamics.storage_;
        auto& owner = storage.state().owner;
        const auto group_count =
            storage.prepared.rigid_groups.group_count;
        CandidateRigidCouponSnapshot result;
        result.accepted = &storage.fields[storage.accepted_slot];
        result.prepared = &storage.candidate_fields();
        result.prepared_view = storage.prepared;
        result.accepted_groups.resize(group_count);
        result.prepared_groups.resize(group_count);
        tl::fea::NodalStamp accepted_stamp;
        auto report = owner.CopyAcceptedRigidGroups(
            {result.accepted_groups.data(), group_count},
            &accepted_stamp);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        tl::fea::NodalPreparedView prepared_view;
        report = owner.CopyPreparedRigidGroups(
            storage.token,
            {result.prepared_groups.data(), group_count},
            &prepared_view);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        if (!tl::fea::trial_identity::SameStamp(
                accepted_stamp, storage.stamp) ||
            !tl::fea::trial_identity::SamePrepared(
                prepared_view, storage.prepared))
            throw std::runtime_error(
                "Coupon rigid snapshots differ from the prepared owner");
        return result;
    }

AcceptedAssemblyCouponSnapshot CandidateRigidCouponAccess::PrepareAcceptedAssembly(
        vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
    return CaptureCensus(dynamics, false);
}

AcceptedAssemblyCouponSnapshot CandidateRigidCouponAccess::PrepareCandidateCensus(
        vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
    return CaptureCensus(dynamics, true);
}

AcceptedAssemblyCouponSnapshot CandidateRigidCouponAccess::CaptureCensus(
        vehicle_dynamics::VehiclePhysicalDynamics& dynamics,
        bool prepared_activity) {
        const auto started = std::chrono::steady_clock::now();
        auto& storage = *dynamics.storage_;
        storage.Prepare();
        if (prepared_activity) {
            // Keep the real self accepted force/STI in Prepare(). Reuse every
            // structural candidate and wall stage from Evaluate(), replacing
            // only self candidate sealing with the diagnostic census below.
            auto retained_self = std::move(storage.self_contact);
            try {
                storage.Evaluate();
            } catch (...) {
                storage.self_contact = std::move(retained_self);
                storage.Discard();
                throw;
            }
            storage.self_contact = std::move(retained_self);
        }
        const auto accepted_ready = std::chrono::steady_clock::now();
        std::cerr << "V5_NONLINEAR_COUPON_PHASE accepted_prepare_s="
                  << std::chrono::duration<double>(
                         accepted_ready - started).count()
                  << std::endl;
        auto& owner = storage.state().owner;
        const auto nodes = storage.startup.accepted().node_count;
        AcceptedAssemblyCouponSnapshot result;
        result.prepared_census = prepared_activity;
        const std::size_t roster_capacity =
            prepared_activity ? std::size_t{1} << 18 : std::size_t{1} << 20;
        // Charge every temporary host snapshot/roster plus a fixed exporter
        // reserve before allocation. The outer 10 GiB RSS guard remains final.
        tl::util::BoundedArenaLayout diagnostic_budget(
            prepared_activity ? std::size_t{1} << 30 : std::size_t{2} << 30);
        tl::util::ArenaRegion budget_region;
        using namespace tlfea::contact::self_contact_transaction;
        if (!dynamics.self_contact_forecast())
            throw std::runtime_error("Candidate census requires a retained self-contact source");
        if (!diagnostic_budget.Append<double>(12 * nodes, budget_region) ||
            !diagnostic_budget.Append<NonlinearCandidateRosterEntry>(roster_capacity, budget_region) ||
            !diagnostic_budget.Append<LinearWorkExhaustedRosterEntry>(roster_capacity, budget_region) ||
            !diagnostic_budget.Append<AcceptedEventCertificate>(
                dynamics.self_contact_forecast()->transaction.accepted_event_capacity, budget_region) ||
            !diagnostic_budget.Append<std::byte>(320u << 20, budget_region))
            throw std::runtime_error("Complete diagnostic census exceeds its fixed additional host cap");
        result.diagnostic_host_upper_bound = diagnostic_budget.bytes();
        result.accepted_positions.resize(3 * nodes);
        result.prepared_positions.resize(3 * nodes);
        std::vector<double> accepted_velocities(3 * nodes);
        std::vector<double> prepared_velocities(3 * nodes);
        auto report = owner.CopyAccepted(
            {result.accepted_positions.data(),
             accepted_velocities.data(), nodes},
            &result.accepted_stamp);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        report = owner.CopyPrepared(
            storage.token,
            {result.prepared_positions.data(),
             prepared_velocities.data(), nodes},
            &result.prepared_view);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        if (!tl::fea::trial_identity::SameStamp(
                result.accepted_stamp, storage.stamp) ||
            !tl::fea::trial_identity::SamePrepared(
                result.prepared_view, storage.prepared))
            throw std::runtime_error(
                "Accepted-assembly coupon differs from owner identity");
        const auto* stages = dynamic_cast<
            const detail::SelfContactStages*>(
                storage.self_contact.get());
        if (!stages)
            throw std::runtime_error(
                "Accepted-assembly coupon has no self-contact stages");
        result.transaction = &stages->contact.data_->transaction;
        result.accepted_receipt = &stages->accepted_;
        result.nonlinear_roster.resize(roster_capacity);
        result.linear_roster.resize(roster_capacity);
        std::size_t nonlinear_count = 0;
        std::size_t linear_count = 0;
        const auto roster_report = prepared_activity ?
            tlfea::contact::self_contact_transaction::
            QualificationAccess::
                ClassifyPreparedCandidateCensus(
                    stages->contact.data_->transaction,
                    owner, storage.token, storage.candidate().mechanics,
                    storage.prepared, stages->accepted_,
                    result.nonlinear_roster.data(),
                    result.nonlinear_roster.size(),
                    &nonlinear_count, &result.nonlinear_summary,
                    result.linear_roster.data(), result.linear_roster.size(),
                    &linear_count, &result.linear_summary,
                    &result.motion_certificates,
                    &result.prepared_census_receipt) :
            tlfea::contact::self_contact_transaction::
            QualificationAccess::
                ClassifyPreparedCandidateCensus(
                    stages->contact.data_->transaction,
                    owner, storage.token, storage.prepared,
                    stages->accepted_,
                    result.nonlinear_roster.data(),
                    result.nonlinear_roster.size(),
                    &nonlinear_count,
                    &result.nonlinear_summary,
                    result.linear_roster.data(),
                    result.linear_roster.size(),
                    &linear_count,
                    &result.linear_summary,
                    &result.motion_certificates);
        if (roster_report.status !=
            tlfea::contact::SelfContactTransactionStatus::Ok)
            throw std::runtime_error(roster_report.message);
        std::cerr << "V5_NONLINEAR_COUPON_PHASE nonlinear_roster_s="
                  << std::chrono::duration<double>(
                         std::chrono::steady_clock::now() -
                         accepted_ready).count()
                  << std::endl;
        result.nonlinear_roster.resize(nonlinear_count);
        result.linear_roster.resize(linear_count);
        const auto certificates =
            tlfea::contact::self_contact_transaction::
                QualificationAccess::AcceptedCertificates(
                    stages->contact.data_->transaction);
        if (!certificates.complete ||
            (certificates.count && !certificates.data))
            throw std::runtime_error(
                "Accepted certificate ledger is incomplete");
        if (certificates.count)
            result.accepted_certificates.assign(
                certificates.data,
                certificates.data + certificates.count);
        return result;
    }

}  // namespace crash::cases::vehicle_self_contact
