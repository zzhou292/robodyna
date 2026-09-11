#include "ActivityInternal.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::cases::vehicle_startup {
namespace {
using Status=TiedCinActivityStatus;
template<class D> bool SameReceipt(const D& actual,const D& common) {
    using Phase=decltype(actual.phase);
    return actual.valid && common.valid && actual.phase==Phase::Accepted && actual.phase==common.phase &&
        actual.owner_id==common.owner_id && actual.configuration_id==common.configuration_id &&
        actual.qualification_id==common.qualification_id && actual.epoch==common.epoch &&
        actual.base_epoch==common.base_epoch && actual.attempt==common.attempt &&
        actual.time==common.time && actual.base_time==common.base_time &&
        actual.velocity_time==common.velocity_time && actual.base_velocity_time==common.base_velocity_time &&
        actual.kick_dt==common.kick_dt && actual.has_completed_interval==common.has_completed_interval &&
        actual.accepted_force_assembled==common.accepted_force_assembled && actual.usage==common.usage;
}
}
TiedCinActivityReport TiedCinWitnessActivity::CaptureAccepted(const tl::fea::FENodalState& owner,
        const tl::fea::ShellBatchPublication& publication,const tl::fea::ShellFormulationParticipants& participants) {
    auto& state=*impl_;
    if (state.poisoned) return {Status::DeviceFailure,"CIN activity adapter is poisoned"};
    const auto& binding=state.roster.binding().shells();
    const auto checked=publication.ValidateAcceptedActivitySources(owner,participants,binding.inventory());
    if (checked.status==tl::fea::ShellPublicationStatus::DeviceFailure) {
        state.poisoned=true;
        return {Status::DeviceFailure,"Accepted shell publication is poisoned"};
    }
    if (checked.status!=tl::fea::ShellPublicationStatus::Success)
        return {Status::StaleOwner,"Accepted shell participants/inventory do not belong to this owner"};
    const auto stamp=owner.accepted();
    tl::fea::ShellBatchDiagnostics common;
    const auto copied=publication.CopyAcceptedDiagnostics(stamp,&common);
    if (copied.status==tl::fea::ShellPublicationStatus::DeviceFailure) {
        state.poisoned=true;
        return {Status::DeviceFailure,"Accepted shell diagnostics are poisoned"};
    }
    if (copied.status!=tl::fea::ShellPublicationStatus::Success)
        return {Status::StaleOwner,"Accepted common shell endpoint is unavailable"};
    const auto failed=[&](bool device) {
        state.poisoned=state.poisoned || device;
        return TiedCinActivityReport{device ? Status::DeviceFailure : Status::ReadbackFailure,
            "Complete accepted shell activity readback failed"};
    };
    if (participants.qeph) {
        tl::fea::qeph::BatchDiagnostics diagnostic;
        const auto read=participants.qeph->CopyAcceptedParentActivity(stamp,state.qeph.data(),state.qeph.size(),&diagnostic);
        if (read.status!=tl::fea::qeph::BatchStatus::Success || !SameReceipt(diagnostic,common.qeph))
            return failed(read.status==tl::fea::qeph::BatchStatus::DeviceFailure);
    }
    if (participants.t3) {
        tl::fea::t3::BatchDiagnostics diagnostic;
        const auto read=participants.t3->CopyAcceptedParentActivity(stamp,state.t3.data(),state.t3.size(),&diagnostic);
        if (read.status!=tl::fea::t3::BatchStatus::Success || !SameReceipt(diagnostic,common.t3))
            return failed(read.status==tl::fea::t3::BatchStatus::DeviceFailure);
    }
    if (participants.qbat) {
        tl::fea::qbat::BatchDiagnostics diagnostic;
        const auto read=participants.qbat->CopyAcceptedParentActivity(stamp,state.qbat.data(),state.qbat.size(),&diagnostic);
        if (read.status!=tl::fea::qbat::BatchStatus::Success || !SameReceipt(diagnostic,common.qbat))
            return failed(read.status==tl::fea::qbat::BatchStatus::DeviceFailure);
    }
    if (!tl::fea::trial_identity::SameStamp(stamp,owner.accepted()))
        return {Status::StaleOwner,"Owner changed while capturing accepted activity"};
    const auto mapped=cin_witness_detail::MapActivity(state.roster.data(),
        {state.qeph.data(),state.t3.data(),state.qbat.data(),state.qeph.size(),state.t3.size(),state.qbat.size()},
        state.candidate.data(),state.candidate.size());
    if (mapped.status!=Status::Success && mapped.status!=Status::PendingPositiveShellWitness) return mapped;
    state.accepted.swap(state.candidate);
    state.stamp=stamp;
    state.receipt=mapped;
    state.valid=true;
    return mapped;
}
}
