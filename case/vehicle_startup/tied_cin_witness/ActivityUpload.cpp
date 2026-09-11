#include "ActivityInternal.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::cases::vehicle_startup {
TiedCinActivityReport TiedCinWitnessActivity::UploadAttempt(tl::fea::FENodalState& owner,
        const tl::fea::NodalTrialToken& token) {
    using Status=TiedCinActivityStatus;
    auto& state=*impl_;
    const auto reject=[&](TiedCinActivityReport report) {
        owner.Discard();
        return report;
    };
    if (state.poisoned) return reject({Status::DeviceFailure,"CIN activity adapter is poisoned"});
    if (!state.valid || !tl::fea::trial_identity::SameStamp(state.stamp,owner.accepted()))
        return reject({Status::StaleOwner,"CIN activity is not from this owner's actual accepted endpoint"});
    if (state.receipt.status!=Status::Success) return reject(state.receipt);
    if (!state.roster.runtime_mappable())
        return reject({Status::PendingPositiveShellWitness,"CIN shell roster has pending domain/count obligations"});
    const auto& roster=state.roster.data();
    const tl::fea::NodalCinWitnessSource source{&state.roster.attachments().model(),roster.ranges.data(),
        roster.witnesses.data(),roster.ranges.size(),roster.witnesses.size()};
    const auto checked=owner.ValidateCinWitnessSource(source);
    if (checked.status!=tl::fea::NodalStatus::Ok)
        return reject({Status::InvalidInput,"Actual CIN owner retained a different source roster"});
    tl::fea::NodalCinAssemblyView view;
    if (owner.BorrowCinAssembly(token,&view).status!=tl::fea::NodalStatus::Ok ||
        view.owner_id!=state.stamp.owner_id || view.base_epoch!=state.stamp.epoch || !view.attempt ||
        !view.qualification_id || view.witness_count!=state.accepted.size() ||
        view.node_count!=state.roster.attachments().model().domain()->node_count())
        return reject({Status::StaleOwner,"CIN activity destination is not the authentic open attempt"});
    auto error=cudaMemcpyAsync(view.witness_activity,state.accepted.data(),state.accepted.size(),
                               cudaMemcpyHostToDevice,view.stream);
    if (error==cudaSuccess) error=cudaStreamSynchronize(view.stream);
    if (error!=cudaSuccess) {
        state.poisoned=true;
        return reject({Status::DeviceFailure,"CIN witness activity upload failed; adapter is unusable"});
    }
    return {Status::Success,"Complete accepted witness flags copied into the existing CIN attempt"};
}
}
