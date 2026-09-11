#include "ContactSummary.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace crash::cases::vehicle_run {
void ObserveAcceptedContact(ContactTotals& output,const vehicle_dynamics::StepObservation& step,
    const tl::fea::NodalStamp& accepted) {
    namespace contact=tlfea::contact;
    const auto& a=step.wall.accepted.contact;
    const auto& b=step.wall.prepared.contact;
    if(!step.wall.enabled || !step.wall.accepted.valid || !step.wall.prepared.valid || !a.valid || !b.valid ||
        !step.wall.prepared.prepared_activity_available || a.phase!=contact::NodalWallDevicePhase::AcceptedBase ||
        b.phase!=contact::NodalWallDevicePhase::PreparedCandidate ||
        a.owner_id!=accepted.owner_id || b.owner_id!=accepted.owner_id ||
        accepted.epoch==0 || a.base_epoch!=accepted.epoch-1 || b.base_epoch!=a.base_epoch ||
        a.attempt!=b.attempt || b.time!=accepted.time || step.proposed_time!=accepted.time ||
        output.intervals!=accepted.epoch-1)
        throw std::invalid_argument("Contact summary requires the next actual accepted wall interval");
    auto next=output;
    for(const auto* row:{&a,&b}) {
        if(!std::isfinite(row->resultant.value) || row->resultant.value<0 ||
            !std::isfinite(row->maximum_penetration) || row->maximum_penetration<0 ||
            !std::isfinite(row->potential.value) || row->potential.value<0)
            throw std::invalid_argument("Contact endpoint observation is nonfinite or negative");
        next.peak_observed_force_n=std::max(next.peak_observed_force_n,row->resultant.value);
        next.peak_observed_penetration_m=std::max(next.peak_observed_penetration_m,row->maximum_penetration);
        next.peak_observed_potential_j=std::max(next.peak_observed_potential_j,row->potential.value);
    }
    next.reported_drift_work_sum_j+=b.drift_work;
    next.last_removed_potential_j=step.wall.prepared.removed_potential.value;
    if(!std::isfinite(next.reported_drift_work_sum_j) || !std::isfinite(next.last_removed_potential_j) ||
        next.last_removed_potential_j<0)
        throw std::invalid_argument("Contact work/removal summary is unrepresentable");
    next.last_same_mask_potential_j=b.potential.value;
    next.accepted_active_parents=step.wall.prepared.accepted_active_parents;
    next.proposed_active_parents=step.wall.prepared.proposed_active_parents;
    next.intervals=accepted.epoch;
    next.available=true;
    output=next;
}
} // namespace crash::cases::vehicle_run
