#pragma once
#include "../VehiclePhysicalDynamics.h"

namespace crash::cases::vehicle_dynamics::limiter {
inline bool MatchesAccepted(const StepObservation& step, const tl::fea::NodalStamp& accepted) noexcept {
    const auto& receipt = step.structural_limiter;
    return receipt.values.kind != tl::fea::NodalCinLimitKind::Unavailable &&
        receipt.owner_id == accepted.owner_id && receipt.owner_id == step.base.owner_id &&
        receipt.base_epoch == step.base.epoch && receipt.base_epoch != UINT64_MAX &&
        receipt.base_epoch+1 == accepted.epoch && receipt.base_time_s == step.base.time &&
        step.proposed_time == accepted.time && receipt.owner_fixed_dt_s == accepted.fixed_dt &&
        receipt.attempt && receipt.values.minimum_dt_s == step.structural_step_limit;
}
} // namespace crash::cases::vehicle_dynamics::limiter
