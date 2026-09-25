// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace tlfea::contact::radioss_type25::search {
Status Maintenance::StageReference(const Current& input, ReferenceToken& output) noexcept {
  if (!impl_) return Status::NotInitialized;
  auto& state = *impl_;
  state.pending = false;
  auto status = state.Check(input, true);
  if (status != Status::Ok) return state.Result(status, &input);
  if (!state.OutputDisjoint(&output, sizeof(output), input) ||
      !tl::fea::trial_identity::Disjoint(&output, sizeof(output), this, sizeof(*this)))
    return state.Result(Status::InvalidInput, &input);
  if (state.sequence == UINT64_MAX || state.generation == UINT64_MAX)
    return state.Result(Status::ResourceLimit, &input);
  ++state.sequence;
  status = state.Execute(input, 1 - state.accepted, true, 0, false);
  if (status != Status::Ok) return status;
  state.staged_stamp = input.stamp;
  state.pending = true;
  ReferenceToken token;
  token.owner_ = state.identity;
  token.sequence_ = state.sequence;
  token.generation_ = state.generation + 1;
  output = token;
  return Status::Ok;
}
Status Maintenance::PublishReference(const ReferenceToken& token) noexcept {
  if (!impl_) return Status::NotInitialized;
  auto& state = *impl_;
  if (!state.usable) return state.Result(Status::Unusable);
  if (!state.pending || token.owner_ != state.identity ||
      token.sequence_ != state.sequence || token.generation_ != state.generation + 1)
    return state.Result(Status::StaleReference);
  state.accepted = 1 - state.accepted;
  state.accepted_stamp = state.staged_stamp;
  state.generation = token.generation_;
  state.reference = true;
  state.pending = false;
  return state.Result(Status::Ok);
}
void Maintenance::DiscardReference() noexcept {
  if (impl_) impl_->pending = false;
}
Status Maintenance::Evaluate(const Current& input, double previous_dt,
    bool force_sort, Report& output) noexcept {
  if (!impl_) return Status::NotInitialized;
  auto& state = *impl_;
  auto status = state.Check(input, false);
  if (status != Status::Ok) return state.Result(status, &input);
  if (!state.reference) return state.Result(Status::NoReference, &input);
  if (!state.OutputDisjoint(&output, sizeof(output), input) ||
      !tl::fea::trial_identity::Disjoint(&output, sizeof(output), this, sizeof(*this)))
    return state.Result(Status::InvalidInput, &input);
  const double dt = state.source.input_units == InputUnits::Si ?
      previous_dt / state.factors.time : previous_dt;
  if (!normal_detail::Nonnegative(previous_dt) || !normal_detail::Nonnegative(dt))
    return state.Result(Status::InvalidInput, &input);
  status = state.Execute(input, state.accepted, false, dt, force_sort);
  if (status != Status::Ok) return status;
  Report result;
  result.stamp = input.stamp;
  result.reference_generation = state.generation;
  result.reference_epoch = state.accepted_stamp.epoch;
  result.extrema = state.control.partial.extrema;
  result.budget = state.control.budget;
  output = result;
  return Status::Ok;
}
FailureInfo Maintenance::last_failure() const noexcept {
  if (impl_) return impl_->failure;
  FailureInfo result;
  result.status = Status::NotInitialized;
  return result;
}
std::uint64_t Maintenance::reference_generation() const noexcept {
  return impl_ ? impl_->generation : 0;
}
Forecast Maintenance::allocations() const noexcept {
  return impl_ ? impl_->layout.forecast : Forecast{};
}
} // namespace tlfea::contact::radioss_type25::search
