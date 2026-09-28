// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
namespace tlfea::contact::radioss_type25::activity_operands {
StageReport State::Stage(unsigned accepted, const tl::fea::PhysicalActivityDeviceView& activity) noexcept {
  using S = TransactionStatus; StageReport out;
  if (!impl_) { out.report = {S::NotInitialized, "Contact operands are not initialized"}; return out; }
  auto& s = *impl_;
  if (!s.usable) { out.report = {S::Unusable, "Contact operand storage is poisoned"}; return out; }
  if (accepted > 1 || !s.ready[accepted]) {
    out.report = {S::StaleAttempt, "Contact operand accepted slot is unavailable"}; return out;
  }
  const auto trial = accepted^1u; s.ready[trial] = false;
  const auto& shape = s.shape;
  const auto mask = [&](const tl::fea::PhysicalActivityFamilyView& v, std::size_t expected) {
    return v.summary.count == expected && (!expected || (v.base && v.current &&
        detail::Disjoint(v.base, expected, s.arena, s.layout.bytes) &&
        detail::Disjoint(v.current, expected, s.arena, s.layout.bytes)));
  };
  if (!activity.generation || !activity.attempt || !activity.accepted.owner_id ||
      activity.stream != s.stream || activity.accepted.node_count != shape.nodes ||
      !mask(activity.qeph, shape.qeph) || !mask(activity.t3, shape.t3) ||
      activity.qbat_count != shape.qbat || activity.type45_count != shape.type45) {
    out.report = {S::SourceMismatch, "Fresh physical activity view does not match contact source shape or stream"}; return out;
  }
  s.control = {};
  const auto failure = [&]() {
    cudaStreamSynchronize(s.stream); s.usable = false;
    out.report = {S::DeviceFailure, "Contact activity staging failed"}; return out;
  };
  auto& d = s.device;
  if (cudaMemcpyAsync(d.control, &s.control, sizeof(s.control), cudaMemcpyHostToDevice, s.stream) != cudaSuccess ||
      cudaMemsetAsync(d.events, 0, shape.mains*sizeof(*d.events), s.stream) != cudaSuccess ||
      detail::Launch(d, accepted, activity, s.stream) != cudaSuccess ||
      cudaMemcpyAsync(&s.control, d.control, sizeof(s.control), cudaMemcpyDeviceToHost, s.stream) != cudaSuccess ||
      cudaStreamSynchronize(s.stream) != cudaSuccess) return failure();
  out.report = detail::Decode(s.control);
  if (out.report.status != S::Ok) return out;
  if (s.control.free_count > shape.mains || s.control.removed_mains > shape.mains ||
      s.control.orphans > shape.secondaries || s.control.affected > shape.emitting ||
      s.control.removed_events > s.control.affected) {
    out.report = {S::NumericalFailure, "Contact activity scalar counts are inconsistent"}; return out;
  }
  s.ready[trial] = true; s.free_count[trial] = s.control.free_count;
  out.changed = s.control.changed != 0; out.staged_slot = trial;
  out.affected_events = s.control.affected; out.removed_events = s.control.removed_events;
  out.removed_mains = s.control.removed_mains; out.orphan_secondaries = s.control.orphans;
  out.free_count = s.control.free_count;
  return out;
}
} // namespace tlfea::contact::radioss_type25::activity_operands
