// Generated exact original one-group body; test-only local status sink.
#pragma once
#include "lib_src/solvers/cin_advance/Groups.h"
namespace tl::fea::cin_group_test {
inline void FrozenMotion(const cin_advance::Input& input, std::uint32_t g) {
  auto* control = input.control;
  const auto* accepted = input.accepted;
  auto* trial = input.trial;
  auto* loads = input.loads;
  const auto groups = input.groups;
  const auto durations = input.durations;
  const auto maximum_angle = input.maximum_angle;
  const auto capture = input.capture;
  const auto n = input.model.node_count;
  const auto Fail = [](nodal_detail::Control* c, NodalStatus status, std::uint32_t node) {
    c->status = status; c->node = node;
    if (status == NodalStatus::StepTooLarge) c->limit.dt = 0;
  };

    const auto result = capture.node
      ? rigid::PrepareGroupCandidate<true>(groups, g, accepted, trial, loads, n, durations, capture)
      : rigid::PrepareGroupCandidate<false>(groups, g, accepted, trial, loads, n, durations);
    if (result.status != rigid::StepStatus::Success) {
      Fail(control, result.status == rigid::StepStatus::RotationLimit
          ? NodalStatus::StepTooLarge : NodalStatus::InvalidOutput, result.node);
      return;
    }
    const auto range = groups.groups[g];
    for (std::uint32_t i = 0; i < range.count; ++i) {
      const auto node = groups.members[range.offset+i].node;
      const auto status = nodal_detail::PrepareNodeOrientation(accepted, trial, node, n,
          durations.drift_dt, maximum_angle, false);
      if (status != NodalStatus::Ok) {
        Fail(control, status, node);
        return;
      }
    }
  
}
}
