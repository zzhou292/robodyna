#pragma once
#include "../ShellBatchFailure.h"
#include "lib_utils/BoundedArena.h"

namespace tl::fea::shell_batch_plasticity_detail {
struct FailureDeviceStorage {
  ShellFailurePolicy* policy = nullptr;
  sections::ConstantFailureParameters* parameters = nullptr;
  ShellBatchFailureState* state[2]{};
};
struct FailureLayout {
  util::ArenaRegion header, policy, parameters, state[2];
  std::size_t bytes = 0;
  bool Initialize(std::size_t count, std::size_t cap) noexcept {
    if (!count || count > 524288) return false;
    FailureLayout next;
    util::BoundedArenaLayout arena(cap);
    if (!arena.Append<FailureDeviceStorage>(1, next.header) ||
        !arena.Append<ShellFailurePolicy>(count, next.policy) ||
        !arena.Append<sections::ConstantFailureParameters>(count, next.parameters)) {
      return false;
    }
    for (auto& region : next.state) {
      if (!arena.Append<ShellBatchFailureState>(count, region)) return false;
    }
    next.bytes = arena.bytes();
    *this = next;
    return true;
  }
  FailureDeviceStorage* Construct(util::HostArena& arena) const noexcept {
    auto* out = arena.Construct<FailureDeviceStorage>(header);
    if (!out) return nullptr;
    out->policy = arena.Construct<ShellFailurePolicy>(policy);
    out->parameters = arena.Construct<sections::ConstantFailureParameters>(parameters);
    for (unsigned i = 0; i < 2; ++i) {
      out->state[i] = arena.Construct<ShellBatchFailureState>(state[i]);
    }
    return out->policy && out->parameters && out->state[0] && out->state[1] ? out : nullptr;
  }
  FailureDeviceStorage Rebase(void* device) const noexcept {
    FailureDeviceStorage out;
    out.policy = util::ArenaPointer<ShellFailurePolicy>(device, policy);
    out.parameters = util::ArenaPointer<sections::ConstantFailureParameters>(device, parameters);
    for (unsigned i = 0; i < 2; ++i) {
      out.state[i] = util::ArenaPointer<ShellBatchFailureState>(device, state[i]);
    }
    return out;
  }
};
} // namespace tl::fea::shell_batch_plasticity_detail
