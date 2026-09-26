// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "PhysicalScratchParticipationState.h"
#include "../ShellBatchPublication.h"
#include <cstdint>
#include <limits>

namespace tl::fea::shell_publication_detail {
// Call-local normalization only: no retained caller arrays or allocation.
struct NormalizedScratchRoster {
  PhysicalScratchParticipationEntry entries[PhysicalScratchSlotCapacity];
  std::size_t slots = PhysicalScratchKindCount, present = 0;
  bool native = false;
};
template<class T> bool ScratchSpan(const T* p, std::size_t count) noexcept {
  if (!count) return p == nullptr;
  if (!p || count > SIZE_MAX / sizeof(T)) return false;
  const auto address = reinterpret_cast<std::uintptr_t>(p);
  return address % alignof(T) == 0 && count * sizeof(T) <= UINTPTR_MAX - address;
}
inline bool ScratchPresent(const ShellPhysicalScratchRosterEntry& entry) noexcept {
  return entry.issuer || entry.source_id;
}
inline ShellPublicationReport NormalizeScratchRoster(
    const ShellPhysicalScratchRoster& roster,
    const ShellPhysicalScratchParticipationLimits& limits,
    NormalizedScratchRoster& result) noexcept {
  using S = ShellPublicationStatus;
  const auto& native = roster.native_interfaces;
  result.native = native.entries || native.count;
  if (result.native) {
    if (native.count > MaxNativeContactInterfaces || native.count > limits.max_native_interfaces ||
        limits.max_native_interfaces > MaxNativeContactInterfaces)
      return {S::ResourceLimit, "Native interface count exceeds the bounded group capacity"};
    if (!native.count || !ScratchSpan(native.entries, native.count) ||
        ScratchPresent(roster.mapped_wall) || ScratchPresent(roster.self_contact))
      return {S::InvalidInput, "Native group view is malformed or mixed with legacy slots"};
    result.slots = native.count;
    for (std::size_t i = 0; i < native.count; ++i) {
      const auto& entry = native.entries[i];
      result.entries[i] = {entry.issuer(), entry.source_id(), ShellPhysicalScratchContributorKind::NativeContact};
    }
  } else {
    result.entries[0] = {roster.mapped_wall.issuer, roster.mapped_wall.source_id,
                         ShellPhysicalScratchContributorKind::MappedWall};
    result.entries[1] = {roster.self_contact.issuer, roster.self_contact.source_id,
                         ShellPhysicalScratchContributorKind::SelfContact};
  }
  for (std::size_t i = 0; i < result.slots; ++i) {
    const auto& entry = result.entries[i];
    if (!entry.issuer && !entry.source_id && !result.native) continue;
    if (!entry.source_id || !ScratchSpan(entry.issuer, std::size_t{1}))
      return {S::InvalidInput, "Scratch participation entry is incomplete or misaligned"};
    ++result.present;
    for (std::size_t j = 0; j < i; ++j) {
      if (entry.issuer == result.entries[j].issuer ||
          (result.native && entry.source_id == result.entries[j].source_id))
        return {S::InvalidInput, "Scratch participation has a duplicate issuer or native source ID"};
    }
  }
  return result.present ? ShellPublicationReport{S::Success, "OK"}
                        : ShellPublicationReport{S::InvalidInput, "Scratch participation roster is empty"};
}
inline bool ScratchReceiptShape(const ShellPhysicalScratchReceiptRoster& receipts,
                                const PhysicalScratchParticipationState* state) noexcept {
  const auto& native = receipts.native_interfaces;
  if (!state || !state->native_group) return native.count == 0 && native.entries == nullptr;
  return !receipts.mapped_wall && !receipts.self_contact && native.count == state->entry_count &&
      native.count <= MaxNativeContactInterfaces && ScratchSpan(native.entries, native.count);
}
inline const ShellPhysicalScratchParticipationReceipt* ScratchReceipt(
    const ShellPhysicalScratchReceiptRoster& receipts,
    const PhysicalScratchParticipationState& state, std::size_t slot) noexcept {
  if (state.native_group) return receipts.native_interfaces.entries[slot];
  return slot == 0 ? receipts.mapped_wall : receipts.self_contact;
}
} // namespace tl::fea::shell_publication_detail
