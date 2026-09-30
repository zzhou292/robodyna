// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <cstddef>
#include <cstdint>

namespace tlfea::contact::self_contact_transaction {

struct QualificationRange {
  const void* data = nullptr;
  std::size_t bytes = 0;
  bool valid = true;
};

template <class T>
QualificationRange QualificationBorrowedRange(
    const T* data, std::size_t count = 1) noexcept {
  if (!count) return {};
  if (!data || count > SIZE_MAX / sizeof(T)) return {nullptr, 0, false};
  const auto bytes = count * sizeof(T);
  if (bytes > UINTPTR_MAX - reinterpret_cast<std::uintptr_t>(data))
    return {nullptr, 0, false};
  return {data, bytes, true};
}

// Validate before writing any output, including scalar counts. Empty optional
// ranges are skipped. The retained-state predicate and existing address-range
// utility enforce the same bounds without allocation or borrowed dereferences.
template <std::size_t Outputs, std::size_t Inputs, class RetainedDisjoint>
bool ValidateQualificationRanges(
    const QualificationRange (&output)[Outputs],
    const QualificationRange (&input)[Inputs],
    RetainedDisjoint retained_disjoint) noexcept {
  for (const auto& value : input)
    if (!value.valid) return false;
  for (std::size_t index = 0; index < Outputs; ++index) {
    const auto& value = output[index];
    if (!value.valid) return false;
    if (!value.bytes) continue;
    if (!retained_disjoint(value.data, value.bytes)) return false;
    for (const auto& borrowed : input)
      if (borrowed.bytes && !tl::fea::trial_identity::Disjoint(
              value.data, value.bytes, borrowed.data, borrowed.bytes))
        return false;
    for (std::size_t previous = 0; previous < index; ++previous)
      if (output[previous].bytes && !tl::fea::trial_identity::Disjoint(
              value.data, value.bytes,
              output[previous].data, output[previous].bytes))
        return false;
  }
  return true;
}

}  // namespace tlfea::contact::self_contact_transaction
