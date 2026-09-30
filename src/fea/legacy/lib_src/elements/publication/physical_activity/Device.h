// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "BatchAccess.h"
namespace tl::fea::physical_activity {
struct FamilyControl {
  unsigned long long first_error = UINT64_MAX;
  std::uint32_t active = 0, first_inactive = UINT32_MAX;
  std::uint32_t removed = 0, first_removed = UINT32_MAX;
};
struct DeviceOutput {
  const std::uint8_t* expected_law = nullptr;
  const std::uint8_t* accepted = nullptr;
  std::uint8_t* staging = nullptr;
  FamilyControl* control = nullptr;
};
cudaError_t Capture(const QephInput&, DeviceOutput, cudaStream_t) noexcept;
cudaError_t Capture(const T3Input&, DeviceOutput, cudaStream_t) noexcept;
PhysicalActivityReport Decode(const FamilyControl&, PhysicalActivityFamily) noexcept;
} // namespace tl::fea::physical_activity
