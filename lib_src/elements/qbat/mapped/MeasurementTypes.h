// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstdint>
#include <type_traits>

namespace tl::fea::qbat::mapped {
// Fresh operands for one parent, never an accepted cache or a partial sum.
// Each slot retains the operand of the original -= expression independently.
struct MeasurementParent {
  double internal_work[2]{}, internal_increment[2]{};
  double plastic_work=0, plastic_increment=0;
  double viscous_work=0, viscous_increment=0;
  double kick_operand[4]{}, drift_operand[4]{};
  double area_ratio=0, thickness_ratio=0, native_dt=0, maximum_strain=0;
  std::uint8_t valid=0, active=0, newly_removed=0;
};
static_assert(std::is_trivially_copyable_v<MeasurementParent>);
static_assert(sizeof(MeasurementParent)==168 && alignof(MeasurementParent)==8,
    "Complete bounded measurement operand packet");
} // namespace tl::fea::qbat::mapped
