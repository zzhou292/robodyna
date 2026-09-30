#pragma once
#include "Fixture.h"
#include "lib_src/solvers/FENodalStateStorage.h"

namespace tl::fea::reset_test {
inline void SeedControl(nodal_detail::Control& value) {
  value.assembly = {tlfea::contact::Status::kOutOfRange, 43, 81, 82};
  value.limit = {1., 2., 3., 4, 5, 6, 7, true};
  value.status = NodalStatus::StepTooLarge; value.node = 21;
  value.structural_limiter.epoch = 31; value.structural_limiter.attempt = 32;
  auto& witness = value.structural_limiter.values;
  witness.kind = NodalCinLimitKind::RigidTrace; witness.node = 11; witness.group = 12;
  witness.translation_fixed_bits = 7; witness.rotation_fixed = witness.rotation_present = 1;
  witness.minimum_dt_s = 1; witness.factor = 2; witness.mass_kg = 3; witness.inertia_kg_m2 = 4;
  witness.translation_stiffness_n_per_m = 5; witness.rotation_stiffness_nm = 6;
  witness.principal_inertia_kg_m2 = {7, 8, 9}; witness.trace_upper_per_s2 = 10;
}
inline void ClearedControl(const nodal_detail::Control& value, std::uint64_t epoch,
                           std::uint64_t attempt, NodalStatus status) {
  EXPECT_EQ(value.status, status); EXPECT_EQ(value.node, UINT32_MAX);
  EXPECT_EQ(value.assembly.status, tlfea::contact::Status::kOk);
  EXPECT_EQ(value.assembly.node, UINT32_MAX);
  EXPECT_EQ(value.assembly.base_epoch, epoch); EXPECT_EQ(value.assembly.attempt, attempt);
  EXPECT_EQ(Bits(value.limit.dt), Bits(0.));
  EXPECT_EQ(Bits(value.limit.stiffness_bound), Bits(0.));
  EXPECT_EQ(Bits(value.limit.damping_bound), Bits(0.));
  EXPECT_EQ(value.limit.stiffness_node, 0u); EXPECT_EQ(value.limit.damping_node, 0u);
  EXPECT_EQ(value.limit.base_epoch, 0u); EXPECT_EQ(value.limit.attempt, 0u);
  EXPECT_FALSE(value.limit.has_stiffness_or_damping);
  const auto& witness = value.structural_limiter;
  EXPECT_EQ(witness.epoch, 0u); EXPECT_EQ(witness.attempt, 0u);
  EXPECT_EQ(witness.values.kind, NodalCinLimitKind::Unavailable);
  EXPECT_EQ(witness.values.node, UINT32_MAX); EXPECT_EQ(witness.values.group, UINT32_MAX);
  EXPECT_EQ(witness.values.translation_fixed_bits, 0u);
  EXPECT_EQ(witness.values.rotation_fixed, 0u); EXPECT_EQ(witness.values.rotation_present, 0u);
  for (double value : {witness.values.minimum_dt_s, witness.values.factor,
      witness.values.mass_kg, witness.values.inertia_kg_m2,
      witness.values.translation_stiffness_n_per_m, witness.values.rotation_stiffness_nm,
      witness.values.principal_inertia_kg_m2.x, witness.values.principal_inertia_kg_m2.y,
      witness.values.principal_inertia_kg_m2.z, witness.values.trace_upper_per_s2})
    EXPECT_EQ(Bits(value), Bits(0.));
}
} // namespace tl::fea::reset_test
