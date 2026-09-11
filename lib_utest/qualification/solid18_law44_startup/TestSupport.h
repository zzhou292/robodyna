// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/law44/ForceScratch.h"
#include "lib_utest/qualification/solid18_law44_force/NativeOracle.h"
#include "lib_utest/qualification/solid18_law44_force/Trajectory.h"
#include <memory>

namespace rear_startup_test {
using namespace rear_force_test;
template<class T, std::size_t N>
inline void ExactArray(const std::array<T,N>& a, const std::array<T,N>& b) {
  EXPECT_EQ(std::memcmp(a.data(),b.data(),sizeof(T)*N),0);
}
inline void Exact(const law::ForceTrial& a, const law::ForceTrial& b) {
  const auto x = Values(a), y = Values(b);
  ExactArray(x.next.point,y.next.point);
  ExactArray(x.next.global,y.next.global);
  ExactArray(x.next.saved,y.next.saved);
  ExactArray(x.next.cursor,y.next.cursor);
  ExactArray(x.observation,y.observation);
  ExactArray(x.force,y.force);
  ExactArray(x.geometry,y.geometry);
  ExactArray(x.diagnostics,y.diagnostics);
  EXPECT_EQ(a.proposed_history.stamp().sample_index,b.proposed_history.stamp().sample_index);
  EXPECT_EQ(a.proposed_history.stamp().time_s,b.proposed_history.stamp().time_s);
  EXPECT_TRUE(law::detail::SameReference(a.proposed_history.reference(),b.proposed_history.reference()));
  EXPECT_TRUE(law::detail::SameMaterial(a.proposed_history.material(),b.proposed_history.material()));
}
inline void DirtyExcluded(law::detail::ForceScratch& scratch) {
  for (auto& point : scratch.trial.geometry.point) {
    for (auto& axis : point.shear_per_m) for (double& x : axis) x = 17;
    for (auto& axis : point.cross_per_m) for (double& x : axis) x = -23;
  }
}
}  // namespace rear_startup_test
