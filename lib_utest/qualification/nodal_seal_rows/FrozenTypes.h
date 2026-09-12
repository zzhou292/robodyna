#pragma once
#include "FrozenStability.h"
#include "lib_src/solvers/FENodalStateStorage.h"
#include <cstddef>
#include <type_traits>

namespace tl::fea::seal_test {
// Control itself is unchanged in production. These original field types let
// the complete frozen caller use its complete frozen stability implementation.
struct FrozenControl {
  seal_frozen_stability::RowBounds rows;
  NodalAssemblyResult assembly;
  seal_frozen_stability::StepLimit limit;
  NodalStatus status = NodalStatus::Ok;
  std::uint32_t node = UINT32_MAX;
};
static_assert(std::is_trivially_copyable_v<FrozenControl>);
static_assert(std::is_trivially_copyable_v<nodal_detail::Control>);
static_assert(sizeof(FrozenControl) == sizeof(nodal_detail::Control));
static_assert(sizeof(seal_frozen_stability::RowBounds) == sizeof(stability::RowBounds));
static_assert(sizeof(seal_frozen_stability::StepLimit) == sizeof(stability::StepLimit));
static_assert(offsetof(FrozenControl, assembly) == offsetof(nodal_detail::Control, assembly));
static_assert(offsetof(FrozenControl, limit) == offsetof(nodal_detail::Control, limit));
static_assert(offsetof(FrozenControl, status) == offsetof(nodal_detail::Control, status));
static_assert(offsetof(FrozenControl, node) == offsetof(nodal_detail::Control, node));
void FrozenLaunch(nodal_detail::Control*, const double*, std::uint32_t,
    double, double, double, bool, cudaStream_t);
} // namespace tl::fea::seal_test
