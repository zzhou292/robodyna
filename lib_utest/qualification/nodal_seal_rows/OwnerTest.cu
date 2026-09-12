#include "../nodal_vehicle/VehicleOwnerFixture.h"
#include "Compare.h"
#include "FrozenStability.h"
#include "lib_src/solvers/FENodalStateStorage.h"
#include "lib_src/solvers/NodalStateLayout.h"
#include "lib_src/solvers/nodal_seal/RowLayout.h"
#include <math_constants.h>

namespace tl::fea::vehicle_test {
namespace {
__global__ void Bounds(NodalAssemblyView view, unsigned fault) {
  const auto n = view.accepted.node_count;
  view.bounds->stiffness[3] = view.bounds->stiffness[n - 1] = 1e12;
  view.bounds->damping[7] = 2;
  if (fault == 1) view.bounds->damping[n - 1] = CUDART_NAN;
  if (fault == 2) {
    view.bounds->damping[n - 1] = -1;
    view.forces.force_y[257] = CUDART_NAN;
  }
  if (fault == 3) {
    view.result->status = tlfea::contact::Status::kInvalidArgument;
    view.result->node = 11;
    view.bounds->damping[n - 1] = CUDART_NAN;
  }
}
}
TEST_F(VehicleOwnerCuda, ReducedSealRejectsAtomicallyAndRetriesWithoutAllocation) {
  constexpr std::size_t n = 513;
  Initial initial(n); FENodalState owner;
  const auto config = initial.config();
  ASSERT_EQ(initial.Initialize(owner, config).status, NodalStatus::Ok);
  const auto allocation = owner.allocations();
  Fields accepted(n); NodalStamp stamp;
  ASSERT_EQ(owner.CopyAccepted(accepted.buffer(), &stamp).status, NodalStatus::Ok);
  for (unsigned fault = 0; fault != 4; ++fault) {
    SCOPED_TRACE(fault);
    NodalTrialToken token; NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token, &view).status, NodalStatus::Ok);
    Bounds<<<1,1,0,view.stream>>>(view, fault);
    const auto result = owner.SealAssembly(token);
    if (!fault) {
      std::vector<double> k(n), c(n); k[3] = k[n - 1] = 1e12; c[7] = 2;
      seal_frozen_stability::RowBounds rows{k.data(), c.data(), n, n, 0, 1, true, true, false};
      seal_frozen_stability::StepLimit limit;
      ASSERT_EQ(seal_frozen_stability::FinalizeRows(&rows, config.timestep_safety,
          config.minimum_dt, config.fixed_dt, &limit), tlfea::contact::Status::kOk);
      EXPECT_EQ(result.status, NodalStatus::StepTooLarge);
      EXPECT_EQ(result.node, 3u);
      EXPECT_EQ(seal_test::Bits(result.stable_dt), seal_test::Bits(limit.dt));
    } else {
      EXPECT_EQ(result.status, fault == 3 ? NodalStatus::ContributorFailure : NodalStatus::InvalidOutput);
      EXPECT_EQ(result.node, fault == 3 ? 11u : (fault == 2 ? 257u : UINT32_MAX));
    }
    owner.Discard();
    Fields rejected(n);
    ASSERT_EQ(owner.CopyAccepted(rejected.buffer(), &stamp).status, NodalStatus::Ok);
    SameFields(accepted, rejected); EXPECT_EQ(stamp.epoch, 0u);
    NodalTrialToken retry;
    ASSERT_EQ(Prepare(owner, retry).status, NodalStatus::Ok);
    owner.Discard();
    EXPECT_EQ(owner.allocations().device_bytes, allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations, allocation.device_allocations);
  }
  NodalTrialToken accepted_trial;
  ASSERT_EQ(Prepare(owner, accepted_trial).status, NodalStatus::Ok);
  ASSERT_EQ(owner.Commit(accepted_trial).status, NodalStatus::Ok);
  EXPECT_EQ(owner.accepted().epoch, 1u);
}
TEST_F(VehicleOwnerCuda, ExactControlTailForecastAdmitsOnlyCompleteAllocation) {
  constexpr std::size_t n = 513;
  Initial initial(n);
  auto config = initial.config();
  nodal_detail::StateLayout layout;
  const auto control_bytes = nodal_seal::ControlBytes(sizeof(nodal_detail::Control), n);
  ASSERT_TRUE(layout.Initialize(n, true, 0, 0, 0, control_bytes, config.max_device_bytes));
  config.max_device_bytes = layout.bytes - 1;
  FENodalState short_owner;
  EXPECT_EQ(initial.Initialize(short_owner, config).status, NodalStatus::ResourceLimit);
  EXPECT_EQ(short_owner.allocations().device_allocations, 0u);
  config.max_device_bytes = layout.bytes;
  FENodalState exact;
  ASSERT_EQ(initial.Initialize(exact, config).status, NodalStatus::Ok);
  EXPECT_EQ(exact.allocations().device_bytes, layout.bytes);
  EXPECT_EQ(exact.allocations().device_allocations, 6u);
  NodalTrialToken token;
  ASSERT_EQ(Prepare(exact, token).status, NodalStatus::Ok);
  ASSERT_EQ(exact.Commit(token).status, NodalStatus::Ok);
}
} // namespace tl::fea::vehicle_test
