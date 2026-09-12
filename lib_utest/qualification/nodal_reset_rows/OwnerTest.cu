#include "ControlChecks.h"
#include "../nodal_vehicle/VehicleOwnerFixture.h"
#include "lib_src/solvers/NodalStateLayout.h"
#include "lib_src/solvers/nodal_seal/RowLayout.h"
#include <cstddef>

namespace tl::fea::vehicle_test {
namespace {
static_assert(offsetof(nodal_detail::Control, rows) == 0);
__global__ void Dirty(NodalAssemblyView view, bool reject) {
  auto* control = reinterpret_cast<nodal_detail::Control*>(view.bounds);
  control->assembly.status = tlfea::contact::Status::kOutOfRange;
  control->limit.dt = 31; control->node = 41;
  control->structural_limiter.epoch = 51;
  control->structural_limiter.attempt = 61;
  control->structural_limiter.values.kind = NodalCinLimitKind::RigidTrace;
  control->structural_limiter.values.mass_kg = 71;
  if (reject) control->rows.attempt = view.attempt + 1;
  for (std::uint32_t i = 0; i < view.bounds->node_count; ++i) {
    view.bounds->stiffness[i] = 17;
    view.bounds->damping[i] = -19;
  }
}
__global__ void Repair(NodalAssemblyView view) { view.bounds->attempt = 0; }
nodal_detail::Control ReadControl(const NodalAssemblyView& view) {
  nodal_detail::Control result;
  EXPECT_EQ(cudaMemcpyAsync(&result, view.bounds, sizeof(result),
                           cudaMemcpyDeviceToHost, view.stream), cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(view.stream), cudaSuccess);
  return result;
}
}
TEST_F(VehicleOwnerCuda, ResetRejectionAndRetryPreserveAcceptedStateAndClearWitness) {
  constexpr std::size_t n = 513;
  Initial initial(n); FENodalState owner;
  ASSERT_EQ(initial.Initialize(owner, initial.config()).status, NodalStatus::Ok);
  const auto allocation = owner.allocations();
  Fields accepted(n), after(n); NodalStamp before, next;
  ASSERT_EQ(owner.CopyAccepted(accepted.buffer(), &before).status, NodalStatus::Ok);
  for (bool reject : {false, true}) {
    NodalTrialToken token; NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token, &view).status, NodalStatus::Ok);
    Dirty<<<1,1,0,view.stream>>>(view, reject);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(view.stream), cudaSuccess);
    owner.Discard();
    NodalTrialToken retry; NodalAssemblyView fresh;
    const auto result = owner.BeginTrial(&retry, &fresh);
    EXPECT_EQ(result.status, reject ? NodalStatus::InvalidOutput : NodalStatus::Ok);
    const auto reset = ReadControl(view);
    reset_test::ClearedControl(reset, before.epoch, view.attempt + 1, result.status);
    if (reject) {
      EXPECT_EQ(fresh.bounds, nullptr); EXPECT_EQ(fresh.attempt, 0u);
      EXPECT_EQ(owner.SealAssembly(retry).status, NodalStatus::StaleTrial);
      EXPECT_FALSE(reset.rows.valid);
      Repair<<<1,1,0,view.stream>>>(view);
      ASSERT_EQ(cudaGetLastError(), cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(view.stream), cudaSuccess);
    } else {
      std::vector<double> rows(2*n, 123.);
      ASSERT_EQ(cudaMemcpyAsync(rows.data(), reset.rows.stiffness, rows.size()*sizeof(double),
                               cudaMemcpyDeviceToHost, view.stream), cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(view.stream), cudaSuccess);
      for (double value : rows) EXPECT_EQ(reset_test::Bits(value), reset_test::Bits(0.));
    }
    owner.Discard();
    ASSERT_EQ(owner.CopyAccepted(after.buffer(), &next).status, NodalStatus::Ok);
    SameFields(accepted, after); EXPECT_EQ(next.owner_id, before.owner_id);
    EXPECT_EQ(next.epoch, before.epoch); EXPECT_EQ(next.time, before.time);
    NodalTrialToken usable;
    ASSERT_EQ(Prepare(owner, usable).status, NodalStatus::Ok);
    owner.Discard();
    EXPECT_EQ(owner.allocations().device_bytes, allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations, allocation.device_allocations);
  }
  NodalTrialToken final;
  ASSERT_EQ(Prepare(owner, final).status, NodalStatus::Ok);
  ASSERT_EQ(owner.Commit(final).status, NodalStatus::Ok);
  EXPECT_EQ(owner.accepted().epoch, before.epoch + 1);
}
TEST_F(VehicleOwnerCuda, ResetUsesUnchangedExactOwnerCapacity) {
  constexpr std::size_t n = 513;
  Initial initial(n);
  auto config = initial.config();
  nodal_detail::StateLayout layout;
  ASSERT_TRUE(layout.Initialize(n, true, 0, 0, 0,
      nodal_seal::ControlBytes(sizeof(nodal_detail::Control), n), config.max_device_bytes));
  config.max_device_bytes = layout.bytes - 1;
  FENodalState short_owner;
  EXPECT_EQ(initial.Initialize(short_owner, config).status, NodalStatus::ResourceLimit);
  EXPECT_EQ(short_owner.allocations().device_allocations, 0u);
  config.max_device_bytes = layout.bytes;
  FENodalState owner;
  ASSERT_EQ(initial.Initialize(owner, config).status, NodalStatus::Ok);
  EXPECT_EQ(owner.allocations().device_bytes, layout.bytes);
  EXPECT_EQ(owner.allocations().device_allocations, 6u);
  for (unsigned step = 0; step < 2; ++step) {
    NodalTrialToken token;
    ASSERT_EQ(Prepare(owner, token).status, NodalStatus::Ok);
    ASSERT_EQ(owner.Commit(token).status, NodalStatus::Ok);
    EXPECT_EQ(owner.allocations().device_bytes, layout.bytes);
    EXPECT_EQ(owner.allocations().device_allocations, 6u);
  }
}
} // namespace tl::fea::vehicle_test
