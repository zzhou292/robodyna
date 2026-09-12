#include "../mapped_wall_evaluation/Fixture.h"
#include "FrozenEvaluation.cuh"

namespace wall_evaluation_test {
void EvaluateStatus(Fixture& f, bool frozen, cudaStream_t stream, bool reset_base,
    m::ObserverScratch observers = {}) {
  if (frozen)
    m::status_frozen::Evaluate(f.storage, f.Side(), f.Kinematics(), f.Identity(),
        Nodes, Parents, stream, reset_base, observers);
  else
    m::parallel::Evaluate(f.storage, f.Side(), f.Kinematics(), f.Identity(),
        Nodes, Parents, stream, reset_base, observers);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
}
void SameStatus(const Fixture& current, const Fixture& frozen) {
  EXPECT_EQ(current.storage->control.status, frozen.storage->control.status);
  EXPECT_EQ(current.storage->control.node, frozen.storage->control.node);
  EXPECT_EQ(current.storage->control.parent, frozen.storage->control.parent);
  EXPECT_EQ(current.storage->control.point.status, frozen.storage->control.point.status);
  EXPECT_EQ(current.storage->control.point.cause, frozen.storage->control.point.cause);
  EXPECT_EQ(nodal_wall_owner_test::Bytes(current.storage->result.diagnostics),
      nodal_wall_owner_test::Bytes(frozen.storage->result.diagnostics));
}

TEST(WallNodeStatusCuda, CompleteEvaluationMatchesFrozenWithObserversAndBaseReset) {
  Fixture current, frozen;
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
  m::ObserverSummary* scratch = nullptr;
  const auto count = m::ObserverBlocks(Nodes);
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&scratch),
      2 * count * sizeof(m::ObserverSummary)), cudaSuccess);
  for (bool tree : {false, true}) {
    for (unsigned mask = 0; mask < 3; ++mask) {
      SCOPED_TRACE(tree);
      SCOPED_TRACE(mask);
      for (auto* f : {&current, &frozen}) {
        f->Restore();
        for (unsigned p = 0; p < Parents; ++p)
          f->input->activity[p] = mask == 0 ? 1 : mask == 1 ? 0 : (p % 3 != 0);
        f->storage->base.diagnostics.valid = true;
        for (unsigned n = 0; n < Nodes; ++n) {
          f->storage->base.nodes[n].force.value = 77;
          if (n % 7 == 0) f->input->x[3 * n] = -.001;
          if (n % 11 == 0) f->input->x[3 * n] = -0.;
          f->input->velocity[3 * n + 1] = (int(n % 3) - 1) * .003;
        }
      }
      current.input->summary.parent_failure = 17;
      const m::ObserverScratch a{tree ? scratch : nullptr, tree ? count : 0};
      const m::ObserverScratch b{tree ? scratch + count : nullptr, tree ? count : 0};
      EvaluateStatus(current, false, stream, true, a);
      EvaluateStatus(frozen, true, stream, true, b);
      ASSERT_EQ(current.storage->control.status, c::NodalWallDeviceStatus::Ok);
      SameStatus(current, frozen);
      v::SameResults(current.Read(), frozen.Read());
      v::SameResults(current.Read(current.storage->base), frozen.Read(frozen.storage->base));
      EXPECT_EQ(current.input->summary.parent_failure, frozen.input->summary.parent_failure);
    }
  }
  EXPECT_EQ(cudaFree(scratch), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}

TEST(WallNodeStatusCuda, PointAndParentPriorityWithRepeatedAttemptAndAdmissionFailure) {
  Fixture current, frozen;
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
  for (unsigned fault = 0; fault < 7; ++fault) {
    SCOPED_TRACE(fault);
    for (auto* f : {&current, &frozen}) {
      f->Restore();
      if (fault == 0 || fault == 1)
        f->input->velocity[3 * (Nodes - 1)] = std::numeric_limits<double>::quiet_NaN();
      if (fault == 1) f->input->x[3 * 2 + 1] = 3;
      if (fault == 2) f->input->velocity[0] = std::numeric_limits<double>::quiet_NaN();
      if (fault == 3)
        f->storage->model.config.law.parent_force_error = std::numeric_limits<double>::min();
      if (fault == 4) f->storage->model.config.law.maximum_penetration = 1e-5;
      if (fault == 5 || fault == 6) {
        f->storage->control.status = c::NodalWallDeviceStatus::InvalidMass;
        f->storage->control.node = Nodes - 1;
        f->input->summary.points_admitted = fault == 6;
        f->input->velocity[0] = std::numeric_limits<double>::quiet_NaN();
      }
      f->storage->base.diagnostics.valid = true;
      f->storage->base.nodes[Nodes - 1].force.value = 123;
    }
    EvaluateStatus(current, false, stream, true);
    EvaluateStatus(frozen, true, stream, true);
    ASSERT_NE(current.storage->control.status, c::NodalWallDeviceStatus::Ok);
    SameStatus(current, frozen);
    v::SameResults(current.Read(current.storage->base), frozen.Read(frozen.storage->base));
    current.Restore(); frozen.Restore();
    current.input->summary.parent_failure = Nodes + 100;
    EvaluateStatus(current, false, stream, false);
    EvaluateStatus(frozen, true, stream, false);
    ASSERT_EQ(current.storage->control.status, c::NodalWallDeviceStatus::Ok);
    SameStatus(current, frozen);
    v::SameResults(current.Read(), frozen.Read());
  }
  EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}
} // namespace wall_evaluation_test
