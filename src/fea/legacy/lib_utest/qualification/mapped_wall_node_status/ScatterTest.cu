#include "../mapped_wall_scatter/Fixture.h"
#include "FrozenScatter.cuh"
#include "lib_src/collision/nodal_wall_mapped/Scatter.cuh"

namespace wall_scatter_test {
struct StatusPacket {
  Packet values;
  m::Summary summary;
};
struct ManagedPacket {
  StatusPacket* value = nullptr;
  ManagedPacket() {
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&value), sizeof(StatusPacket)), cudaSuccess);
  }
  ~ManagedPacket() { cudaFree(value); }
};
void EvaluateScatter(StatusPacket& packet, bool frozen, cudaStream_t stream, bool summary) {
  auto& values = packet.values;
  auto side = values.Side();
  if (summary) side.summary = &packet.summary;
  if (frozen)
    m::status_frozen::Scatter(&values.storage, side, values.View(), values.Cin(), Nodes, stream);
  else
    m::parallel::Scatter(&values.storage, side, values.View(), values.Cin(), Nodes, stream);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
}

TEST(WallNodeStatusCuda, BothScatterPhasesMatchCompleteFrozenCallerAndRetry) {
  ManagedPacket current, frozen;
  ASSERT_NE(current.value, nullptr); ASSERT_NE(frozen.value, nullptr);
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
  for (bool with_summary : {false, true}) {
    for (unsigned fault = 0; fault <= 6; ++fault) {
      SCOPED_TRACE(with_summary);
      SCOPED_TRACE(fault);
      for (auto* p : {current.value, frozen.value}) {
        p->values.Reset(); p->summary = {};
        Fault(p->values, fault);
      }
      // A previous phase's integer must never survive into a new attempt.
      current.value->summary.parent_failure = 3;
      const Destinations before(current.value->values);
      EvaluateScatter(*current.value, false, stream, with_summary);
      EvaluateScatter(*frozen.value, true, stream, with_summary);
      Same(current.value->values, frozen.value->values);
      if (fault < 6) before.Unchanged(current.value->values);
      EXPECT_EQ(std::memcmp(current.value->values.staged, frozen.value->values.staged,
          sizeof(Packet::staged)), 0);
      EXPECT_EQ(std::memcmp(current.value->values.private_stiffness, frozen.value->values.private_stiffness,
          sizeof(Packet::private_stiffness)), 0);
      if (with_summary && fault != 5) EXPECT_EQ(current.value->summary.parent_failure, ~0ull);

      for (auto* p : {current.value, frozen.value}) p->values.Reset();
      EvaluateScatter(*current.value, false, stream, with_summary);
      EvaluateScatter(*frozen.value, true, stream, with_summary);
      ASSERT_EQ(current.value->values.storage.control.status, c::NodalWallDeviceStatus::Ok);
      Same(current.value->values, frozen.value->values);
    }
  }
  EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}
} // namespace wall_scatter_test
