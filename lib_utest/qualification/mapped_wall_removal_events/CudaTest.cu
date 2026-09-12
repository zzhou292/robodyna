#include "Fixture.h"
#include "lib_src/collision/nodal_wall_mapped/RemovalEvents.cuh"
#include <cstdio>
#include <memory>
#include <stdexcept>

namespace wall_removal_test {
namespace fe = tl::fea;
namespace {
void Check(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
__global__ void Current(d::Storage* pointer, m::Sidecar side, fe::NodalPreparedView view) {
  auto& storage=*pointer;
  if(m::removal_events::RemovedPotential(storage,side) && !threadIdx.x) {
    storage.result.diagnostics.stiffness_rate_bound=side.summary->rate;
    storage.result.diagnostics.valid=true;
  }
}
__global__ void Frozen(d::Storage* pointer, m::Sidecar side, fe::NodalPreparedView view) {
  wall_removal_frozen::FinishCandidate(pointer, side, view);
}
struct Delete { void operator()(void* p) const noexcept { cudaFree(p); } };
class Device {
 public:
  explicit Device(Packet& p) : p_(p), side_(p.Side()) {
    Check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
    side_.accepted = static_cast<std::uint8_t*>(Allocate(p.accepted.size()));
    side_.proposed = static_cast<std::uint8_t*>(Allocate(p.proposed.size()));
    parents_ = static_cast<c::NodalWallParentResult*>(Allocate(p.parents.size()*sizeof(*parents_)));
    side_.summary = static_cast<m::Summary*>(Allocate(sizeof(p.summary)));
    storage_ = static_cast<d::Storage*>(Allocate(sizeof(p.storage)));
    Restore();
  }
  ~Device() { cudaStreamDestroy(stream_); }
  void Restore() {
    auto storage = p_.storage;
    storage.result.parents = p_.storage.result.parents ? parents_ : nullptr;
    Put(storage_, &storage, sizeof(storage));
    Put(side_.summary, &p_.summary, sizeof(p_.summary));
    Put(side_.accepted, p_.accepted.data(), p_.accepted.size());
    Put(side_.proposed, p_.proposed.data(), p_.proposed.size());
    Put(parents_, p_.parents.data(), p_.parents.size()*sizeof(*parents_));
    Check(cudaStreamSynchronize(stream_));
  }
  void Launch(bool frozen) {
    if (frozen) Frozen<<<1, 1, 0, stream_>>>(storage_, side_, {});
    else Current<<<1, r::Threads, 0, stream_>>>(storage_, side_, {});
    Check(cudaGetLastError());
  }
  void Read() {
    d::Storage storage;
    Check(cudaMemcpyAsync(&storage, storage_, sizeof(storage), cudaMemcpyDeviceToHost, stream_));
    Check(cudaMemcpyAsync(&p_.summary, side_.summary, sizeof(p_.summary), cudaMemcpyDeviceToHost, stream_));
    Check(cudaStreamSynchronize(stream_));
    p_.storage.control = storage.control;
    p_.storage.result.diagnostics = storage.result.diagnostics;
  }
  float Time(bool frozen, unsigned count) {
    cudaEvent_t begin, end;
    Check(cudaEventCreate(&begin)); Check(cudaEventCreate(&end));
    Check(cudaEventRecord(begin, stream_));
    for (unsigned i = 0; i < count; ++i) Launch(frozen);
    Check(cudaEventRecord(end, stream_));
    Check(cudaEventSynchronize(end));
    float milliseconds = 0;
    Check(cudaEventElapsedTime(&milliseconds, begin, end));
    Check(cudaEventDestroy(begin)); Check(cudaEventDestroy(end));
    return milliseconds;
  }
  void CheckInputs() {
    std::vector<c::NodalWallParentResult> parents(p_.parents.size());
    std::vector<std::uint8_t> accepted(p_.accepted.size()), proposed(p_.proposed.size());
    if (!parents.empty()) {
      Check(cudaMemcpyAsync(parents.data(), parents_, parents.size()*sizeof(*parents_), cudaMemcpyDeviceToHost, stream_));
      Check(cudaMemcpyAsync(accepted.data(), side_.accepted, accepted.size(), cudaMemcpyDeviceToHost, stream_));
      Check(cudaMemcpyAsync(proposed.data(), side_.proposed, proposed.size(), cudaMemcpyDeviceToHost, stream_));
      Check(cudaStreamSynchronize(stream_));
      EXPECT_EQ(std::memcmp(parents.data(), p_.parents.data(), parents.size()*sizeof(*parents_)), 0);
    }
    EXPECT_EQ(accepted, p_.accepted);
    EXPECT_EQ(proposed, p_.proposed);
  }
 private:
  void* Allocate(std::size_t bytes) {
    if (!bytes) return nullptr;
    void* value = nullptr;
    Check(cudaMalloc(&value, bytes));
    owned_.emplace_back(value);
    return value;
  }
  void Put(void* target, const void* source, std::size_t bytes) {
    if (bytes) Check(cudaMemcpyAsync(target, source, bytes, cudaMemcpyHostToDevice, stream_));
  }
  Packet& p_;
  m::Sidecar side_;
  d::Storage* storage_ = nullptr;
  c::NodalWallParentResult* parents_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::vector<std::unique_ptr<void, Delete>> owned_;
};
}
TEST(WallRemovalCuda, FrozenWholeCallerAtBoundariesAndFullAdmittedCap) {
  for (unsigned n : {0u, 1u, 31u, 32u, 33u, 255u, 256u, 257u, 777u, 524288u}) {
    Packet a(n), b(n);
    Device x(a), y(b);
    for (unsigned pattern = 0; pattern < 4; ++pattern) {
      SCOPED_TRACE(n);
      SCOPED_TRACE(pattern);
      a.Reset(pattern); b.Reset(pattern);
      x.Restore(); y.Restore();
      x.Launch(true); y.Launch(false);
      x.Read(); y.Read(); Same(a, b);
      ASSERT_TRUE(b.storage.result.diagnostics.valid);
      x.CheckInputs(); y.CheckInputs();
    }
  }
}
TEST(WallRemovalCuda, InvalidEncodingsAndEveryFailurePrefixThenSameAllocationRetry) {
  Packet a, b;
  Device x(a), y(b);
  for (unsigned fault = 0; fault < 13; ++fault) {
    SCOPED_TRACE(fault);
    Fault(a, fault); Fault(b, fault);
    x.Restore(); y.Restore();
    x.Launch(true); y.Launch(false);
    x.Read(); y.Read(); Same(a, b);
    x.CheckInputs(); y.CheckInputs();
    a.Reset(); b.Reset();
    x.Restore(); y.Restore();
    x.Launch(true); y.Launch(false);
    x.Read(); y.Read(); Same(a, b);
    ASSERT_TRUE(b.storage.result.diagnostics.valid);
  }
  for (unsigned raw : {2u, 127u, 128u, 254u, 255u}) {
    a.Reset(1); b.Reset(1);
    a.accepted[32] = b.accepted[32] = raw;
    a.parents[32].potential = b.parents[32].potential = {NAN, NAN, NAN, NAN};
    x.Restore(); y.Restore();
    x.Launch(true); y.Launch(false);
    x.Read(); y.Read(); Same(a, b);
    EXPECT_EQ(b.storage.control.parent, 32u);
  }
}
TEST(WallRemovalCuda, SparseAndDenseSyntheticTimingWithExactPackets) {
  constexpr unsigned count = 131072, repeats = 8;
  Packet a(count), b(count);
  Device x(a), y(b);
  for (unsigned pattern = 1; pattern < 4; ++pattern) {
    a.Reset(pattern); b.Reset(pattern);
    x.Restore(); y.Restore();
    x.Launch(true); y.Launch(false); // Warm both complete callers.
    x.Read(); y.Read(); Same(a, b);
    const float serial = x.Time(true, repeats);
    const float tiled = y.Time(false, repeats);
    x.Read(); y.Read(); Same(a, b);
    std::printf("WALL_REMOVAL_TIMING {\"parents\":%u,\"pattern\":%u,\"calls\":%u,"
        "\"serial_total_ms\":%.9g,\"tiled_total_ms\":%.9g}\n",
        count, pattern, repeats, serial, tiled);
  }
}
} // namespace wall_removal_test
