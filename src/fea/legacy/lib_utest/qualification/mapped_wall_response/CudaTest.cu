// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/collision/nodal_wall_mapped/Response.cuh"
#include <memory>
#include <stdexcept>
namespace wall_response_test {
namespace {
void Check(cudaError_t value) {
  if (value != cudaSuccess) throw std::runtime_error(cudaGetErrorString(value));
}
struct Delete { void operator()(void* p) const noexcept { cudaFree(p); } };
__global__ void Frozen(d::Storage* storage, m::Sidecar side, fe::NodalAssemblyView view) {
  wall_response_frozen::CheckResponse(storage, side, view);
}
class Device {
 public:
  explicit Device(Packet& p) : packet_(p), side_(p.Side()), view_(p.View()) {
    nodes_ = Copy(p.nodes);
    side_.bodies = Copy(p.bodies);
    side_.roots = Copy(p.roots);
    side_.inverse = Copy(p.inverse);
    side_.traces = Copy(p.traces);
    side_.summary = static_cast<m::Summary*>(Upload(&p.summary, sizeof(p.summary)));
    side_.response.offsets = Copy(p.offsets);
    side_.response.rows = Copy(p.rows);
    side_.response.maxima = Copy(p.maxima);
    view_.accepted.position_xyz = Copy(p.position);
    storage_ = static_cast<d::Storage*>(Upload(&p.storage, sizeof(p.storage)));
    Restore();
  }
  void Restore() {
    auto source = packet_.storage;
    source.result.nodes = packet_.storage.result.nodes ? nodes_ : nullptr;
    Put(storage_, &source, sizeof(source));
    Put(side_.summary, &packet_.summary, sizeof(packet_.summary));
    Put(nodes_, packet_.nodes);
    Put(side_.bodies, packet_.bodies);
    Put(side_.roots, packet_.roots);
    Put(side_.inverse, packet_.inverse);
    Put(side_.traces, packet_.traces);
    Put(side_.response.offsets, packet_.offsets);
    Put(side_.response.rows, packet_.rows);
    Put(side_.response.maxima, packet_.maxima);
    Put(view_.accepted.position_xyz, packet_.position);
  }
  void Run(bool frozen) {
    if (frozen) Frozen<<<1, 1>>>(storage_, side_, view_);
    else r::Launch(storage_, side_, view_, nullptr);
    Check(cudaGetLastError());
    Check(cudaDeviceSynchronize());
    d::Storage result;
    Check(cudaMemcpy(&result, storage_, sizeof(result), cudaMemcpyDeviceToHost));
    packet_.storage.control = result.control;
    packet_.storage.result.diagnostics = result.result.diagnostics;
    Check(cudaMemcpy(&packet_.summary, side_.summary, sizeof(packet_.summary), cudaMemcpyDeviceToHost));
    if (packet_.groups)
      Check(cudaMemcpy(packet_.traces.data(), side_.traces, packet_.traces.size()*sizeof(double), cudaMemcpyDeviceToHost));
  }
 private:
  static void Put(const void* to, const void* from, std::size_t bytes) {
    if (bytes) Check(cudaMemcpy(const_cast<void*>(to), from, bytes, cudaMemcpyHostToDevice));
  }
  template<class T> static void Put(const T* to, const std::vector<T>& from) {
    Put(to, from.data(), from.size()*sizeof(T));
  }
  void* Upload(const void* source, std::size_t bytes) {
    if (!bytes) return nullptr;
    void* result = nullptr;
    Check(cudaMalloc(&result, bytes));
    allocations_.emplace_back(result);
    Put(result, source, bytes);
    return result;
  }
  template<class T> T* Copy(const std::vector<T>& input) {
    return static_cast<T*>(Upload(input.data(), input.size()*sizeof(T)));
  }
  Packet& packet_;
  m::Sidecar side_;
  fe::NodalAssemblyView view_;
  c::NodalWallPointResult* nodes_ = nullptr;
  d::Storage* storage_ = nullptr;
  std::vector<std::unique_ptr<void, Delete>> allocations_;
};
}
TEST(MappedWallResponseCuda, CompleteFrozenResponseMatchesOrderedTracesAndMaxima) {
  for (unsigned count : {7u, 127u, 128u, 129u, 263u, 33001u}) {
    for (unsigned groups : {0u, 7u, 1024u}) {
      SCOPED_TRACE(count);
      SCOPED_TRACE(groups);
      Packet serial(count, groups), parallel(count, groups);
      Device a(parallel), b(serial);
      a.Run(false); b.Run(true);
      ASSERT_EQ(parallel.storage.control.status, c::NodalWallDeviceStatus::Ok);
      Same(parallel, serial);
    }
  }
}
TEST(MappedWallResponseCuda, ExactFirstErrorPartialDiagnosticsThenSameAllocationRetry) {
  for (unsigned fault = 0; fault < 13; ++fault) {
    SCOPED_TRACE(fault);
    Packet serial, parallel;
    Fault(serial, fault); Fault(parallel, fault);
    Device a(parallel), b(serial);
    a.Run(false); b.Run(true);
    ASSERT_NE(parallel.storage.control.status, c::NodalWallDeviceStatus::Ok);
    Same(parallel, serial);
    serial.Reset(); parallel.Reset();
    a.Restore(); b.Restore();
    a.Run(false); b.Run(true);
    ASSERT_EQ(parallel.storage.control.status, c::NodalWallDeviceStatus::Ok);
    Same(parallel, serial);
    a.Run(false); b.Run(true);
    Same(parallel, serial);
  }
}
TEST(MappedWallResponseCuda, ZeroConsumedInputsAndPriorRejectedStageLeaveSeededOutputs) {
  for (bool prior_error : {false, true}) {
    Packet serial, parallel;
    for (auto* p : {&serial, &parallel}) {
      for (auto& node : p->nodes) node.stiffness.upper = node.stiffness.value = -0.;
      for (auto& body : p->bodies) body.mass = NAN;
      std::fill(p->roots.begin(), p->roots.end(), UINT32_MAX-1);
      std::fill(p->inverse.begin(), p->inverse.end(), NAN);
      std::fill(p->position.begin(), p->position.end(), NAN);
      p->storage.model.config.owner.fixed_dt = -0.;
      if (prior_error) {
        p->storage.control.status = c::NodalWallDeviceStatus::GeometryFailure;
        p->storage.result.nodes = nullptr;
        p->summary.parent_failure = 79;
      }
    }
    Device a(parallel), b(serial);
    a.Run(false); b.Run(true);
    Same(parallel, serial);
    EXPECT_EQ(parallel.storage.control.status, prior_error ? c::NodalWallDeviceStatus::GeometryFailure
                                                        : c::NodalWallDeviceStatus::Ok);
  }
}
} // namespace wall_response_test
