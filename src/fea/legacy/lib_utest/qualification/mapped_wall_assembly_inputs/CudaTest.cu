// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/collision/nodal_wall_mapped/AssemblyValidation.cuh"
#include <memory>
#include <stdexcept>
namespace wall_assembly_test {
namespace {
void Check(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
struct Delete {
  void operator()(void* pointer) const noexcept { cudaFree(pointer); }
};
__global__ void Frozen(d::Storage* storage, m::Sidecar side, fe::NodalAssemblyView view) {
  storage->control = {};
  *side.summary = {};
  side.summary->points_admitted = wall_assembly_frozen::ValidateAssembly(*storage, side, view);
}
class Device {
 public:
  explicit Device(Packet& p) : packet_(p), view_(p.View()), side_(p.Side()) {
    auto storage = p.storage;
    storage.model.nodes = Copy(p.nodes);
    storage.model.initial_position = p.storage.model.initial_position ? Copy(p.initial) : nullptr;
    storage_ = static_cast<d::Storage*>(Upload(&storage, sizeof(storage)));
    view_.accepted.position_xyz = Copy(p.position);
    view_.mass.inverse_mass = Copy(p.inverse);
    view_.mass.fixed = Copy(p.fixed);
    view_.translation_fixed_bits = Copy(p.translation);
    view_.result = static_cast<fe::NodalAssemblyResult*>(Upload(&p.result, sizeof(p.result)));
    view_.bounds = static_cast<fe::stability::RowBounds*>(Upload(&p.bounds, sizeof(p.bounds)));
    side_.roots = Copy(p.roots);
    side_.inverse = Copy(p.copied);
    side_.summary = static_cast<m::Summary*>(Upload(&p.summary, sizeof(p.summary)));
  }
  void RestoreInputs() {
    const auto copy = [](const void* target, const void* source, std::size_t bytes) {
      Check(cudaMemcpy(const_cast<void*>(target), source, bytes, cudaMemcpyHostToDevice));
    };
    copy(view_.accepted.position_xyz, packet_.position.data(), packet_.position.size()*sizeof(double));
    copy(view_.mass.inverse_mass, packet_.inverse.data(), packet_.inverse.size()*sizeof(double));
    copy(view_.mass.fixed, packet_.fixed.data(), packet_.fixed.size());
    copy(view_.translation_fixed_bits, packet_.translation.data(), packet_.translation.size());
    copy(side_.roots, packet_.roots.data(), packet_.roots.size()*sizeof(std::uint32_t));
    copy(view_.result, &packet_.result, sizeof(packet_.result));
    copy(view_.bounds, &packet_.bounds, sizeof(packet_.bounds));
  }
  void Run(bool frozen) {
    if (frozen) Frozen<<<1, 1>>>(storage_, side_, view_);
    else a::Launch(storage_, side_, view_, packet_.count, nullptr);
    Check(cudaGetLastError());
    Check(cudaDeviceSynchronize());
    const auto* control = reinterpret_cast<const unsigned char*>(storage_)+offsetof(d::Storage, control);
    Check(cudaMemcpy(&packet_.storage.control, control, sizeof(packet_.storage.control), cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(&packet_.summary, side_.summary, sizeof(packet_.summary), cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(packet_.copied.data(), side_.inverse, packet_.copied.size()*sizeof(double), cudaMemcpyDeviceToHost));
  }
 private:
  void* Upload(const void* source, std::size_t bytes) {
    if (!bytes) return nullptr;
    void* pointer = nullptr;
    Check(cudaMalloc(&pointer, bytes));
    allocations_.emplace_back(pointer);
    Check(cudaMemcpy(pointer, source, bytes, cudaMemcpyHostToDevice));
    return pointer;
  }
  template<class T> T* Copy(const std::vector<T>& source) {
    return static_cast<T*>(Upload(source.data(), source.size()*sizeof(T)));
  }
  Packet& packet_;
  fe::NodalAssemblyView view_;
  m::Sidecar side_;
  d::Storage* storage_ = nullptr;
  std::vector<std::unique_ptr<void, Delete>> allocations_;
};
}
TEST(MappedWallAssemblyInputsCuda, CompleteFrozenValidationMatchesAcrossBoundedGridAndRoleBranches) {
  for (unsigned count : {7u, 127u, 128u, 129u, 263u, 33001u}) {
    for (unsigned epoch : {0u, 3u}) {
      SCOPED_TRACE(count);
      SCOPED_TRACE(epoch);
      Packet serial(count), parallel(count);
      for (auto* p : {&serial, &parallel}) {
        p->epoch = epoch;
        p->Reset();
        if (epoch) {
          std::fill(p->position.begin(), p->position.end(), NAN);
          p->storage.model.initial_position = nullptr;
        }
        p->summary.parent_failure = 0; // Every attempt must reseed scratch.
      }
      Device a(parallel), b(serial);
      a.Run(false);
      b.Run(true);
      ASSERT_TRUE(parallel.summary.points_admitted);
      Same(parallel, serial);
      EXPECT_EQ(parallel.summary.rate, 0);
      EXPECT_EQ(parallel.summary.removed_potential.value, 0);
    }
  }
}
TEST(MappedWallAssemblyInputsCuda, FullErrorPriorityAndExactPartialInverseStateThenRetry) {
  for (unsigned fault = 0; fault < 12; ++fault) {
    SCOPED_TRACE(fault);
    Packet serial, parallel;
    Fault(serial, fault);
    Fault(parallel, fault);
    Device a(parallel), b(serial);
    a.Run(false);
    b.Run(true);
    ASSERT_FALSE(parallel.summary.points_admitted);
    Same(parallel, serial);
    // Repair the same source packet without replacing either device allocation.
    serial.Reset();
    parallel.Reset();
    a.RestoreInputs();
    b.RestoreInputs();
    a.Run(false);
    b.Run(true);
    ASSERT_TRUE(parallel.summary.points_admitted);
    Same(parallel, serial);
    a.Run(false);
    b.Run(true);
    Same(parallel, serial);
  }
}
} // namespace wall_assembly_test
