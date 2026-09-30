#pragma once
#include "Oracle.h"
#include "../self_contact_surface/Fixture.h"
#include "lib_src/collision/SelfContactBroadphase.h"
#include <cuda_runtime.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace surface_broadphase_test {
namespace ct = tlfea::contact;
using S = ct::SelfContactBroadphaseStatus;
inline void Cuda(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
struct Stream {
  cudaStream_t value = nullptr;
  Stream() { Cuda(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking)); }
  ~Stream() { if (value) cudaStreamDestroy(value); }
};
template<class T> struct Device {
  T* data = nullptr;
  std::size_t count;
  explicit Device(std::size_t size) : count(size) { Cuda(cudaMalloc(&data, sizeof(T) * size)); }
  ~Device() { if (data) cudaFree(data); }
  void Upload(const std::vector<T>& values, cudaStream_t stream) {
    if (values.size() != count) throw std::runtime_error("Fixture upload extent");
    Cuda(cudaMemcpyAsync(data, values.data(), count * sizeof(T), cudaMemcpyHostToDevice, stream));
  }
};
inline std::vector<ct::Vec3> Positions(const ct::SelfContactSurfaceBinding& source, bool separated) {
  const auto* domain = source.physical()->domain();
  std::vector<ct::Vec3> out(domain->node_count());
  for (std::size_t n = 0; n < out.size(); ++n) {
    // Only original IDs 20..23 belong to the distinct second QEPH layer.
    const auto id = domain->nodes()[n].source_id;
    out[n] = {separated && id >= 20 && id <= 23 ? 8. : 0., 0., 0.};
  }
  return out;
}
inline std::vector<double> Pack(const std::vector<ct::Vec3>& points, bool soa) {
  std::vector<double> out(points.size() * 3);
  for (std::size_t n = 0; n < points.size(); ++n) {
    const double xyz[]{points[n].x, points[n].y, points[n].z};
    for (unsigned c = 0; c < 3; ++c) out[soa ? n + c * points.size() : 3 * n + c] = xyz[c];
  }
  return out;
}
inline ct::VectorView View(const Device<double>& device, bool soa) {
  const auto n = static_cast<std::uint32_t>(device.count / 3);
  return {device.data, n, soa ? 1u : 3u, soa ? n : 1u};
}
// Test coordinates are small exact dyadics. Their sum/difference with the
// represented thickness fits long double exactly; this oracle never reuses
// production CUDA inflation or sweep arithmetic.
inline double Directed(long double exact, bool upper) {
  double rounded = static_cast<double>(exact);
  if ((upper && static_cast<long double>(rounded) < exact) ||
      (!upper && static_cast<long double>(rounded) > exact))
    rounded = std::nextafter(rounded, upper ? INFINITY : -INFINITY);
  return rounded;
}
inline std::vector<AABB> Boxes(const ct::SelfContactSurfaceBinding& source,
    const std::vector<ct::Vec3>& current, const std::vector<ct::Vec3>* endpoint = nullptr) {
  std::vector<AABB> out;
  for (const auto& parent : source.parents()) {
    double low[3]{INFINITY, INFINITY, INFINITY}, high[3]{-INFINITY, -INFINITY, -INFINITY};
    for (unsigned local = 0; local < parent.arity; ++local) {
      const auto node = parent.arity == 3 ? parent.t3.nodes[local] : parent.q4.nodes[local];
      const auto include = [&](ct::Vec3 value) {
        const double xyz[]{value.x, value.y, value.z};
        for (unsigned c = 0; c < 3; ++c) { low[c] = std::min(low[c], xyz[c]); high[c] = std::max(high[c], xyz[c]); }
      };
      include(current[node]);
      if (endpoint) include((*endpoint)[node]);
    }
    for (unsigned c = 0; c < 3; ++c) {
      low[c] = Directed(static_cast<long double>(low[c]) - parent.reference_half_thickness_m, false);
      high[c] = Directed(static_cast<long double>(high[c]) + parent.reference_half_thickness_m, true);
    }
    out.push_back({{low[0], low[1], low[2]}, {high[0], high[1], high[2]}, static_cast<int>(out.size())});
  }
  return out;
}
inline std::vector<Key> Read(const ct::SelfContactBroadphase& broadphase, cudaStream_t stream) {
  const auto view = broadphase.pairs();
  EXPECT_TRUE(view.complete);
  std::vector<Key> out(view.count);
  if (view.count) {
    Cuda(cudaMemcpyAsync(out.data(), view.device_keys, view.count * sizeof(Key), cudaMemcpyDeviceToHost, stream));
    Cuda(cudaStreamSynchronize(stream));
  } else EXPECT_EQ(view.device_keys, nullptr);
  return out;
}
inline void Revoked(const ct::SelfContactBroadphase& broadphase) {
  EXPECT_FALSE(broadphase.pairs().complete);
  EXPECT_EQ(broadphase.pairs().count, 0u);
  EXPECT_EQ(broadphase.pairs().device_keys, nullptr);
}
} // namespace surface_broadphase_test
