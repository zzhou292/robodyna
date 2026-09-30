// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <cub/cub.cuh>
#include <memory>
#include <stdexcept>

namespace tl::constraints::tied_shell {
namespace {
void Cuda(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error("Tied search CUDA operation failed");
}
class Radii {
 public:
  Radii() = default;
  Radii(const Radii&) = delete;
  Radii& operator=(const Radii&) = delete;
  ~Radii() { if (data) cudaFree(data); }
  void Upload(const std::vector<double>& values) {
    Cuda(cudaMalloc(&data, values.size() * sizeof(double)));
    Cuda(cudaMemcpy(data, values.data(), values.size() * sizeof(double), cudaMemcpyHostToDevice));
  }
  double* data = nullptr;
};
driver_detail::Scratch QueryScratch(std::size_t count) {
  driver_detail::Scratch out;
  Cuda(cub::DeviceRadixSort::SortPairs(nullptr, out.sort, static_cast<double*>(nullptr),
      static_cast<double*>(nullptr), static_cast<int*>(nullptr), static_cast<int*>(nullptr), int(count)));
  Cuda(cub::DeviceScan::ExclusiveSum(nullptr, out.scan, static_cast<unsigned long long*>(nullptr),
      static_cast<unsigned long long*>(nullptr), int(count + 1)));
  return out;
}
}
SearchDriverReport AssessSearch(const SearchDriverInput& in, const SearchDriverLimits& limits,
    SearchDriverResult& output) noexcept {
  using namespace driver_detail;
  try {
    auto report = CheckCounts(in, limits);
    if (!report) return report;
    SearchDriverResult staged;
    // Reject impossible fixed budgets before any CUDA operation or borrowed
    // payload access. Exact CUB scratch admission follows before allocation.
    report = Budget(in, limits, {}, staged.budget);
    if (!report) return report;
    const auto primitives = in.master_count + in.secondary_count;
    const auto scratch = QueryScratch(primitives);
    report = Budget(in, limits, scratch, staged.budget);
    if (!report) return report;
    report = CheckValues(in);
    if (!report) return report;
    std::vector<NativeSearchBounds> bounds;
    std::vector<double> radii;
    report = PrepareBounds(in, bounds, radii);
    if (!report) return report;
    Eigen::MatrixXd positions(in.node_count, 3);
    Eigen::MatrixXi elements(primitives, 4);
    Eigen::VectorXi bodies(primitives);
    for (std::size_t n = 0; n < in.node_count; ++n) {
      const auto x = in.working_positions[n];
      positions.row(n) << x.x, x.y, x.z;
    }
    for (std::size_t m = 0; m < in.master_count; ++m) {
      for (unsigned n = 0; n < 4; ++n) elements(m, n) = in.masters[m].nodes[n];
      bodies[m] = 0;
    }
    for (std::size_t s = 0; s < in.secondary_count; ++s) {
      elements.row(in.master_count + s).setConstant(in.secondary_nodes[s]);
      bodies[in.master_count + s] = 1;
    }
    Radii device_radii;
    Broadphase broadphase; // Destroyed before its borrowed radii.
    broadphase.SetVerbose(false);
    device_radii.Upload(radii);
    broadphase.Initialize(positions, elements, bodies);
    if (broadphase.tempStorageBytes != scratch.sort)
      return Error(SearchDriverStatus::ResourceLimit, "Tied broadphase sort forecast changed");
    broadphase.EnableSelfCollision(false);
    BroadphaseAABBOptions options;
    options.d_elementInflation = device_radii.data;
    broadphase.SetAABBOptions(options);
    const auto workspace = staged.budget.device_bytes - FixedDeviceBytes(in, scratch);
    broadphase.SetDetectionLimits({staged.budget.pair_capacity, workspace});
    broadphase.CreateAABB();
    broadphase.SortAABBs(limits.axis);
    broadphase.DetectCollisions(true); // Synchronous copy completes pair fill.
    if (broadphase.scanTempStorageBytes != scratch.scan ||
        broadphase.GetDetectionWorkspaceBytes() > workspace)
      return Error(SearchDriverStatus::ResourceLimit, "Tied detection forecast changed");
    staged.axis = limits.axis;
    report = Reduce(in, broadphase.h_collisionPairs, bounds, staged);
    if (!report) return report;
    output = std::move(staged);
    return {};
  } catch (const std::bad_alloc&) {
    return driver_detail::Error(SearchDriverStatus::AllocationFailure, "Tied search host allocation failed");
  } catch (const std::length_error&) {
    return driver_detail::Error(SearchDriverStatus::ResourceLimit, "Tied search candidate/workspace capacity exceeded");
  } catch (const std::invalid_argument&) {
    return driver_detail::Error(SearchDriverStatus::InvalidInput, "Tied search broadphase input rejected");
  } catch (const std::exception&) {
    return driver_detail::Error(SearchDriverStatus::CudaFailure, "Tied search CUDA stage failed");
  }
}
} // namespace tl::constraints::tied_shell
