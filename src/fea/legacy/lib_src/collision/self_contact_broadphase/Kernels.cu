#include "Storage.h"
#include "../broadphase/Sweep.h"
#include <cfloat>

namespace tlfea::contact::self_contact_broadphase {
namespace {
__global__ void Reset(Control* control) { *control = {}; }
__device__ void Include(Vec3 point, double3& lower, double3& upper, bool& valid) {
  valid = valid && IsFinite(point);
  lower.x = fmin(lower.x, point.x); lower.y = fmin(lower.y, point.y); lower.z = fmin(lower.z, point.z);
  upper.x = fmax(upper.x, point.x); upper.y = fmax(upper.y, point.y); upper.z = fmax(upper.z, point.z);
}
__global__ void BuildBounds(const Parent* parents, int n, SelfContactBroadphaseInput input,
    AABB* boxes, Control* control) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += blockDim.x * gridDim.x) {
    const auto& parent = parents[i];
    const auto first = input.current.at(parent.nodes[0]);
    double3 lower = make_double3(first.x, first.y, first.z), upper = lower;
    bool valid = true;
    for (unsigned local = 0; local < parent.arity; ++local) {
      const auto node = parent.nodes[local];
      Include(input.current.at(node), lower, upper, valid);
      if (input.motion == SelfContactBoundsMotion::LinearNodalEndpoints)
        Include(input.endpoint.at(node), lower, upper, valid);
    }
    // Same directed inflation as computeAABBKernel. Bilinear Q4 and linear T3
    // are convex combinations of corners. Endpoint union covers linear nodal
    // interpolation only, never a finite-rotation arc or continuous collision.
    const double radius = parent.half_thickness;
    lower.x = __dsub_rd(lower.x, radius); lower.y = __dsub_rd(lower.y, radius);
    lower.z = __dsub_rd(lower.z, radius);
    upper.x = __dadd_ru(upper.x, radius); upper.y = __dadd_ru(upper.y, radius);
    upper.z = __dadd_ru(upper.z, radius);
    valid = valid && IsFinite({lower.x, lower.y, lower.z}) && IsFinite({upper.x, upper.y, upper.z});
    if (!valid) atomicMin(&control->invalid_parent, static_cast<std::uint32_t>(i));
    boxes[i] = {lower, upper, i};
  }
}
__global__ void CountLater(const AABB* boxes, int n, unsigned axis, unsigned long long* counts) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += blockDim.x * gridDim.x) {
    broadphase_detail::CountPairs count;
    broadphase_detail::VisitLater(boxes, n, i, axis, broadphase_detail::KeepAll{}, count);
    counts[i] = count.count;
  }
}
struct WriteCanonical {
  SelfContactPairKey* keys;
  unsigned long long offset;
  __host__ __device__ void operator()(int a, int b) {
    const auto low = static_cast<std::uint32_t>(a < b ? a : b);
    const auto high = static_cast<std::uint32_t>(a < b ? b : a);
    keys[offset++] = (static_cast<SelfContactPairKey>(low) << 32) | high;
  }
};
__global__ void FillLater(const AABB* boxes, int n, unsigned axis,
    const unsigned long long* offsets, SelfContactPairKey* keys) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += blockDim.x * gridDim.x) {
    WriteCanonical output{keys, offsets[i]};
    broadphase_detail::VisitLater(boxes, n, i, axis, broadphase_detail::KeepAll{}, output);
  }
}
unsigned Blocks(std::size_t n) { const auto b = (n + 255) / 256; return b < 256 ? b : 256; }
}
cudaError_t Bounds(void* arena, const Layout& layout, SelfContactBroadphaseInput input,
    cudaStream_t stream) noexcept {
  auto* control = tl::util::ArenaPointer<Control>(arena, layout.control);
  Reset<<<1,1,0,stream>>>(control);
  auto error = cudaPeekAtLastError(); if (error != cudaSuccess) return error;
  BuildBounds<<<Blocks(layout.forecast.parents),256,0,stream>>>(
      tl::util::ArenaPointer<Parent>(arena, layout.parents), layout.forecast.parents, input,
      tl::util::ArenaPointer<AABB>(arena, layout.boxes), control);
  return cudaPeekAtLastError();
}
cudaError_t Count(void* arena, const Layout& layout, unsigned axis, cudaStream_t stream) noexcept {
  auto* counts = tl::util::ArenaPointer<unsigned long long>(arena, layout.counts);
  auto error = cudaMemsetAsync(counts + layout.forecast.parents, 0, sizeof(*counts), stream);
  if (error != cudaSuccess) return error;
  CountLater<<<Blocks(layout.forecast.parents),256,0,stream>>>(
      tl::util::ArenaPointer<AABB>(arena, layout.sorted_boxes), layout.forecast.parents, axis, counts);
  return cudaPeekAtLastError();
}
cudaError_t Fill(void* arena, const Layout& layout, unsigned axis, cudaStream_t stream) noexcept {
  FillLater<<<Blocks(layout.forecast.parents),256,0,stream>>>(
      tl::util::ArenaPointer<AABB>(arena, layout.sorted_boxes), layout.forecast.parents, axis,
      tl::util::ArenaPointer<unsigned long long>(arena, layout.offsets),
      tl::util::ArenaPointer<SelfContactPairKey>(arena, layout.pair_keys));
  return cudaPeekAtLastError();
}
} // namespace tlfea::contact::self_contact_broadphase
