// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Prism.h"

namespace tlfea::contact::self_contact_filters {
namespace {
template <bool accepted>
__global__ void Classify(const TriangleGeometry* base, const TriangleGeometry* prepared,
    const FacetProperties* properties, const FixedTrianglePair* pairs,
    std::size_t count, SelfContactFacetPrismAxisLimit limit, PairResult* results) {
  for (std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
       i < count; i += blockDim.x * gridDim.x) {
    const auto pair = pairs[i];
    const auto a = properties[pair.first], b = properties[pair.second];
    PairResult result;
    if constexpr (accepted) {
      const auto filtered = detail::ClassifyAcceptedFacetPairImpl(
          base[pair.first], a.half_thickness, a.complete_rigid_group,
          base[pair.second], b.half_thickness, b.complete_rigid_group);
      result.status = filtered.status;
      result.category = filtered.category;
    } else {
      bool valid = false;
      result.separated = detail::CertifiedLinearFacetPrismSeparationImpl<true, false>(
          base[pair.first], prepared[pair.first], a.half_thickness,
          base[pair.second], prepared[pair.second], b.half_thickness,
          limit, &result.axis, &valid, nullptr);
      result.status = valid ? SelfContactFacetFilterStatus::Ok
                            : SelfContactFacetFilterStatus::InvalidInput;
    }
    results[i] = result;
  }
}
}  // namespace
cudaError_t Launch(void* device, const Layout& layout, std::size_t count,
    bool accepted, SelfContactFacetPrismAxisLimit limit, cudaStream_t stream) noexcept {
  if (!count) return cudaSuccess;
  const unsigned blocks = static_cast<unsigned>((count + 127) / 128 > 256 ? 256 : (count + 127) / 128);
  const auto* base = tl::util::ArenaPointer<TriangleGeometry>(device, layout.accepted);
  const auto* prepared = tl::util::ArenaPointer<TriangleGeometry>(device, layout.prepared);
  const auto* properties = tl::util::ArenaPointer<FacetProperties>(device, layout.properties);
  const auto* pairs = tl::util::ArenaPointer<FixedTrianglePair>(device, layout.pairs);
  auto* output = tl::util::ArenaPointer<PairResult>(device, layout.results);
  if (accepted) Classify<true><<<blocks, 128, 0, stream>>>(base, prepared, properties, pairs, count, limit, output);
  else Classify<false><<<blocks, 128, 0, stream>>>(base, prepared, properties, pairs, count, limit, output);
  return cudaPeekAtLastError();
}
}  // namespace tlfea::contact::self_contact_filters
