#pragma once
#include "Layout.h"

namespace tlfea::contact {
struct SelfContactBroadphase::Impl {
  explicit Impl(const SelfContactSurfaceBinding& input) : source(input) {}
  ~Impl();
  SelfContactSurfaceBinding source;
  self_contact_broadphase::Layout layout;
  self_contact_broadphase::Control host_control;
  tl::util::HostArena conservative_bounds;
  AABB* staged_bounds = nullptr;
  void* device = nullptr;
  bool usable = true, complete = false;
  std::uint64_t pair_count = 0;
};
} // namespace tlfea::contact

namespace tlfea::contact::self_contact_broadphase {
cudaError_t QueryScratch(int parents, int pairs, ScratchRequirements&) noexcept;
cudaError_t SortBoxes(void*, const Layout&, unsigned axis, cudaStream_t) noexcept;
cudaError_t ScanCounts(void*, const Layout&, cudaStream_t) noexcept;
cudaError_t SortPairs(void*, const Layout&, int count, cudaStream_t) noexcept;
cudaError_t Bounds(void*, const Layout&, SelfContactBroadphaseInput, cudaStream_t) noexcept;
cudaError_t Count(void*, const Layout&, unsigned axis, cudaStream_t) noexcept;
cudaError_t Fill(void*, const Layout&, unsigned axis, cudaStream_t) noexcept;
} // namespace tlfea::contact::self_contact_broadphase
