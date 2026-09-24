// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Batch.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_filters {
struct Layout {
  tl::util::ArenaRegion accepted, prepared, properties, pairs, results;
  tl::util::ArenaRegion host_results;
  Forecast forecast;
};
Report MakeLayout(Limits, std::size_t owner_bytes, Layout&) noexcept;
cudaError_t Launch(void*, const Layout&, std::size_t, bool,
                   SelfContactFacetPrismAxisLimit, cudaStream_t) noexcept;
struct Batch::Impl {
  ~Impl();
  Layout layout;
  tl::util::HostArena host;
  PairResult* staging = nullptr;
  void* device = nullptr;
  cudaStream_t stream = nullptr;
  std::size_t scene_count = 0, result_count = 0;
  std::uint64_t generation = 0;
  bool usable = true, scene_ready = false, complete = false;
};
}  // namespace tlfea::contact::self_contact_filters
