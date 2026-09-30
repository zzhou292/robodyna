// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RadiossType25Candidates.h"
#include "Launch.h"
namespace tlfea::contact::radioss_type25::candidates {
struct Inventory::Impl {
  ~Impl();
  Source source;Limits limits;detail::Layout layout;detail::Device device;
  void* arena=nullptr;cudaStream_t stream=nullptr;
  std::uint64_t identity=0,sequence=0;bool usable=true,pending=false;
  detail::Control control;Report report;
  Status Check(const Current&) const noexcept;
  Status Fence(cudaError_t) noexcept;
};
} // namespace tlfea::contact::radioss_type25::candidates
