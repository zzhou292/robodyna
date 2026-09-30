// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Types.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include "lib_utils/SourceIdentityIndex.h"
#include <array>
namespace tlfea::contact::radioss_type25::runtime_detail::physical_main {
using Face=std::array<std::uint32_t,4>;
struct IndexForecast {TransactionReport report;std::size_t bytes=0;};
// Ephemeral startup lookup. Borrows the immutable physical binding during the
// source call; it never changes source order, coefficients, a participant or owner.
class Index {
 public:
  static IndexForecast Preflight(const tl::fea::ShellPhysicalBinding&,std::size_t cap) noexcept;
  TransactionReport Initialize(const tl::fea::ShellPhysicalBinding&,std::size_t cap);
  // Ordinals preserve the complete physical shell/solid table order. They are
  // source lookups only, never runtime activity or synthetic contact owners.
  std::size_t ShellOrdinal(std::uint64_t id) const noexcept {
    return physical_&&id ? shells_.First(id) : SIZE_MAX;
  }
  std::size_t SolidOrdinal(std::uint64_t id) const noexcept {
    return physical_&&id ? solids_.First(id) : SIZE_MAX;
  }
  bool Shell(std::uint64_t source_id,Face&,bool& triangle) const noexcept;
  bool Solid(std::uint64_t source_id,std::array<std::uint32_t,8>&) const noexcept;
 private:
  const tl::fea::ShellPhysicalBinding* physical_=nullptr;
  tl::util::SourceIdentityIndex<0> shells_,solids_;
};
bool SameFace(const Face&,const Face&) noexcept;
bool Contains(const std::uint32_t*,std::size_t,const Face&) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail::physical_main
