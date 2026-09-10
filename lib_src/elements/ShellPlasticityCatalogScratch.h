// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBindingIdentityIndex.h"
#include "ShellPlasticityCatalogLimits.h"

namespace tl::fea::shell_plasticity_binding_detail {
// Legacy catalog sizes stay entirely inline. Index sorting changes neither
// publication order nor validation order. Definitions retain source order too.
struct CatalogScratch {
  using Index=shell_binding_detail::IdentityIndex<MaxPlasticityCatalogDefinitions>;
  using Seen=tl::util::BoundedStartupArray<bool,MaxPlasticityCatalogDefinitions>;
  Index parts,materials,sections;
  Seen qeph,t3;
  std::array<bool,MaxPlasticityCatalogDefinitions> material_seen{},section_seen{};
  static constexpr std::size_t Bytes(std::size_t parents,std::size_t q,std::size_t t) noexcept {
    return sizeof(CatalogScratch)+Index::Storage::ExtraBytes(parents)+
      Seen::ExtraBytes(q)+Seen::ExtraBytes(t);
  }
};
} // namespace tl::fea::shell_plasticity_binding_detail
