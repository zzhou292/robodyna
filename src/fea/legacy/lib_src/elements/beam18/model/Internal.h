// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Model.h"
#include "../ForceChecks.h"
#include "../../../assembly/NodalDomainIdentity.h"
#include "../../../../lib_utils/BoundedArena.h"
#include "../../../../lib_utils/SourceIdentityIndex.h"

namespace tl::fea::beam18::model_detail {
using S=ModelStatus;
using Index=util::SourceIdentityIndex<0>;
struct Plan {
  util::BoundedArenaLayout arena;
  util::ArenaRegion parents,materials,curves;
  std::size_t material_count=0,curve_points=0,retained=0,startup=0;
  explicit Plan(std::size_t cap):arena(cap) {}
};
ModelReport Count(const NodalNodeDomain&,ModelInput,ModelLimits,std::size_t) noexcept;
ModelReport Inventory(ModelInput,ModelLimits,const Index&,const Index&,Plan&) noexcept;
ModelReport Budget(const NodalNodeDomain&,ModelInput,ModelLimits,std::size_t,Plan&) noexcept;
ModelReport BindParent(const NodalNodeDomain&,const Reference&,Parent&,std::size_t) noexcept;
bool SameMaterial(const Material&,const Material&) noexcept;
bool SameReference(const Reference&,const Reference&) noexcept;
bool CopyMaterial(const Material&,double*&,Material&) noexcept;
} // namespace tl::fea::beam18::model_detail
