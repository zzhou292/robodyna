// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../current_normals/Stages.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// All fields are private to the contact transaction's one bounded arena. The
// accepted history selector selects the matching cache; there is no new clock.
struct NormalShape {
  bool enabled=false;
  std::size_t free_count=0;
  normal_activation::Profile activation;
  bool mixed=false;
};
struct NormalLayout {
  tl::util::ArenaRegion topology,coefficients,free_mains,optimized,partners;
  tl::util::ArenaRegion face[2],references[2],active,tags,neighbor,eligible,tage,slots;
  std::size_t bytes=0;
};
struct NormalDevice {
  NormalShape shape;
  current_normals::Topology topology;
  const double* coefficients=nullptr;
  const std::uint32_t* free_mains=nullptr;
  lifecycle::OptimizedRow* optimized=nullptr;
  StoredNormal* face[2]{};
  startup::NormalReference* references[2]{};
  std::uint32_t* active=nullptr;
  std::uint32_t* tags=nullptr;
  StoredNormal* neighbor=nullptr;
  unsigned char* eligible=nullptr;
  unsigned char* tage=nullptr;
  std::uint32_t* slots=nullptr;
};
} // namespace tlfea::contact::radioss_type25::runtime_detail
