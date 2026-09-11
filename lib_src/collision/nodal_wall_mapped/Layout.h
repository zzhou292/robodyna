// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NodalWallContactArena.h"
#include "../RigidNormalResponse.h"
namespace tlfea::contact::nodal_wall_mapped {
struct Summary {
  Q4CertifiedIntegral removed_potential;
  double rate=0;
  // Integer arbitration only; parent floating sums stay in original slot order.
  unsigned long long parent_failure=~0ull;
  bool points_admitted=false;
};
struct Sidecar {
  std::uint8_t* accepted=nullptr;
  std::uint8_t* proposed=nullptr;
  std::uint32_t* roots=nullptr; // Compact incident node -> group, UINT32_MAX ordinary.
  RigidContactBody* bodies=nullptr;
  double* traces=nullptr;
  double* stiffness=nullptr; // Compact private STI addition before any scatter.
  double* inverse=nullptr; // Current accepted inverse, frozen only for this attempt.
  Summary* summary=nullptr;
  std::size_t groups=0;
};
struct Layout {
  tl::util::ArenaRegion accepted,proposed,roots,bodies,traces,stiffness,inverse,summary;
  std::size_t bytes=0;
};
bool MakeLayout(std::size_t parents,std::size_t nodes,std::size_t groups,
    std::size_t cap,Layout&) noexcept;
Sidecar Bind(void*,const Layout&) noexcept;
} // namespace tlfea::contact::nodal_wall_mapped
