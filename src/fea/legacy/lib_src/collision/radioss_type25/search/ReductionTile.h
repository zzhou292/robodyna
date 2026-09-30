// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Device.h"
#include <type_traits>
namespace tlfea::contact::radioss_type25::search::detail {
// Plain shared storage, following the existing mapped reduction pattern.
// Do not place nontrivial default-initialized public records in CUDA shared memory.
struct ReductionTile {
  double values[25][Threads];
  std::uint64_t secondary[Threads], main[Threads];
  std::size_t invalid[Threads];
  Status status[Threads];
};
static_assert(std::is_trivial_v<ReductionTile>);
TL_MATH_HOST_DEVICE inline void Store(ReductionTile& tile,unsigned slot,const Partial& p) noexcept {
  const MotionExtrema* boxes[]{&p.extrema.secondary_displacement,&p.extrema.main_displacement,
      &p.extrema.secondary_velocity,&p.extrema.main_velocity};
  for(unsigned i=0;i<4;++i) {
    const auto& b=*boxes[i];
    tile.values[6*i][slot]=b.maximum.x;tile.values[6*i+1][slot]=b.maximum.y;tile.values[6*i+2][slot]=b.maximum.z;
    tile.values[6*i+3][slot]=b.minimum.x;tile.values[6*i+4][slot]=b.minimum.y;tile.values[6*i+5][slot]=b.minimum.z;
  }
  tile.values[24][slot]=p.extrema.maximum_gap_change;
  tile.secondary[slot]=p.extrema.secondary_uses;tile.main[slot]=p.extrema.main_uses;
  tile.invalid[slot]=p.invalid;tile.status[slot]=p.status;
}
TL_MATH_HOST_DEVICE inline Partial Load(const ReductionTile& tile,unsigned slot) noexcept {
  Partial p;
  MotionExtrema* boxes[]{&p.extrema.secondary_displacement,&p.extrema.main_displacement,
      &p.extrema.secondary_velocity,&p.extrema.main_velocity};
  for(unsigned i=0;i<4;++i) {
    auto& b=*boxes[i];
    b.maximum={tile.values[6*i][slot],tile.values[6*i+1][slot],tile.values[6*i+2][slot]};
    b.minimum={tile.values[6*i+3][slot],tile.values[6*i+4][slot],tile.values[6*i+5][slot]};
  }
  p.extrema.maximum_gap_change=tile.values[24][slot];
  p.extrema.secondary_uses=tile.secondary[slot];p.extrema.main_uses=tile.main[slot];
  p.invalid=tile.invalid[slot];p.status=tile.status[slot];return p;
}
} // namespace tlfea::contact::radioss_type25::search::detail
