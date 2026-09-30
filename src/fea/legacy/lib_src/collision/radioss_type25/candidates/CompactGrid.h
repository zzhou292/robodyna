// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "InventoryTypes.h"
namespace tlfea::contact::radioss_type25::candidates::detail {
inline constexpr std::size_t MaximumCompactEncounters=std::size_t{32}<<20;
struct GridControl {
  unsigned long long lower_bits[3]{},upper_bits[3]{};
  Bounds bounds;
  double span[3]{};
  unsigned cells[3]{1,1,1};
  unsigned packing_failed=0;
};
TL_MATH_HOST_DEVICE inline unsigned GridResolution(std::uint64_t active) {
  unsigned n=1;while(n<128&&std::uint64_t(n)*n*n<active)n*=2;return n;
}
TL_MATH_HOST_DEVICE inline unsigned GridCell(double x,double low,double high,double span,unsigned cells) {
  if(cells==1||x<=low)return 0;
  if(x>=high)return cells-1;
  const auto cell=unsigned(((x-low)/span)*double(cells));
  return cell<cells?cell:cells-1;
}
TL_MATH_HOST_DEVICE inline double GridKey(const unsigned* cells,unsigned x,unsigned y,unsigned z) {
  return double((std::uint64_t(z)*cells[1]+y)*cells[0]+x);
}
}
