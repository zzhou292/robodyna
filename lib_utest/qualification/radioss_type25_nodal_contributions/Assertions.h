// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <gtest/gtest.h>
#include <cstring>
namespace type25_contribution_test {
inline std::uint64_t Bits(double x){std::uint64_t bits;std::memcpy(&bits,&x,sizeof bits);return bits;}
inline void Exact(double a,double b){EXPECT_EQ(Bits(a),Bits(b));}
inline void Same(n::NativeSolidNodalShares a,n::NativeSolidNodalShares b) {
  Exact(a.volume_share,b.volume_share);Exact(a.bulk_volume_share,b.bulk_volume_share);EXPECT_EQ(a.defined_raw_slot_mask,b.defined_raw_slot_mask);
}
inline void CompareSolid(const n::NativeSolidNodalInput& in,const n::NativeSolidNodalShares& actual) {
  constexpr double seed=-873.25;const auto native=OracleSolid(in,seed);
  const unsigned mask=in.kind==n::SolidNodalKind::Hex8?0xff:0x77;EXPECT_EQ(actual.defined_raw_slot_mask,mask);
  for(unsigned slot=0;slot<8;++slot) {
    SCOPED_TRACE(slot);
    if(mask&(1u<<slot)){Exact(actual.volume_share,native.volume[slot]);Exact(actual.bulk_volume_share,native.bulk_volume[slot]);}
    else {Exact(native.volume[slot],seed);Exact(native.bulk_volume[slot],seed);}
  }
  for(unsigned slot=0;slot<12;++slot){Exact(native.extended_volume[slot],seed);Exact(native.extended_bulk_volume[slot],seed);}
}
} // namespace type25_contribution_test
