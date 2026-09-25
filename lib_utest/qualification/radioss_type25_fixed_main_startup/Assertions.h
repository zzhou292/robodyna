// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <cstring>
namespace type25_startup_test {
inline std::uint32_t Bits(float value){std::uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));return bits;}
inline void Same(n::StoredNormal a,n::StoredNormal b) {
  EXPECT_EQ(Bits(a.x),Bits(b.x));EXPECT_EQ(Bits(a.y),Bits(b.y));EXPECT_EQ(Bits(a.z),Bits(b.z));
}
inline void SameStarter(const s::Snapshot& view,const NativeResult& expected) {
  ASSERT_EQ(view.main_count,expected.mains.size());
  ASSERT_EQ(view.starter.reference_count,expected.starter_references.size());
  ASSERT_EQ(view.normal_incidence_count,expected.incidence.size());
  for(std::size_t i=0;i<view.main_count;++i) {
    SCOPED_TRACE(i);const auto& a=view.mains[i];const auto& b=expected.mains[i];
    EXPECT_EQ(a.source_id,b.source_id);EXPECT_EQ(a.global_id,b.global_id);EXPECT_EQ(a.segment_type,b.segment_type);
    EXPECT_EQ(view.expanded_to_primary[i],expected.expanded_to_primary[i]);
    for(unsigned k=0;k<4;++k) {
      EXPECT_EQ(a.nodes[k],b.nodes[k]);EXPECT_EQ(a.neighbors[k],b.neighbors[k]);
      EXPECT_EQ(a.neighbor_edges[k],b.neighbor_edges[k]);EXPECT_EQ(a.normal_reference[k],b.normal_reference[k]);
      Same(view.starter.face_normals[4*i+k],expected.starter_normals[4*i+k]);
    }
  }
  for(std::size_t i=0;i<view.primary_count;++i)EXPECT_EQ(view.primary_to_partner[i],expected.primary_to_partner[i]);
  for(std::size_t i=0;i<=view.starter.reference_count;++i)EXPECT_EQ(view.normal_offsets[i],expected.offsets[i]);
  for(std::size_t i=0;i<view.normal_incidence_count;++i)EXPECT_EQ(view.normal_mains[i],expected.incidence[i]);
  for(std::size_t i=0;i<view.starter.reference_count;++i) {
    EXPECT_EQ(view.starter.references[i].boundary,expected.starter_references[i].boundary);
    for(unsigned k=0;k<2;++k)Same(view.starter.references[i].bisector[k],expected.starter_references[i].bisector[k]);
  }
}
inline void Same(const Built& actual,const NativeResult& expected) {
  SameStarter(actual.startup,expected);
  ASSERT_EQ(actual.ready.normals.reference_count,expected.ready_references.size());
  for(std::size_t i=0;i<4*actual.startup.main_count;++i)Same(actual.ready.normals.face_normals[i],expected.ready_normals[i]);
  for(std::size_t i=0;i<actual.ready.normals.reference_count;++i) {
    EXPECT_EQ(actual.ready.normals.references[i].boundary,expected.ready_references[i].boundary);
    for(unsigned k=0;k<2;++k)Same(actual.ready.normals.references[i].bisector[k],expected.ready_references[i].bisector[k]);
  }
}
} // namespace type25_startup_test
