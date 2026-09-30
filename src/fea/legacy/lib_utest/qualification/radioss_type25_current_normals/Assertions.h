// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
namespace type25_current_normals_test {
inline std::uint32_t Bits(float x){std::uint32_t b;std::memcpy(&b,&x,sizeof b);return b;}
inline void SameNormal(n::StoredNormal a,n::StoredNormal b) {
  EXPECT_EQ(Bits(a.x),Bits(b.x));
  EXPECT_EQ(Bits(a.y),Bits(b.y));
  EXPECT_EQ(Bits(a.z),Bits(b.z));
}
inline void SameNormals(const n::StoredNormal* a,const n::StoredNormal* b,std::size_t count) {
  for(std::size_t i=0;i<count;++i){SCOPED_TRACE(i);SameNormal(a[i],b[i]);}
}
inline void SameNormals(const std::vector<n::StoredNormal>& a,const std::vector<n::StoredNormal>& b) {
  ASSERT_EQ(a.size(),b.size());SameNormals(a.data(),b.data(),a.size());
}
inline void SameReferences(const s::NormalReference* a,const s::NormalReference* b,std::size_t count) {
  for(std::size_t i=0;i<count;++i){SCOPED_TRACE(i);
    EXPECT_EQ(a[i].boundary,b[i].boundary);SameNormal(a[i].bisector[0],b[i].bisector[0]);SameNormal(a[i].bisector[1],b[i].bisector[1]);}
}
inline void SameReferences(const std::vector<s::NormalReference>& a,const std::vector<s::NormalReference>& b) {
  ASSERT_EQ(a.size(),b.size());SameReferences(a.data(),b.data(),a.size());
}
inline void SameReport(c::Report a,c::Report b){EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.main,b.main);EXPECT_EQ(a.reference,b.reference);EXPECT_EQ(a.node,b.node);}
inline void Same(const Result& actual,const NativeResult& expected) {
  ASSERT_EQ(actual.report.status,c::Status::Ok);ASSERT_TRUE(expected.finite);
  SameNormals(actual.normals,expected.normals);SameReferences(actual.references,expected.references);
}
}
