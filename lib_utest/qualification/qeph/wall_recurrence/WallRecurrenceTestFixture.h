#pragma once
#include "ContactBranchProbe.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace tl::qualification::qeph::wall_recurrence::test {
inline void Near(double actual,long double expected,double scale=1) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),
            2e-12L*std::max(static_cast<long double>(scale),std::abs(expected)));
}
inline void Certificate(const contact::Q4CertifiedIntegral& a,long double truth) {
  EXPECT_TRUE(std::isfinite(a.value)); EXPECT_TRUE(std::isfinite(a.lower));
  EXPECT_TRUE(std::isfinite(a.upper)); EXPECT_TRUE(std::isfinite(a.error));
  EXPECT_LE(static_cast<long double>(a.lower),truth); EXPECT_GE(static_cast<long double>(a.upper),truth);
  EXPECT_GE(a.error,0); EXPECT_LE(std::abs(static_cast<long double>(a.value)-a.lower),a.error);
  EXPECT_LE(std::abs(static_cast<long double>(a.value)-a.upper),a.error);
}
// Independent represented-coordinate diagonal area, no owning shape/area law.
inline long double Area(const Reference& reference) {
  const auto& x=reference.data().input.position;
  const long double a[]{static_cast<long double>(x[2].x)-x[0].x,
                        static_cast<long double>(x[2].y)-x[0].y,
                        static_cast<long double>(x[2].z)-x[0].z};
  const long double b[]{static_cast<long double>(x[3].x)-x[1].x,
                        static_cast<long double>(x[3].y)-x[1].y,
                        static_cast<long double>(x[3].z)-x[1].z};
  const long double cross[]{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
  return .5L*std::sqrt(cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2]);
}
inline std::vector<unsigned char> Bytes(const void* data,std::size_t count) {
  std::vector<unsigned char> out(count); if(count) std::memcpy(out.data(),data,count); return out;
}
// Snapshot exact existing object/container bytes, not a copy-constructed POD's
// unspecified padding. Failure cannot resize, replace or mutate any output.
struct SampleSnapshot {
  std::vector<unsigned char> object,state,nodes,parents;
  explicit SampleSnapshot(const ContactMapSample& s)
      :object(Bytes(&s,sizeof(s))),state(Bytes(s.state.data(),s.state.size()*sizeof(double))),
       nodes(Bytes(s.nodes.data(),s.nodes.size()*sizeof(contact::NodalWallPointResult))),
       parents(Bytes(s.parents.data(),s.parents.size()*sizeof(contact::NodalWallParentResult))) {}
  void Check(const ContactMapSample& s) const {
    EXPECT_EQ(object,Bytes(&s,sizeof(s))); EXPECT_EQ(state,Bytes(s.state.data(),s.state.size()*sizeof(double)));
    EXPECT_EQ(nodes,Bytes(s.nodes.data(),s.nodes.size()*sizeof(contact::NodalWallPointResult)));
    EXPECT_EQ(parents,Bytes(s.parents.data(),s.parents.size()*sizeof(contact::NodalWallParentResult)));
  }
};
} // namespace tl::qualification::qeph::wall_recurrence::test
