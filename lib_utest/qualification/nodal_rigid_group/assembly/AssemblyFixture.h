#pragma once
#include "lib_src/constraints/NodalRigidAssemblyValues.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <gtest/gtest.h>

namespace rigid_assembly_test {
namespace r=tl::fea::rigid;
using S=r::AssemblyValueStatus;
struct Fixture {
  std::array<r::AssemblyMassPoint,4> part{{{{-.6,.2,.4},2,.03},{{.5,-.4,.2},3,.04},
    {{.3,.7,-.2},4,.05},{{-.2,-.3,.8},1,.02}}};
  std::array<r::AssemblyMassPoint,2> extra{{{{1.2,-.8,.5},5,0},{{-.7,.1,-.9},.25,.01}}};
  r::AssemblyPrimary primary{{.03,.07,-.04},1e-20,1e-20};
  r::AssemblyBodyInput Input(bool extras=true) const {
    return {primary,part.data(),extras?extra.data():nullptr,part.size(),extras?extra.size():0,{1,1}};
  }
  void ScaleMass(double scale) {
    for(auto* points:{&part[0],&extra[0]}) {
      const auto count=points==part.data()?part.size():extra.size();
      for(std::size_t i=0;i<count;++i){points[i].mass*=scale;points[i].inertia*=scale;}
    }
  }
};
inline std::array<double,13> Values(const r::AssemblyRawBody& b) {
  std::array<double,13> out{b.mass,b.center.x,b.center.y,b.center.z};
  std::copy(std::begin(b.tensor.v),std::end(b.tensor.v),out.begin()+4);return out;
}
inline void Same(const r::AssemblyRawBody& a,const r::AssemblyRawBody& b) {
  const auto av=Values(a),bv=Values(b);for(unsigned i=0;i<av.size();++i)EXPECT_DOUBLE_EQ(av[i],bv[i]);
  EXPECT_EQ(a.ledger.part_members,b.ledger.part_members);EXPECT_EQ(a.ledger.extra_members,b.ledger.extra_members);
  EXPECT_EQ(a.ledger.primaries,b.ledger.primaries);EXPECT_DOUBLE_EQ(a.ledger.primary_mass,b.ledger.primary_mass);
}
template<class T> auto Bytes(const T& t) {
  std::array<unsigned char,sizeof(T)> b{};std::memcpy(b.data(),&t,sizeof(T));return b;
}
inline void Near(long double actual,long double expected,long double scale) {
  EXPECT_LE(std::abs(actual-expected),128*std::numeric_limits<double>::epsilon()*scale);
}
} // namespace rigid_assembly_test
