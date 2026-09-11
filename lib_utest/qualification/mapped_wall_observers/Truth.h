#pragma once
#include "Fixture.h"
#include <cstring>
namespace wall_observer_test {
static_assert(__FLT128_MANT_DIG__==113,"Independent binary128 truth accumulator");
using High=__float128;
inline High Abs(High x) {return x<0?-x:x;}
struct TruthSum {
  High value=0,absolute=0;
  void Add(double x) {value+=High(x);absolute+=Abs(High(x));}
  High Bound(unsigned nodes) const {
    const auto blocks=m::ObserverBlocks(nodes);
    const High depth=(nodes+blocks*128-1)/(blocks*128)+7+(blocks+127)/128+7+1;
    const High du=depth/High(std::uint64_t{1}<<53);
    const High additions=High(nodes)+2*High(blocks)*128+257;
    // Rounded leaves only: fixed-path depth, absolute terms for cancellation,
    // and gradual-underflow at every addition. The factor two also covers
    // the much smaller binary128 truth-accumulation rounding.
    return 2*(du*absolute+additions*High(std::numeric_limits<double>::denorm_min()))/(1-du);
  }
};
struct Truth {
  TruthSum sums[9]; // force, potential, reaction3, moment3, power
  High lower[2]{},upper[2]{};
  void Add(const c::NodalWallPointResult& node) {
    const auto f=node.force,u=node.potential;
    const double terms[]={f.value,u.value,node.wall_reaction.x,node.wall_reaction.y,node.wall_reaction.z,
      node.wall_moment.x,node.wall_moment.y,node.wall_moment.z,node.surface_power};
    for(unsigned i=0;i<9;++i)sums[i].Add(terms[i]);
    lower[0]+=High(f.lower);upper[0]+=High(f.upper);lower[1]+=High(u.lower);upper[1]+=High(u.upper);
  }
};
inline void CheckTruth(const c::NodalWallPointResult* nodes,unsigned count,const c::NodalWallDiagnostics& actual) {
  Truth truth;for(unsigned n=0;n<count;++n)truth.Add(nodes[n]);
  const double values[]={actual.resultant.value,actual.potential.value,actual.wall_reaction.x,
    actual.wall_reaction.y,actual.wall_reaction.z,actual.wall_moment.x,actual.wall_moment.y,actual.wall_moment.z,
    actual.surface_power};
  for(unsigned i=0;i<9;++i) {
    const auto bound=truth.sums[i].Bound(count);
    EXPECT_TRUE(Abs(High(values[i])-truth.sums[i].value)<=bound)<<"channel "<<i;
    const double corrupt=::nextafter(double(truth.sums[i].value+4*bound+High(1)),INFINITY);
    EXPECT_TRUE(Abs(High(corrupt)-truth.sums[i].value)>bound)<<"corruption control "<<i;
  }
  const c::Q4CertifiedIntegral integral[]={actual.resultant,actual.potential};
  for(unsigned i=0;i<2;++i) {
    EXPECT_TRUE(High(integral[i].lower)<=truth.lower[i]);
    EXPECT_TRUE(High(integral[i].upper)>=truth.upper[i]);
    EXPECT_TRUE(High(integral[i].error)>=Abs(High(integral[i].value)-truth.lower[i]));
    EXPECT_TRUE(High(integral[i].error)>=Abs(High(integral[i].value)-truth.upper[i]));
    const High corrupt=High(integral[i].upper)+High(1)+High(integral[i].error);
    EXPECT_TRUE(corrupt>truth.upper[i]);
  }
}
inline std::uint64_t Bits(double value) {std::uint64_t out;std::memcpy(&out,&value,8);return out;}
inline void SameObserved(const c::NodalWallDiagnostics& a,const c::NodalWallDiagnostics& b) {
  const c::Q4CertifiedIntegral ac[]={a.resultant,a.potential},bc[]={b.resultant,b.potential};
  for(unsigned i=0;i<2;++i) {
    EXPECT_EQ(Bits(ac[i].value),Bits(bc[i].value));EXPECT_EQ(Bits(ac[i].lower),Bits(bc[i].lower));
    EXPECT_EQ(Bits(ac[i].upper),Bits(bc[i].upper));EXPECT_EQ(Bits(ac[i].error),Bits(bc[i].error));
  }
  const auto x=Signed(a),y=Signed(b);for(unsigned i=0;i<7;++i)EXPECT_EQ(Bits(x[i]),Bits(y[i]));
  EXPECT_EQ(Bits(a.maximum_penetration),Bits(b.maximum_penetration));
  EXPECT_EQ(Bits(a.stiffness_rate_bound),Bits(b.stiffness_rate_bound));
  EXPECT_EQ(a.node_count,b.node_count);EXPECT_EQ(a.parent_count,b.parent_count);
  EXPECT_EQ(a.owner_id,b.owner_id);EXPECT_EQ(a.attempt,b.attempt);EXPECT_EQ(a.base_epoch,b.base_epoch);
}
} // namespace wall_observer_test
