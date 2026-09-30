// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"

namespace rigid_part_model_test::precise {
using Q=__float128;
inline Q Abs(Q x) {return x<0?-x:x;}
inline Q Max(Q a,Q b) {return a>b?a:b;}
inline Q Gamma(std::size_t n) {
  const Q nu=Q(n)*Q(std::numeric_limits<double>::epsilon()/2);
  return nu/(1-nu);
}
struct Body {
  Q mass=0;
  std::array<Q,3> center{},absolute_numerator{};
  std::array<Q,9> tensor{},absolute_tensor{};
  Q coordinate_scale=0;
  std::size_t terms=0;
};
inline std::array<Q,3> Position(r::Vec3 x) {return {Q(x.x),Q(x.y),Q(x.z)};}
inline void AddCenter(Body& b,Q mass,std::array<Q,3> x) {
  b.mass+=mass;
  for(unsigned k=0;k<3;++k) {
    b.center[k]+=mass*x[k];
    b.absolute_numerator[k]+=Abs(mass*x[k]);
    b.coordinate_scale=Max(b.coordinate_scale,Abs(x[k]));
  }
  ++b.terms;
}
inline void AddTensor(Body& b,Q mass,Q j,std::array<Q,3> x) {
  for(unsigned k=0;k<3;++k) x[k]-=b.center[k];
  for(unsigned a=0;a<3;++a) for(unsigned c=0;c<3;++c) {
    Q term=0;
    if(a==c) {
      term=j;
      for(unsigned k=0;k<3;++k) if(k!=a) term+=mass*x[k]*x[k];
    } else term=-mass*x[a]*x[c];
    b.tensor[3*a+c]+=term;
    b.absolute_tensor[3*a+c]+=Abs(term);
  }
}
inline Body Original(const Packet& packet) {
  Body b;
  std::array<Q,3> mean{};
  for(const auto& p:packet.part) {
    const auto x=Position(p.position);
    for(unsigned k=0;k<3;++k) mean[k]+=x[k];
  }
  for(auto& x:mean) x/=packet.part.size();
  AddCenter(b,Q(packet.primary.mass),mean);
  for(const auto& p:packet.part) AddCenter(b,Q(p.mass),Position(p.position));
  for(const auto& p:packet.extra) AddCenter(b,Q(p.mass),Position(p.position));
  for(auto& x:b.center) x/=b.mass;
  // Exact real-valued INIRBY extra-reset branch, without reproducing binary64
  // intermediate sums. The original PART-only primary mean is independently Q.
  AddTensor(b,Q(packet.primary.mass),Q(packet.primary.inertia),packet.extra.empty()?mean:b.center);
  for(const auto& p:packet.part) AddTensor(b,Q(p.mass),Q(p.inertia),Position(p.position));
  for(const auto& p:packet.extra) AddTensor(b,Q(p.mass),Q(p.inertia),Position(p.position));
  return b;
}
inline Body Merge(const Body& a,const Body& c) {
  Body b;
  AddCenter(b,a.mass,a.center);AddCenter(b,c.mass,c.center);
  for(auto& x:b.center) x/=b.mass;
  b.terms=a.terms+c.terms;
  b.coordinate_scale=Max(a.coordinate_scale,c.coordinate_scale);
  for(unsigned k=0;k<3;++k) b.absolute_numerator[k]=a.absolute_numerator[k]+c.absolute_numerator[k];
  for(unsigned k=0;k<9;++k) {
    b.tensor[k]=a.tensor[k]+c.tensor[k];
    b.absolute_tensor[k]=a.absolute_tensor[k]+c.absolute_tensor[k];
  }
  AddTensor(b,a.mass,0,a.center);AddTensor(b,c.mass,0,c.center);
  return b;
}
inline void Check(const r::AssemblyRawBody& actual,const Body& q) {
  // Conservative operation-count envelope includes the PART mean, positive
  // mass sums, two center divisions and the optional raw merge. Absolute signed
  // tensor terms bound cancellation; transported center error is separate.
  const Q gamma=Gamma(32*q.terms+256);
  EXPECT_LE(double(Abs(Q(actual.mass)-q.mass)),double(gamma*q.mass));
  Q center_error=0;
  const auto x=Position(actual.center);
  for(unsigned k=0;k<3;++k) {
    const Q bound=gamma*(q.absolute_numerator[k]/q.mass+q.coordinate_scale);
    EXPECT_LE(double(Abs(x[k]-q.center[k])),double(bound));
    center_error=Max(center_error,bound);
  }
  // Each difference magnitude <=2*coordinate_scale. Both diagonal squares and
  // signed off-diagonal products include at most two coordinate-error terms.
  const Q transport=4*q.mass*(4*q.coordinate_scale*center_error+center_error*center_error);
  for(unsigned k=0;k<9;++k)
    EXPECT_LE(double(Abs(Q(actual.tensor.v[k])-q.tensor[k])),
              double(gamma*q.absolute_tensor[k]+transport))<<"tensor channel "<<k;
}
} // namespace rigid_part_model_test::precise
