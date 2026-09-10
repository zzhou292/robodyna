#pragma once
#include "AssemblyFixture.h"

namespace rigid_assembly_test {
struct ScalarBody {long double mass=0;std::array<long double,3> center{};std::array<long double,9> tensor{};};
inline std::array<long double,3> Vector(r::Vec3 x){return {x.x,x.y,x.z};}
inline void Tensor(ScalarBody& out,long double mass,long double j,std::array<long double,3> x) {
  for(unsigned a=0;a<3;++a)x[a]-=out.center[a];
  for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b) {
    if(a==b) {
      long double orbital=0;for(unsigned c=0;c<3;++c)if(c!=a)orbital+=x[c]*x[c];
      out.tensor[3*a+b]+=j+mass*orbital;
    } else out.tensor[3*a+b]-=mass*x[a]*x[b];
  }
}
inline ScalarBody Oracle(const r::AssemblyBodyInput& in) {
  ScalarBody out;out.mass=in.primary.mass;
  const auto origin=Vector(in.primary.position);
  for(unsigned a=0;a<3;++a)out.center[a]=origin[a]*in.primary.mass;
  for(std::size_t i=0;i<in.part_count;++i) {
    const auto p=in.part[i];out.mass+=p.mass;const auto x=Vector(p.position);
    for(unsigned a=0;a<3;++a)out.center[a]+=x[a]*p.mass;
  }
  for(auto& x:out.center)x/=out.mass;
  if(in.extra_count) {
    for(auto& x:out.center)x*=out.mass;
    for(std::size_t i=0;i<in.extra_count;++i) {
      const auto p=in.extra[i];out.mass+=p.mass;const auto x=Vector(p.position);
      for(unsigned a=0;a<3;++a)out.center[a]+=x[a]*p.mass;
    }
    for(auto& x:out.center)x/=out.mass;
  }
  // Source INIRBY's extra-node branch has already replaced XG by the new COM.
  // This is an explicit native branch oracle, not an assertion of exact physical
  // equivalence to retaining a non-negligible primary at its original location.
  Tensor(out,in.primary.mass,in.primary.inertia,in.extra_count?out.center:origin);
  for(std::size_t i=0;i<in.part_count;++i)Tensor(out,in.part[i].mass,in.part[i].inertia,Vector(in.part[i].position));
  for(std::size_t i=0;i<in.extra_count;++i)Tensor(out,in.extra[i].mass,in.extra[i].inertia,Vector(in.extra[i].position));
  return out;
}
inline ScalarBody Merge(const ScalarBody& a,const ScalarBody& b) {
  ScalarBody out;out.mass=a.mass+b.mass;
  for(unsigned i=0;i<3;++i)out.center[i]=(a.mass*a.center[i]+b.mass*b.center[i])/out.mass;
  for(unsigned i=0;i<9;++i)out.tensor[i]=a.tensor[i]+b.tensor[i];
  Tensor(out,a.mass,0,a.center);Tensor(out,b.mass,0,b.center);return out;
}
inline void Compare(const r::AssemblyRawBody& actual,const ScalarBody& expected) {
  Near(actual.mass,expected.mass,expected.mass);
  const auto x=Vector(actual.center);long double position_scale=1;
  for(auto c:expected.center)position_scale=std::max(position_scale,std::abs(c));
  for(unsigned i=0;i<3;++i)Near(x[i],expected.center[i],position_scale);
  long double tensor_scale=0;for(auto c:expected.tensor)tensor_scale=std::max(tensor_scale,std::abs(c));
  for(unsigned i=0;i<9;++i)Near(actual.tensor.v[i],expected.tensor[i],tensor_scale);
}
} // namespace rigid_assembly_test
