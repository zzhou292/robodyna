#pragma once
#include "GroupObservationFixture.h"
namespace rigid_observation_test {
// Independent scalar long-double construction in WORLD coordinates. No
// production dot/cross/tensor/kinetic helpers or new eigensolver is used.
using Triple=std::array<long double,3>;
using Tensor=std::array<long double,9>;
inline Triple L(Vec3 a) { return {a.x,a.y,a.z}; }
inline long double Norm(Triple a) { long double s=0; for(auto x:a) s+=x*x; return s; }
inline Triple Cross(Triple a,Triple b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline Triple WorldArm(const rigid::GroupKineticInput& in,Vec3 reference) {
  const auto& g=*in.metric.group; const auto x=L(reference),c=L(g.center); Triple body{},world{};
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) body[i]+=static_cast<long double>(g.principal.axes.v[3*j+i])*(x[j]-c[j]);
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) world[i]+=static_cast<long double>(in.group.principal_axes.v[3*i+j])*body[j];
  return world;
}
inline void AddTensor(Tensor& t,long double mass,long double j,Triple arm) {
  for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b)
    t[3*a+b]+=(a==b?j+mass*Norm(arm):0)-mass*arm[a]*arm[b];
}
struct KineticOracle {
  Tensor tensor{};
  long double native_translation=0,native_rotation=0,physical_rotation=0,added_rotation=0;
  long double translation=0,rotation=0,orbital=0,native_spin=0,physical_spin=0,added_spin=0;
  long double primary_translation=0,primary_parallel=0,primary_isotropic=0,correction=0;
};
inline KineticOracle Oracle(const rigid::GroupKineticInput& in) {
  KineticOracle out; const auto& g=*in.metric.group; const auto w=L(in.group.omega),v=L(in.group.velocity);
  for(unsigned i=0;i<in.metric.member_count;++i) {
    const auto& m=in.metric.members[i]; const auto arm=WorldArm(in,m.position); const auto velocity=L(in.members[i].velocity);
    const auto omega=L(in.members[i].omega);
    out.native_translation+=.5L*m.mass_kg*Norm(velocity); out.native_rotation+=.5L*m.total_inertia_kg_m2*Norm(omega);
    out.physical_rotation+=.5L*m.physical_inertia_kg_m2*Norm(omega); out.added_rotation+=.5L*m.added_inertia_kg_m2*Norm(omega);
    out.orbital+=.5L*m.mass_kg*Norm(Cross(w,arm)); out.native_spin+=.5L*m.total_inertia_kg_m2*Norm(w);
    out.physical_spin+=.5L*m.physical_inertia_kg_m2*Norm(w); out.added_spin+=.5L*m.added_inertia_kg_m2*Norm(w);
    AddTensor(out.tensor,m.mass_kg,m.total_inertia_kg_m2,arm);
  }
  const auto primary_arm=WorldArm(in,g.generated_primary_position);
  const auto& reg=g.regularization;
  AddTensor(out.tensor,reg.primary_mass_kg,reg.primary_isotropic_inertia_kg_m2,primary_arm);
  out.primary_translation=.5L*reg.primary_mass_kg*Norm(v);
  out.primary_parallel=.5L*reg.primary_mass_kg*Norm(Cross(w,primary_arm));
  out.primary_isotropic=.5L*reg.primary_isotropic_inertia_kg_m2*Norm(w);
  for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b) for(unsigned k=0;k<3;++k) {
    const long double addition=rigid_test::Get(reg.principal_inertia_added,k);
    const long double value=static_cast<long double>(in.group.principal_axes.v[3*a+k])*addition*in.group.principal_axes.v[3*b+k];
    out.tensor[3*a+b]+=value; out.correction+=.5L*w[a]*value*w[b];
  }
  for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b) out.rotation+=.5L*w[a]*out.tensor[3*a+b]*w[b];
  out.translation=.5L*g.total_mass_kg*Norm(v);
  return out;
}
inline void Compare(const rigid::GroupKineticObservation& actual,const KineticOracle& ref) {
  const auto& n=actual.members; const auto& g=actual.aggregate;
  Near(n.translation,ref.native_translation); Near(n.native_rotation,ref.native_rotation);
  Near(n.physical_rotation,ref.physical_rotation); Near(n.added_rotation,ref.added_rotation);
  Near(n.total,ref.native_translation+ref.native_rotation);
  Near(g.translation,ref.translation); Near(g.rotation,ref.rotation); Near(g.total,ref.translation+ref.rotation);
  Near(g.member_orbital_rotation,ref.orbital); Near(g.native_member_rotation,ref.native_spin);
  Near(g.physical_member_rotation,ref.physical_spin); Near(g.added_member_rotation,ref.added_spin);
  Near(g.primary_translation,ref.primary_translation); Near(g.primary_parallel_axis_rotation,ref.primary_parallel);
  Near(g.primary_isotropic_rotation,ref.primary_isotropic); Near(g.principal_correction_rotation,ref.correction);
  EXPECT_LE(std::abs(g.decomposition_residual),g.decomposition_roundoff_budget);
}
} // namespace rigid_observation_test
