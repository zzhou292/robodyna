#pragma once
#include "lib_src/collision/NodalWallContactStorage.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_utest/q4_planar_geometry_fixture.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>
#include <string>

namespace nodal_wall_owner_test {
namespace sc=tlfea::contact;
namespace fe=tl::fea;
namespace detail=sc::nodal_wall_device_detail;
using Code=sc::NodalWallDeviceStatus;
constexpr unsigned Capacity=fe::MaxTranslationNodes;
constexpr double ForceBudget=5e-7,EnergyBudget=1.2500000000000005e-12,Arithmetic=2e-12;
constexpr std::uint64_t UnitQualification=0x4e574e4f44455431ULL;
// A contact-only discrete normal-spring unit admission, never BQ4/QEPH or a
// claim that contact is a prescribed constant load. h*sqrt(max k/m)<.01 here.
constexpr double Step=1./1024;
struct Fixture {
  unsigned count=2,n=6;
  std::array<double,3*Capacity> reference{},x{},v{},omega{};
  std::array<double,4*Capacity> q{};
  std::array<double,Capacity> inverse{},inverse_j{};
  std::array<std::uint8_t,Capacity> fixed{},rotation_fixed{};
  sc::SurfaceQ4 parents[2]{{{2,0,1,3},101,201,0,0},{{4,2,3,5},102,202,0,0}};
  sc::Q4ParametricReference ref;
  sc::NodalWallWeights weights;
  q4_planar_test::Wall wall=q4_planar_test::Square();
  sc::PlanarWallBox motion{{0,-1,-.75},{0,1,.75}};
  Fixture() {
    inverse.fill(1); inverse_j.fill(1);
    const double y[6]={-.75,-.75,-.25,-.25,.75,.75};
    for (unsigned i=0;i<Capacity;++i) q[4*i]=1;
    for (unsigned i=0;i<6;++i) {
      reference[3*i]=0; reference[3*i+1]=y[i]; reference[3*i+2]=i%2?-.5:.5;
    }
    x=reference; Depth(1./32);
  }
  void Depth(double depth) { for (unsigned i=0;i<6;++i) x[3*i]=depth; }
  sc::VectorView View(const std::array<double,3*Capacity>& value) const { return {value.data(),n,3,1}; }
  bool Prepare(bool reverse=false) {
    if (ref.Initialize(View(reference),parents,count).status!=sc::Q4ParametricStatus::Ok) return false;
    sc::NodalWallParentInput input[2]{{&ref,0,nullptr},{&ref,1,nullptr}};
    if (reverse && count==2) std::swap(input[0],input[1]);
    return weights.Initialize(n,input,count).status==sc::NodalWallStatus::Ok;
  }
  sc::NodalWallDeviceConfig Config(fe::NodalStamp stamp={}) const {
    if (!stamp.owner_id) {
      stamp.owner_id=77; stamp.node_count=n; stamp.fixed_dt=Step; stamp.has_rotations=true;
      stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    }
    sc::NodalWallDeviceConfig c; c.owner=stamp; c.configuration_id=991; c.qualification_id=UnitQualification;
    c.wall_binding_id=771; c.law={0,16,.5,ForceBudget,EnergyBudget}; return c;
  }
  sc::NodalWallDeviceReport Model(detail::Model* out,sc::NodalWallDeviceConfig config) const {
    return detail::PrepareModel(config,wall.view(),weights,View(x),inverse.data(),fixed.data(),motion,out);
  }
  bool Owner(fe::FENodalState& owner,double h=Step) const {
    fe::NodalStateConfig c; c.node_count=n; c.fixed_dt=h; c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto r=owner.Initialize(c,{x.data(),v.data(),omega.data(),n,q.data()},inverse.data(),
                                   {fixed.data(),rotation_fixed.data(),inverse_j.data()});
    EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; return r.status==fe::NodalStatus::Ok;
  }
  bool Bind(fe::FENodalState& owner,sc::NodalWallContactDevice& contact,sc::NodalWallDeviceConfig config) const {
    config.owner=owner.accepted();
    const auto r=contact.Initialize(config,wall.view(),weights,View(x),inverse.data(),fixed.data(),motion);
    EXPECT_EQ(r.status,Code::Ok)<<r.message; return r.status==Code::Ok;
  }
  sc::NodalWallResult Host(const std::array<double,3*Capacity>& position,
      const std::array<double,3*Capacity>& velocity,std::uint64_t epoch,std::uint64_t attempt,
      sc::NodalWallConfig config) const {
    std::array<std::uint8_t,Capacity> bits{}; for (unsigned i=0;i<n;++i) bits[i]=fixed[i]==7;
    sc::NodalWallResult out;
    const auto r=sc::EvaluateNodalWallContact(weights,View(position),View(velocity),
        {inverse.data(),bits.data(),n,epoch,sc::TranslationMassModel::kIsotropicLumped},config,attempt,&out);
    EXPECT_EQ(r.status,sc::NodalWallStatus::Ok); return out;
  }
};
template<class T> auto Bytes(const T& a) {
  std::array<unsigned char,sizeof(T)> result; std::memcpy(result.data(),&a,sizeof(T)); return result;
}
template<class T> void Unchanged(const T& a,const std::array<unsigned char,sizeof(T)>& expected) {
  EXPECT_EQ(std::memcmp(&a,expected.data(),sizeof(T)),0);
}
inline void Near(double a,long double b,double scale=1) {
  EXPECT_LE(std::abs(static_cast<long double>(a)-b),Arithmetic*(scale+std::abs(b)));
}
inline void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
inline void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
inline void Same(const sc::NodalWallDeviceResults& a,const sc::NodalWallResult& b) {
  ASSERT_TRUE(a.diagnostics.valid); ASSERT_TRUE(b.valid);
  ASSERT_EQ(a.diagnostics.parent_count,b.parent_count); ASSERT_EQ(a.diagnostics.node_count,b.node_count);
  Same(a.diagnostics.resultant,b.resultant); Same(a.diagnostics.potential,b.potential);
  Same(a.diagnostics.wall_reaction,b.wall_reaction); Same(a.diagnostics.wall_moment,b.wall_moment);
  EXPECT_EQ(a.diagnostics.surface_power,b.surface_power);
  for (unsigned i=0;i<b.node_count;++i) {
    const auto& x=a.nodes[i]; const auto& y=b.nodes[i];
    EXPECT_EQ(x.node,y.node); EXPECT_EQ(x.fixed,y.fixed); EXPECT_EQ(x.valid,y.valid);
    EXPECT_EQ(x.touching_or_penetrating,y.touching_or_penetrating);
    EXPECT_EQ(x.base_epoch,y.base_epoch); EXPECT_EQ(x.attempt,y.attempt);
    Same(x.force,y.force); Same(x.potential,y.potential); Same(x.stiffness,y.stiffness);
    Same(x.force_world,y.force_world); Same(x.wall_point,y.wall_point);
    Same(x.wall_reaction,y.wall_reaction); Same(x.wall_moment,y.wall_moment);
    EXPECT_EQ(x.surface_power,y.surface_power); EXPECT_EQ(x.row.count,y.row.count); EXPECT_EQ(x.row.valid,y.row.valid);
    EXPECT_EQ(x.row.base_epoch,y.row.base_epoch); EXPECT_EQ(x.row.attempt,y.row.attempt);
    for (unsigned j=0;j<sc::kMaxNormalNodes;++j) {
      EXPECT_EQ(x.row.nodes[j],y.row.nodes[j]); EXPECT_EQ(x.row.stiffness[j],y.row.stiffness[j]);
      EXPECT_EQ(x.row.damping[j],y.row.damping[j]);
    }
    EXPECT_EQ(x.local_velocity_first_timestep,0);
  }
  for (unsigned p=0;p<b.parent_count;++p) {
    const auto& x=a.parents[p]; const auto& y=b.parents[p];
    EXPECT_EQ(x.parent_element_id,y.parent_element_id); EXPECT_EQ(x.parent_face_id,y.parent_face_id);
    EXPECT_EQ(x.feature_id,y.feature_id); EXPECT_EQ(x.arity,y.arity); EXPECT_EQ(x.family,y.family);
    EXPECT_EQ(x.valid,y.valid); Same(x.resultant,y.resultant); Same(x.potential,y.potential);
    for (unsigned l=0;l<4;++l) Same(x.force[l],y.force[l]);
  }
}
struct Snapshot {
  std::array<double,3*Capacity> x{},v{},omega{},reaction{},couple{};
  std::array<double,4*Capacity> q{};
  fe::NodalStamp stamp;
};
inline Snapshot Read(fe::FENodalState& owner) {
  Snapshot s;
  EXPECT_EQ(owner.CopyAccepted({s.x.data(),s.v.data(),Capacity,s.q.data(),s.omega.data(),
                                s.reaction.data(),s.couple.data()},&s.stamp).status,fe::NodalStatus::Ok);
  return s;
}
inline void Same(const Snapshot& a,const Snapshot& b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.omega,b.omega); EXPECT_EQ(a.q,b.q);
  EXPECT_EQ(a.reaction,b.reaction); EXPECT_EQ(a.couple,b.couple);
  EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id); EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);
  EXPECT_EQ(a.stamp.time,b.stamp.time); EXPECT_EQ(a.stamp.velocity_time,b.stamp.velocity_time);
  EXPECT_EQ(a.stamp.velocity_phase,b.stamp.velocity_phase); EXPECT_EQ(a.stamp.reaction_kick_dt,b.stamp.reaction_kick_dt);
}
} // namespace nodal_wall_owner_test
