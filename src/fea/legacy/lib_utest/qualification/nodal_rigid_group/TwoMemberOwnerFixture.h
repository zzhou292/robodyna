#pragma once
#include "TwoMemberFixture.h"
#include "GroupOwnerFixture.h"
namespace rigid_two_owner_test {
namespace rt=rigid_two_test;namespace ro=rigid_owner_test;namespace nt=ro::nt;namespace fe=tl::fea;
using Code=fe::NodalStatus;using Cuda=nt::NodalTemporalCuda;
struct Fixture {
  rt::SourceFixture source;nt::Initial input;
  Fixture() {
    input.n=9;input.h=1./1024;
    for(unsigned n=0;n<8;++n) {const auto& m=source.members[n];
      input.inverse[n]=1/m.mass_kg;input.inverse_inertia[n]=1/m.total_inertia_kg_m2;
      for(unsigned a=0;a<3;++a) {input.x[3*n+a]=rt::Get(m.position,a);
        input.v[3*n+a]=rt::Get(tl::math::Vec3{.3,-.2,.1},a);}
    }
    input.x[24]=2;input.v[24]=-.125;
  }
  fe::NodalReport Initialize(fe::FENodalState& owner,bool capture=false) const {
    fe::NodalStateConfig config;config.node_count=input.n;config.fixed_dt=input.h;
    config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    config.capture_force_stage_accelerations=capture;
    return owner.Initialize(config,{input.x.data(),input.v.data(),input.omega.data(),input.n,input.q.data()},
      input.inverse.data(),{input.fixed.data(),input.rotation_fixed.data(),input.inverse_inertia.data()},source.model);
  }
  nt::Loads Loads(unsigned step=0) const {
    nt::Loads loads;const double scale=step<24?1.:(step<48?-.5:0.);
    for(unsigned n=0;n<8;++n) {
      const tl::math::Vec3 force{scale*.01*(n%2?1:-1),scale*.015,scale*-.003};
      const tl::math::Vec3 couple{scale*2e-5,scale*-3e-5,scale*4e-5};
      for(unsigned a=0;a<3;++a) {loads.force[3*n+a]=rt::Get(force,a);loads.couple[3*n+a]=rt::Get(couple,a);}
    }
    loads.force[24]=4;loads.couple[26]=1;return loads;
  }
};
struct Groups {
  std::array<fe::NodalRigidGroupSnapshot,4> values{};fe::NodalStamp stamp;
  fe::NodalRigidGroupSnapshotBuffer buffer(){return {values.data(),values.size()};}
};
inline bool Read(fe::FENodalState& owner,Groups& out) {
  const auto r=owner.CopyAcceptedRigidGroups(out.buffer(),&out.stamp);
  EXPECT_EQ(r.status,Code::Ok)<<r.message;return r.status==Code::Ok;
}
inline void SameGroups(const Groups& a,const Groups& b) {
  for(unsigned g=0;g<4;++g) {EXPECT_EQ(a.values[g].source_group_id,b.values[g].source_group_id);
    EXPECT_EQ(a.values[g].source_node_set_id,b.values[g].source_node_set_id);
    EXPECT_EQ(rt::Bytes(a.values[g].state),rt::Bytes(b.values[g].state));}
}
inline void SameOwners(fe::FENodalState& a,fe::FENodalState& b) {
  nt::Snapshot x,y;ASSERT_TRUE(nt::Read(a,x));ASSERT_TRUE(nt::Read(b,y));
  x.stamp.owner_id=y.stamp.owner_id;nt::SameState(x,y);
  Groups gx,gy;ASSERT_TRUE(Read(a,gx));ASSERT_TRUE(Read(b,gy));SameGroups(gx,gy);
}
inline tl::math::Vec3 Node(const double* p,unsigned n){return {p[3*n],p[3*n+1],p[3*n+2]};}
} // namespace rigid_two_owner_test
