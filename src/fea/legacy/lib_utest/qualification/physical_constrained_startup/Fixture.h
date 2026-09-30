// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../physical_publication/OwnerFixture.h"
#include "../type25/EvaluationValues.h"
#include "../type13_recurrence/Fixture.h"
#include "../solid_resident/ResultValues.h"
namespace constrained_startup_test {
namespace fe=tl::fea;
namespace common=physical_publication_test;
inline fe::ShellBatchStartup Startup(bool constrained=true) {
  return {constrained?fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation:
      fe::ShellBatchStartupKind::ReferenceUniformTranslation,{-0.,.5,-1.25}};
}
struct Rig : common::Rig {
  explicit Rig(bool constrained=true)
      : common::Rig(false,2.5,false,common::ContactConstraintLayout::Legacy,false,.05,
                    Startup(constrained),true,true) {}
};
inline void FixIsland(common::Fixture& f) {
  for(std::uint64_t id=20;id<=23;++id) {
    const auto node=f.domain.Find(id);
    ASSERT_LT(node,f.domain.node_count());
    ASSERT_GT(f.m[node],0);ASSERT_GT(f.j[node],0);
    f.fixed[node]=7;f.rotation_fixed[node]=1;f.im[node]=0;f.ij[node]=0;
    const auto v=fe::shell_startup_detail::ProjectVelocity(f.startup.uniform_velocity,7);
    f.v[3*node]=v.x;f.v[3*node+1]=v.y;f.v[3*node+2]=v.z;
  }
}
inline fe::NodalReport InitializeOwner(common::Fixture& f,fe::FENodalState& owner) {
  const auto cin=f.CinStartup();
  return owner.Initialize(f.OwnerConfig(),{f.x.data(),f.v.data(),f.w.data(),f.domain.node_count(),f.q.data()},
      f.im.data(),{f.fixed.data(),f.rotation_fixed.data(),f.ij.data(),f.present.data()},f.rigid,&cin);
}
inline void FixedState(common::Rig& rig) {
  const auto n=rig.fixture.domain.node_count();std::vector<double> x(3*n),v(3*n),q(4*n),w(3*n);
  fe::NodalStamp stamp;
  ASSERT_TRUE(common::Good(rig.owner.CopyAccepted({x.data(),v.data(),n,q.data(),w.data()},&stamp)));
  for(std::uint64_t id=20;id<=23;++id) {
    const auto node=rig.fixture.domain.Find(id);
    for(unsigned k=0;k<3;++k) {
      EXPECT_EQ(common::Bits(x[3*node+k]),common::Bits(rig.fixture.x[3*node+k]));
      EXPECT_EQ(common::Bits(v[3*node+k]),common::Bits(0.));
      EXPECT_EQ(w[3*node+k],0);
    }
    EXPECT_EQ(q[4*node],1);
  }
}
inline fe::NodalValidationReceipt Receipt(const fe::NodalPreparedView& view,bool pass=true) {
  return {view.owner_id,view.kinematics.base_epoch,view.attempt,common::Qualification,pass};
}
inline std::vector<std::uint64_t> MovingCache(common::Rig& rig) {
  std::vector<std::uint64_t> result;
  const auto append=[&](const auto& values){for(double value:values)result.push_back(common::Bits(value));};
  const auto stamp=rig.owner.accepted();
  fe::type25::Evaluation welds[2];fe::type25::BatchDiagnostics wd;
  EXPECT_TRUE(common::Good(rig.welds.CopyAcceptedResults(stamp,welds,2,&wd)));
  for(const auto& value:welds){append(type25_test::EvaluationValues(value));result.push_back(value.history.active);}
  std::vector<fe::type13::Evaluation> springs(rig.fixture.beams.connection_count());
  fe::type13::BatchDiagnostics td;
  EXPECT_TRUE(common::Good(rig.beams.CopyAcceptedResults(stamp,springs.data(),springs.size(),&td)));
  for(const auto& value:springs)append(type13_recurrence_test::Values(value));
  solid_resident_test::Results solids;fe::solids::BatchDiagnostics sd;
  EXPECT_TRUE(common::Good(rig.solids.CopyAcceptedResults(stamp,solids.Buffers(),&sd)));
  append(solid_resident_test::Values(solids.a));append(solid_resident_test::Values(solids.b));
  append(solid_resident_test::Values(solids.c));return result;
}
} // namespace constrained_startup_test
