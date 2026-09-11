// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../nodal/NodalTemporalFixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace rotation_presence_test {
namespace nt=tl_test::nodal_temporal;
namespace fe=tl::fea;
using Code=fe::NodalStatus;
struct Fixture {
  nt::Initial in;
  std::array<std::uint8_t,nt::Capacity> present;
  Fixture() {
    in.n=3; in.h=.01; present.fill(1);
    in.rotation_fixed[1]=1; in.inverse_inertia[1]=0;
    present[2]=0; in.inverse_inertia[2]=0; in.v[6]=1;
  }
  fe::NodalReport Init(fe::FENodalState& owner,std::size_t cap=fe::MaxTranslationDeviceBytes) const {
    fe::NodalStateConfig config;
    config.node_count=in.n; config.fixed_dt=in.h; config.max_device_bytes=cap;
    config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    return owner.Initialize(config,{in.x.data(),in.v.data(),in.omega.data(),in.n,in.q.data()},
      in.inverse.data(),{in.fixed.data(),in.rotation_fixed.data(),in.inverse_inertia.data(),present.data()});
  }
};
TEST(RotationPresenceCuda, IndependentTranslationHasNoSpinOrInventedReactionAcrossHalfKicks) {
  Fixture f; fe::FENodalState owner;
  ASSERT_EQ(f.Init(owner).status,Code::Ok);
  ASSERT_TRUE(owner.accepted().has_rotation_presence);
  const auto allocations=owner.allocations();
  nt::Loads loads;
  loads.force[6]=4; loads.couple[2]=2; loads.couple[5]=3;
  for(unsigned k=1;k<=3;++k) {
    ASSERT_TRUE(nt::StepConstant(owner,loads));
    nt::Snapshot value; ASSERT_TRUE(nt::Read(owner,value));
    EXPECT_NEAR(value.x[6],k*f.in.h+.5*k*k*f.in.h*f.in.h*2,2e-15);
    EXPECT_NEAR(value.v[6],1+(k-.5)*f.in.h*2,2e-15);
    EXPECT_EQ(value.omega[8],0); EXPECT_EQ(value.couple[8],0);
    EXPECT_EQ(value.couple[5],-3); // A genuinely fixed rotation has a reaction.
    EXPECT_NEAR(value.omega[2],(k-.5)*f.in.h*.5,2e-15);
    EXPECT_EQ(value.q[8],1); EXPECT_EQ(value.q[9],0); EXPECT_EQ(value.q[10],0); EXPECT_EQ(value.q[11],0);
  }
  EXPECT_EQ(owner.allocations().device_bytes,allocations.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocations.device_allocations);
}
TEST(RotationPresenceCuda, UnexpectedLateCoupleRejectsWholeTrialAndExactRetry) {
  Fixture f; fe::FENodalState owner;
  ASSERT_EQ(f.Init(owner).status,Code::Ok);
  nt::Snapshot before,after; ASSERT_TRUE(nt::Read(owner,before));
  nt::Loads loads; loads.force[6]=4; loads.couple[8]=1;
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_TRUE(nt::BeginLoad(owner,loads,token,view));
  ASSERT_NE(view.rotation_present,nullptr);
  auto forged=view; forged.rotation_present=nullptr;
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(forged).status,Code::StaleTrial);
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  const auto report=fe::AdvanceStaggeredPrescribed(owner,token,nt::Admission(owner,view));
  EXPECT_EQ(report.status,Code::InvalidOutput); EXPECT_EQ(report.node,2u);
  ASSERT_TRUE(nt::Read(owner,after));
  nt::SameState(before,after);
  EXPECT_TRUE(fe::trial_identity::SameStamp(before.stamp,after.stamp));
  loads.couple[8]=0;
  ASSERT_TRUE(nt::StepConstant(owner,loads));
  ASSERT_TRUE(nt::Read(owner,after));
  EXPECT_EQ(after.stamp.epoch,1u);
  EXPECT_NEAR(after.x[6],f.in.h+f.in.h*f.in.h,2e-15);
  EXPECT_EQ(after.couple[8],0);
}
TEST(RotationPresenceCuda, ExplicitAdmissionRequiresZeroInertiaSpinAndSeparateFixedMeaning) {
  for(unsigned error=0;error<4;++error) {
    Fixture f; fe::FENodalState owner;
    switch(error) {
      case 0:f.present[2]=2;break;
      case 1:f.in.inverse_inertia[2]=1;break;
      case 2:f.in.omega[8]=.1;break;
      case 3:f.in.rotation_fixed[2]=1;break;
    }
    EXPECT_EQ(f.Init(owner).status,Code::InvalidInput);
    EXPECT_EQ(owner.allocations().device_allocations,0u);
    Fixture good;
    ASSERT_EQ(good.Init(owner).status,Code::Ok);
  }
  Fixture f; fe::FENodalState measured,rejected,exact,legacy;
  ASSERT_EQ(f.Init(measured).status,Code::Ok);
  const auto bytes=measured.allocations().device_bytes;
  EXPECT_EQ(f.Init(rejected,bytes-1).status,Code::ResourceLimit);
  ASSERT_EQ(f.Init(exact,bytes).status,Code::Ok);
  EXPECT_EQ(measured.allocations().device_allocations,6u);
  EXPECT_EQ(f.in.Initialize(legacy).status,Code::InvalidInput); // Omission cannot silently admit zero J.
}
} // namespace rotation_presence_test
