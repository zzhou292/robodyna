// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../tied_cin_runtime/OwnerFixture.h"
#include "../nodal_rigid_group/GroupOwnerFixture.h"

namespace rotation_presence_test {
namespace fe=tl::fea;
using Code=fe::NodalStatus;
TEST(RotationPresenceCuda, RigidRuntimePreservesSeparateOrdinaryTranslationOnlyNode) {
  rigid_owner_test::Fixture f;
  std::vector<std::uint8_t> present(f.input.n,1); present.back()=0;
  f.input.inverse_inertia[f.input.n-1]=0;
  fe::NodalStateConfig config;
  config.node_count=f.input.n; config.fixed_dt=f.input.h;
  config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  auto init=[&](fe::FENodalState& owner) {
    return owner.Initialize(config,{f.input.x.data(),f.input.v.data(),f.input.omega.data(),
      f.input.n,f.input.q.data()},f.input.inverse.data(),
      {f.input.fixed.data(),f.input.rotation_fixed.data(),f.input.inverse_inertia.data(),present.data()},f.model);
  };
  fe::FENodalState owner,invalid;
  ASSERT_EQ(init(owner).status,Code::Ok);
  auto load=f.Load(); load.couple[3*(f.input.n-1)+2]=0;
  for(unsigned step=0;step<3;++step) ASSERT_TRUE(rigid_owner_test::Step(owner,load));
  tl_test::nodal_temporal::Snapshot out;
  ASSERT_TRUE(tl_test::nodal_temporal::Read(owner,out));
  const auto node=f.input.n-1;
  EXPECT_NEAR(out.v[3*node],-.125+2.5*f.input.h*2,2e-15);
  EXPECT_EQ(out.omega[3*node+2],0); EXPECT_EQ(out.couple[3*node+2],0);
  present[0]=0; f.input.inverse_inertia[0]=0;
  EXPECT_EQ(init(invalid).status,Code::InvalidInput);
  EXPECT_EQ(invalid.allocations().device_allocations,0u);
}
TEST(RotationPresenceCuda, CinTransfersKeepAnUnrelatedSolidNodeTranslationOnly) {
  namespace cr=cin_runtime_test;
  cr::Fixture f;
  auto nodes=f.source.nodes;
  nodes.push_back({999,{0,0,2}});
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({73,nodes.data(),nodes.size()}));
  cr::tied::TiedCinAttachmentModel model;
  ASSERT_TRUE(cr::tied::PrepareCinAttachments(f.source.post,domain,f.source.Input(),&model));
  const auto n=nodes.size(),extra=n-1;
  f.x.resize(3*n); f.x[3*extra+2]=2;
  f.velocity.resize(3*n); f.omega.resize(3*n); f.q.resize(4*n); f.q[4*extra]=1;
  f.mass.push_back(2); f.inertia.push_back(0); f.inverse.push_back(.5); f.inverse_j.push_back(0);
  f.fixed.push_back(0); f.stif.push_back(0); f.stifr.push_back(0);
  f.load.assign(6*n,0); f.load[extra]=4;
  std::vector<std::uint8_t> present(n,1); present.back()=0;
  auto dofs=f.Dofs(); dofs.rotation_present=present.data();
  auto startup=f.Startup(); startup.model=&model;
  fe::FENodalState owner,invalid;
  ASSERT_EQ(owner.Initialize(f.Config(),f.Kinematics(),f.inverse.data(),dofs,startup).status,Code::Ok);
  cr::Snapshot initial(n,f.rows.size()); fe::NodalStamp initial_stamp;
  ASSERT_EQ(owner.CopyAcceptedCin(initial.Cin(),&initial_stamp).status,Code::Ok);
  EXPECT_EQ(initial_stamp.epoch,0u); EXPECT_EQ(initial.coefficients[n+extra],0);
  for(unsigned step=0;step<3;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalCinAssemblyView cin;
    cr::Fill(owner,f,token,view,cin);
    ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
    ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,cr::Admission(view)).status,Code::Ok);
    cr::Snapshot candidate(n,f.rows.size()); fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.CopyPreparedCin(token,candidate.Cin(),&prepared).status,Code::Ok);
    EXPECT_EQ(candidate.coefficients[n+extra],0);
    cr::Complete(owner,token,view);
    ASSERT_EQ(owner.Commit(token).status,Code::Ok);
  }
  cr::Snapshot result(n,f.rows.size()); fe::NodalStamp stamp;
  cr::Accepted(owner,result,stamp);
  EXPECT_NEAR(result.nodes[3*n+3*extra],5e-6,2e-20);
  EXPECT_EQ(result.coefficients[n+extra],0);
  EXPECT_EQ(result.nodes[6*n+3*extra+2],0); EXPECT_EQ(result.nodes[16*n+3*extra+2],0);
  present[f.rows[0].secondary]=0;
  EXPECT_EQ(invalid.Initialize(f.Config(),f.Kinematics(),f.inverse.data(),dofs,startup).status,Code::InvalidInput);
  EXPECT_EQ(invalid.allocations().device_allocations,0u);
}
} // namespace rotation_presence_test
