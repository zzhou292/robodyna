// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../type45_model/Fixture.h"
#include "lib_src/elements/type45/resident/Contexts.h"
#include "lib_src/elements/type45/resident/Storage.h"

namespace type45_model_test {
namespace rd=joint::resident_detail;
struct Data {
  Fixture fixture;
  joint::Model model;
  joint::BatchConfig config;
  std::vector<fe::NodalCinPhysicalMain> mains;
  Data() {
    const auto input=Inputs(fixture);
    const auto report=model.Initialize(fixture.binding,{fixture.domain.source_instance_id(),{input.data(),input.size()}});
    EXPECT_TRUE(report)<<report.message;
    config.owner.owner_id=901;config.owner.node_count=fixture.domain.node_count();
    config.owner.fixed_dt=1e-4;config.owner.has_rotations=true;
    config.owner.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    config.owner.velocity_phase=fe::NodalVelocityPhase::Collocated;
    auto& info=config.owner.rigid_groups;
    info.source_instance_id=903;info.group_count=fixture.binding.groups().size();
    info.member_count=fixture.binding.members().size();
    config.configuration_id=910;config.qualification_id=911;
    config.profile=joint::BatchProfile::PhysicalAggregateV1;
    config.cin_attachment_count=2;config.cin_witness_count=2;
    for(const auto& g:fixture.binding.groups()) {
      const auto i=g.principal.inertia;
      mains.push_back({g.source_kind,g.source_id,g.source_node_set_id,g.center,g.mass_kg,
        ::fmin(i.x,::fmin(i.y,i.z)),120,4});
    }
  }
};
TEST(Type45ResidentLayout,CountsAndExactCapsPrecedeInputReadsAndPreserveOutputs) {
  Data d;rd::ArenaLayout layout;
  ASSERT_TRUE(rd::Plan(d.config,d.model,layout));
  EXPECT_EQ(layout.slab[0].count,3u);EXPECT_EQ(layout.mains.count,d.mains.size());
  auto cap=d.config;cap.limits.max_device_bytes=layout.bytes;
  rd::ArenaLayout retry;ASSERT_TRUE(rd::Plan(cap,d.model,retry));
  --cap.limits.max_device_bytes;retry.bytes=777;
  EXPECT_FALSE(rd::Plan(cap,d.model,retry));EXPECT_EQ(retry.bytes,777u);
  joint::BatchForecast total;ASSERT_TRUE(joint::Batch::Forecast(d.config,d.model,total));
  EXPECT_EQ(total.startup_host_bytes,total.retained_model_backing_bytes+total.incremental_host_bytes);
  EXPECT_EQ(total.retained_model_backing_bytes,d.model.owned_payload_bytes()-sizeof(joint::Model));
  cap=d.config;cap.limits.max_host_bytes=total.startup_host_bytes;
  ASSERT_TRUE(joint::Batch::Forecast(cap,d.model,total));
  --cap.limits.max_host_bytes;total.device_bytes=999;
  EXPECT_FALSE(joint::Batch::Forecast(cap,d.model,total));EXPECT_EQ(total.device_bytes,999u);
  EXPECT_FALSE(rd::MakeLayout(SIZE_MAX,d.config,retry));EXPECT_EQ(retry.bytes,777u);
}
TEST(Type45ResidentLayout,ForecastAndReadbackRejectEveryRetainedAuthorityRange) {
  Data d;
  const auto& rigid=*d.model.rigid_binding();
  const auto& ledger=*rigid.coefficients();
  const auto check=[&](const void* address) {
    ASSERT_NE(address,nullptr);
    EXPECT_FALSE(rd::ModelOutputDisjoint(d.model,address,sizeof(joint::BatchForecast)));
  };
  check(&d.model);check(d.model.joints().data());
  check(rigid.groups().data());check(rigid.members().data());
  check(ledger.nodes().data());check(ledger.domain()->nodes().data());
  check(ledger.shells()->mapping().data());check(ledger.shells()->shells()->nodes().data());
  if(ledger.type25()) check(ledger.type25()->references());
  if(ledger.type13()) check(ledger.type13()->records().data());
  if(ledger.element_mass()) check(ledger.element_mass()->records().data());
  if(ledger.solids()) check(ledger.solids()->parents().data());
  // The public forecast must reject a borrowed output before its final store.
  const auto before=ledger.nodes()[0].coefficients.mass;
  auto* overlap=reinterpret_cast<joint::BatchForecast*>(
      const_cast<fe::NodalCoefficientNode*>(ledger.nodes().data()));
  EXPECT_FALSE(joint::Batch::Forecast(d.config,d.model,*overlap));
  EXPECT_EQ(ledger.nodes()[0].coefficients.mass,before);
  joint::BatchForecast output;EXPECT_TRUE(joint::Batch::Forecast(d.config,d.model,output));
}
TEST(Type45ResidentLayout,UploadAndReadbackStorageOwnTypedLifetimesWithoutAliasingSlabs) {
  Data d;rd::ArenaLayout layout;ASSERT_TRUE(rd::Plan(d.config,d.model,layout));
  tl::util::HostArena upload;ASSERT_TRUE(upload.Initialize(layout.bytes));
  ASSERT_TRUE(rd::BuildUpload(d.config,d.model,upload,layout));
  const auto device=rd::RebasedHeader(upload.data(),layout);
  EXPECT_NE(device.slab[0],device.slab[1]);
  EXPECT_EQ(device.count,3u);
  for(std::size_t j=0;j<device.count;++j) {
    EXPECT_EQ(device.joints[j].geometry.source_joint_id,d.model.joints()[j].geometry.source_joint_id);
    ASSERT_EQ(rd::InitializeState(device.joints[j],device.slab[0][j]),joint::Status::Success);
    EXPECT_FALSE(device.slab[1][j].history.ready());
  }
  static_assert(!std::is_copy_constructible_v<joint::Batch>);
  static_assert(!std::is_copy_constructible_v<tl::util::HostArena>);
  static_assert(std::is_trivially_copyable_v<rd::State>);
}
TEST(Type45ResidentContext,CompleteSourceOrderAndActualCinIdentityRejectLateMismatchBeforePublishing) {
  Data d;fe::NodalPreparedView view;view.attempt=7;
  fe::NodalCinPhysicalMainStamp receipt{fe::NodalCinPhysicalMainPolicy::PhysicalAggregateV1,
    d.config.owner.owner_id,0,7,912,0,d.config.owner.fixed_dt,d.config.owner.rigid_groups};
  std::array<joint::AutomaticStiffnessContext,3> output;
  for(auto& c:output) c.target_dt_s=777;
  const auto check=[&] {return rd::Contexts(d.model,d.config.owner,view,912,receipt,
      d.mains.data(),d.mains.size(),output.data());};
  const auto saved=d.mains.back();++d.mains.back().source_group_id;
  EXPECT_FALSE(check());for(const auto& c:output) EXPECT_EQ(c.target_dt_s,777);
  d.mains.back()=saved;
  receipt.cin_qualification_id=d.config.qualification_id;EXPECT_FALSE(check());
  receipt.cin_qualification_id=912;ASSERT_TRUE(check());
  for(std::size_t j=0;j<output.size();++j) for(unsigned e=0;e<2;++e) {
    const auto& source=d.mains[d.model.joints()[j].body_groups[e]];
    EXPECT_EQ(output[j].main[e].source_body_id,source.source_group_id);
    EXPECT_EQ(output[j].main[e].inertia_kg_m2,source.minimum_principal_inertia_kg_m2);
    EXPECT_EQ(output[j].main[e].translational_stiffness_n_m,120);
  }
  // The two bodies share numeric source ID but keep distinct native roles.
  ASSERT_NE(d.mains[0].source_kind,d.mains[1].source_kind);
  std::swap(d.mains[0],d.mains[1]);EXPECT_FALSE(check());
  std::swap(d.mains[0],d.mains[1]);ASSERT_TRUE(check());
}
} // namespace type45_model_test
