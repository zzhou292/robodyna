// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "ConstructorChecks.h"
#include "lib_utest/qualification/solid_model/OriginalFixture.h"
#include "lib_utest/qualification/solid18_law44_force/TestSupport.h"
#include "lib_utest/qualification/law90_solid18_reference/SourceFixture.h"
#include "lib_utest/qualification/law90_preparation/TestSupport.h"
#include "RearOriginalFixture.h"
namespace extended_resident_test {
TEST(ExtendedResidentOriginal, All4063ConstructorsKeepActualRearAndBlankHuFoamNativeValues) {
  solid_model_test::OriginalFixture old;
  std::map<std::uint64_t,tl::math::Vec3> unique;
  for(const auto& node:old.nodes)unique.emplace(node.source_id,node.position);
  auto collect=[&](const auto& reference) {
    const auto& input=reference.input();
    for(unsigned n=0;n<8;++n) {
      const auto added=unique.emplace(input.source_node_id[n],input.position_m[n]);
      if(!added.second)EXPECT_TRUE(fe::shell_startup_detail::SameVector(added.first->second,input.position_m[n]));
    }
  };
  std::vector<s::Input18Law44> rear;
  std::vector<s::Input18Law90> foam;
  for(unsigned i=0;i<std::size(rear18_test::original::Cells);++i) {
    const auto input=rear18_test::original::Input(i);s::Input18Law44 value;
    ASSERT_EQ(fe::solid18::law44::InitializeReference(input,value.reference),fe::solid18::Status::Success);
    value.material=rear_force_test::OriginalParameters(input.source_part_id);
    collect(value.reference);rear.push_back(value);
  }
  const auto foam_input=law90_test::OriginalBlankHuInput();
  tl::material::law90::PreparedMaterial material;
  ASSERT_EQ(tl::material::law90::PrepareSI(foam_input,law90_test::OriginalBlankHuCurve(),material),tl::material::law90::Status::Ok);
  ASSERT_EQ(material.reader().loading_flag,1);
  for(unsigned i=0;i<law90_reference_test::fixture::element_count;++i) {
    auto input=law90_reference_test::Original(i);
    // Qualified native SDI density interpretation, explicit alongside canonical fixture geometry.
    input.density_kg_m3=foam_input.density_kg_m3;
    s::Input18Law90 value;value.material=material;
    ASSERT_EQ(fe::solid18::total_strain::InitializeReference90(input,value.reference),fe::solid18::Status::Success);
    collect(value.reference);foam.push_back(value);
  }
  std::vector<fe::NodalDomainNode> nodes;
  for(auto it=unique.rbegin();it!=unique.rend();++it)nodes.push_back({it->first,it->second});
  fe::NodalNodeDomain domain;ASSERT_TRUE(domain.Initialize({777,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
  s::Model model;ASSERT_TRUE(model.Initialize(domain,{777,{old.a.data(),old.a.size()},
      {old.b.data(),old.b.size()},{old.c.data(),old.c.size()},{rear.data(),rear.size()},
      {foam.data(),foam.size()},s::ModelProfile::ExtendedLaw44Law90}));
  auto config=Config(domain.node_count());config.startup.uniform_velocity={15.6464,0,0};
  s::BatchForecast forecast;ASSERT_TRUE(s::Batch::Forecast(config,model,forecast));
  s::Batch batch;auto report=batch.InitializeJoined(config,model);ASSERT_TRUE(report)<<report.message;
  Results results(model);s::BatchDiagnostics diagnostics;
  report=s::BatchQualificationPeer::ReadConstructed(batch,results.Buffers(),diagnostics);ASSERT_TRUE(report)<<report.message;
  unsigned repeated=0;
  for(unsigned i=0;i<rear.size();++i) {
    const auto& parent=model.solid18_law44()[i];SCOPED_TRACE(parent.reference.input().source_element_id);
    ASSERT_TRUE(RearConstructor(parent,model.materials44()[parent.material_index].value,config.startup.uniform_velocity,results.rear[i]));
    repeated+=parent.reference.topology()==fe::solid18::law44::SourceTopology::RepeatedPairs56And78;
  }
  for(unsigned i=0;i<foam.size();++i) {
    const auto& parent=model.solid18_law90()[i];SCOPED_TRACE(parent.reference.input().source_element_id);
    ASSERT_TRUE(FoamConstructor(parent,model.materials90()[parent.material_index].value,foam_input,
      config.startup.uniform_velocity,results.foam[i]));
  }
  EXPECT_EQ(repeated,109u);EXPECT_EQ(results.rear.size(),306u);EXPECT_EQ(results.foam.size(),1345u);
  const std::size_t counts[]{908,1309,195,306,1345};
  for(unsigned f=0;f<5;++f)EXPECT_EQ(diagnostics.parent_count[f],counts[f]);
  RecordProperty("parents",4063);RecordProperty("rear_native_constructors",306);
  RecordProperty("foam_iflag1_native_constructors",1345);RecordProperty("repeated_rear",repeated);
  RecordProperty("device_bytes",forecast.device_bytes);RecordProperty("startup_host_bytes",forecast.startup_host_bytes);
}
} // namespace extended_resident_test
