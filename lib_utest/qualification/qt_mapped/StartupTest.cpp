// SPDX-License-Identifier: MIT
#include "../qbat_mapped/Fixture.h"
#include "../shell_execution/Fixture.h"
#include "lib_src/elements/qeph/mapped/Startup.h"
#include "lib_src/elements/t3/mapped/Startup.h"
#include "lib_src/elements/qeph/mapped/Result.h"
#include "lib_src/elements/t3/mapped/Result.h"

namespace qt_mapped_test {
namespace fe=tl::fea;
using qbat_binding_test::Bits;
using qbat_binding_test::Bytes;
template<class Config> Config Configuration(const qbat_mapped_test::Fixture& source,std::size_t count) {
  Config config;
  config.owner=source.Config().owner;
  config.configuration_id=81;
  config.qualification_id=82;
  config.element_count=count;
  config.usage=decltype(config.usage)::CoupledForces;
  return config;
}
template<class Storage> void SameLedger(const qbat_mapped_test::Fixture& source,const Storage& storage) {
  const auto values=source.ledger.nodes();
  for (std::size_t node=0;node<values.size();++node) {
    EXPECT_EQ(Bits(storage.model.mass[node]),Bits(values[node].coefficients.mass));
    EXPECT_EQ(Bits(storage.model.inertia[node]),Bits(values[node].coefficients.isotropic_inertia));
  }
  EXPECT_EQ(storage.model.mass[source.mechanics.zero_mass],0);
  EXPECT_EQ(storage.model.inertia[source.mechanics.ordinary],0);
}
TEST(QtMappedStartup, CompleteScrambledOwnerAndNativeFamilyIndices) {
  qbat_mapped_test::Fixture source;
  auto q=Configuration<fe::qeph::QephBatchConfig>(source,2);
  auto t=Configuration<fe::t3::T3BatchConfig>(source,1);
  fe::qeph::mapped::Forecast qf;
  fe::t3::mapped::Forecast tf;
  ASSERT_EQ(fe::qeph::mapped::MakeForecast(q,source.physical,source.Witnesses(),{},256,qf).status,fe::qeph::BatchStatus::Success);
  ASSERT_EQ(fe::t3::mapped::MakeForecast(t,source.physical,source.Witnesses(),{},256,tf).status,fe::t3::BatchStatus::Success);
  tl::util::HostArena qa,ta;
  ASSERT_TRUE(qa.Initialize(qf.device.bytes));
  ASSERT_TRUE(ta.Initialize(tf.device.bytes));
  auto* qs=qf.device.Construct(qa);
  auto* ts=tf.device.Construct(ta);
  ASSERT_NE(qs,nullptr);
  ASSERT_NE(ts,nullptr);
  ASSERT_EQ(fe::qeph::mapped::BuildModel(q,source.physical,*qs).status,fe::qeph::BatchStatus::Success);
  ASSERT_EQ(fe::t3::mapped::BuildModel(t,source.physical,*ts).status,fe::t3::BatchStatus::Success);
  SameLedger(source,*qs);
  SameLedger(source,*ts);
  for (std::size_t parent=0;parent<2;++parent) {
    for (unsigned slot=0;slot<4;++slot) {
      EXPECT_EQ(qs->model.element[parent].nodes[slot],source.mechanics.domain.Find(
          qs->model.element[parent].reference.input.node_ids[slot]));
    }
    EXPECT_TRUE(fe::qeph::mapped::ValidResult(qs->model.element[parent].reference,qs->slab[0].element[parent],0,0,false));
  }
  for (unsigned slot=0;slot<3;++slot) {
    EXPECT_EQ(ts->model.element[0].nodes[slot],source.mechanics.domain.Find(
        ts->model.element[0].reference.input.node_ids[slot]));
  }
  EXPECT_TRUE(fe::t3::mapped::ValidResult(ts->model.element[0].reference,ts->slab[0].element[0],0,0,false));
}
TEST(QtMappedStartup, ExactCombinedBudgetAndWrongRosterPreserveForecast) {
  qbat_mapped_test::Fixture source;
  auto config=Configuration<fe::t3::T3BatchConfig>(source,1);
  fe::t3::mapped::Forecast baseline;
  ASSERT_EQ(fe::t3::mapped::MakeForecast(config,source.physical,source.Witnesses(),{},256,baseline).status,fe::t3::BatchStatus::Success);
  auto output=baseline;
  config.storage_limits.max_host_bytes=baseline.host_bytes-1;
  EXPECT_EQ(fe::t3::mapped::MakeForecast(config,source.physical,source.Witnesses(),{},256,output).status,fe::t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(Bytes(output),Bytes(baseline));
  config.storage_limits.max_host_bytes=baseline.host_bytes;
  ASSERT_EQ(fe::t3::mapped::MakeForecast(config,source.physical,source.Witnesses(),{},256,output).status,fe::t3::BatchStatus::Success);
  auto invalid=source.Witnesses();
  invalid.witness_count=SIZE_MAX;
  EXPECT_NE(fe::t3::mapped::MakeForecast(config,source.physical,invalid,{},256,output).status,fe::t3::BatchStatus::Success);
  EXPECT_EQ(Bytes(output),Bytes(baseline));
  config.max_device_bytes=baseline.device.bytes;
  EXPECT_EQ(fe::t3::mapped::MakeForecast(config,source.physical,source.Witnesses(),{},256,output).status,fe::t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(Bytes(output),Bytes(baseline));
}
TEST(QtMappedStartup, ExplicitRigidSkinAndConstitutiveLayersShareExactTopology) {
  shell_execution_test::Fixture source;
  auto materials=source.source.base.materials;
  materials[1].curve_id=0;
  materials[1].hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
  materials[1].linear={10e6,0};
  materials[1].rate=materials[2].rate;
  auto input=source.source.Catalog();
  input.materials=materials.data();
  input.curves=nullptr;
  input.curve_count=0;
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeExecutionCatalog(source.shells,input).status,fe::ShellPlasticityBindingStatus::Success);
  auto rows=source.source.Failures();
  for (auto& row:rows) if (row.source.material_id==1001) {
    row.policy=fe::ShellFailurePolicy::Tab1AnyPoint;
    row.constant={};
    row.tab1.table={{-1,0,1},1};
  }
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.InitializeExecution(catalog,rows.data(),rows.size()).status,fe::ShellPlasticityBindingStatus::Success);
  fe::ShellExecutionBinding execution;
  ASSERT_EQ(execution.Initialize(catalog,source.ledger,source.rigid).status,fe::ShellPlasticityBindingStatus::Success);
  fe::ShellPhysicalBinding physical;
  ASSERT_TRUE(physical.InitializeExecution({&source.shells,&catalog,&failure,nullptr},source.ledger,execution));
  fe::t3::T3BatchConfig config;
  config.owner.node_count=source.domain.node_count();
  config.element_count=source.shells.t3_count();
  fe::t3::batch_detail::Layout layout;
  ASSERT_TRUE(layout.Initialize(config.element_count,config.owner.node_count,1u<<20));
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(layout.bytes));
  auto* storage=layout.Construct(arena);
  ASSERT_NE(storage,nullptr);
  ASSERT_EQ(fe::t3::mapped::BuildModel(config,physical,*storage).status,fe::t3::BatchStatus::Success);
  for (std::size_t row=0;row<config.element_count;++row) {
    const auto* role=execution.parent(fe::ShellBindingFamily::T3,row);
    ASSERT_NE(role,nullptr);
    EXPECT_TRUE(fe::t3::mapped::ValidResult(storage->model.element[row].reference,
        storage->slab[0].element[row],0,0,role->law==fe::ShellSectionLaw::RigidSkin));
  }
  EXPECT_EQ(execution.parent(fe::ShellBindingFamily::T3,0)->material_points,0u);
  EXPECT_EQ(execution.parent(fe::ShellBindingFamily::T3,2)->material_points,1u);
  const auto skin=fe::ShellBatchLayeredSection::RigidSkin();
  EXPECT_EQ(skin.law(),fe::ShellSectionLaw::RigidSkin);
  EXPECT_EQ(skin.plastic(),nullptr);
  EXPECT_EQ(skin.elastic(),nullptr);
  EXPECT_EQ(skin.one_point(),nullptr);
}
} // namespace qt_mapped_test
