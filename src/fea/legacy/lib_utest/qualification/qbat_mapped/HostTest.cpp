// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "../qbat_resident/Fixture.h"

namespace qbat_mapped_test {
TEST(QbatMappedHost,CompleteScrambledPhysicalDomainAndTypedZeroRoles) {
  Fixture f;
  const auto config=f.Config();
  mapped::Forecast forecast;
  ASSERT_EQ(mapped::MakeForecast(config,f.physical,f.Witnesses(),256,forecast).status,qb::BatchStatus::Success);
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(forecast.device.bytes));
  auto* storage=forecast.device.Construct(arena);
  ASSERT_NE(storage,nullptr);
  qb::BatchDiagnostics diagnostics;
  ASSERT_EQ(mapped::BuildModel(config,f.physical,*storage,diagnostics).status,qb::BatchStatus::Success);
  ASSERT_TRUE(storage->model.mapped);
  for (unsigned slot=0;slot<4;++slot) {
    EXPECT_EQ(storage->model.element[0].nodes[slot],f.mechanics.domain.Find(10+slot));
  }
  for (std::size_t node=0;node<config.owner.node_count;++node) {
    EXPECT_EQ(Bits(storage->model.mass[node]),Bits(f.ledger.nodes()[node].coefficients.mass));
    EXPECT_EQ(Bits(storage->model.inertia[node]),Bits(f.ledger.nodes()[node].coefficients.isotropic_inertia));
  }
  EXPECT_EQ(storage->model.mass[f.mechanics.zero_mass],0);
  EXPECT_EQ(storage->model.inertia[f.mechanics.ordinary],0);
  EXPECT_EQ(storage->slab[0].element[0].stamp.sample_index,0u);
  EXPECT_EQ(storage->slab[0].element[0].diagnostics.translation_stiffness_n_m,0);
  const fe::ShellFormulationScope legacy{f.physical.shells(),f.physical.catalog(),f.physical.failure(),nullptr};
  EXPECT_NE(batch::ValidateStartup(config,legacy).status,qb::BatchStatus::Success);
}
TEST(QbatMappedHost,ExactPayloadBudgetAndLateIdentityRetry) {
  Fixture f;
  auto config=f.Config();
  const auto source=f.Witnesses();
  mapped::Forecast baseline;
  ASSERT_EQ(mapped::MakeForecast(config,f.physical,source,256,baseline).status,qb::BatchStatus::Success);
  auto preserved=baseline;
  config.storage_limits.max_host_bytes=baseline.host_bytes-1;
  EXPECT_EQ(mapped::MakeForecast(config,f.physical,source,256,preserved).status,qb::BatchStatus::ResourceLimit);
  EXPECT_EQ(Bytes(preserved),Bytes(baseline));
  config.storage_limits.max_host_bytes=baseline.host_bytes;
  ASSERT_EQ(mapped::MakeForecast(config,f.physical,source,256,preserved).status,qb::BatchStatus::Success);
  auto wrong=source;
  wrong.witness_count=SIZE_MAX;
  EXPECT_NE(mapped::MakeForecast(config,f.physical,wrong,256,preserved).status,qb::BatchStatus::Success);
  EXPECT_EQ(Bytes(preserved),Bytes(baseline));
  EXPECT_EQ(mapped::MakeForecast(config,f.physical,source,256,preserved).status,qb::BatchStatus::Success);
  fe::shell_physical_owner::ProofLayout proof;
  ASSERT_TRUE(fe::shell_physical_owner::ForecastProof(524288,65536,80u<<20,proof));
  EXPECT_EQ(proof.bytes,(15u*524288+2u*65536+1)*sizeof(double));
  EXPECT_FALSE(fe::shell_physical_owner::ForecastProof(SIZE_MAX,65536,SIZE_MAX,proof));
}
TEST(QbatMappedHost,ReadbackRejectsRetainedNonShellProducerAliases) {
  Fixture f;
  qb::BatchDiagnostics output;
  ASSERT_TRUE(fe::shell_physical_owner::OutputDisjoint(f.physical,&output,sizeof(output)));
  const auto nodes=f.ledger.nodes();
  EXPECT_FALSE(fe::shell_physical_owner::OutputDisjoint(f.physical,nodes.data(),sizeof(output)));
  EXPECT_FALSE(fe::shell_physical_owner::OutputDisjoint(f.physical,f.mechanics.domain.nodes().data(),sizeof(double)));
  EXPECT_FALSE(fe::shell_physical_owner::OutputDisjoint(f.physical,f.point.records().data(),sizeof(double)));
  EXPECT_FALSE(fe::shell_physical_owner::OutputDisjoint(f.physical,f.mechanics.springs.references(),sizeof(double)));
  EXPECT_FALSE(fe::shell_physical_owner::OutputDisjoint(f.physical,f.mechanics.solids.parents().data(),sizeof(double)));
  fe::NodalNodeDomain separate;
  ASSERT_TRUE(separate.Initialize({1,f.mechanics.domain.nodes().data(),f.mechanics.domain.node_count()}));
  ASSERT_TRUE(separate.Matches(f.mechanics.domain));
  ASSERT_FALSE(separate.SharesStorage(f.mechanics.domain));
  fe::ElementMassContributions point;
  std::vector<fe::ElementMassSource> records;
  for (const auto& record:f.point.records()) records.push_back(record.source);
  ASSERT_TRUE(point.Initialize(separate,{1,1000,records.data(),records.size()}));
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.InitializeWithSolids({{&f.mechanics.shells,&f.mechanics.springs},&point,&f.mechanics.solids}));
  fe::ShellPhysicalBinding physical;
  ASSERT_TRUE(physical.Initialize({f.physical.shells(),&f.catalog,&f.failure,nullptr},ledger));
  EXPECT_FALSE(fe::shell_physical_owner::OutputDisjoint(physical,separate.nodes().data(),sizeof(double)));
}
TEST(QbatMappedHost,VirginCurrentRemovedStiffnessAndLateOverflowPreservation) {
  qbat_force_test::Fixture source;
  const auto element=qbat_resident_test::Element(source);
  qb::BatchResult virgin;
  ASSERT_EQ(batch::InitializeResult(element,virgin),qb::Status::kSuccess);
  mapped::NodalStiffness initial;
  ASSERT_TRUE(mapped::AcceptedStiffness(element,virgin,initial));
  EXPECT_GT(initial.translation[0],0);
  EXPECT_EQ(initial.rotation[0],0);
  EXPECT_GT(element.reference.coefficients().nodal_rotation_stiffness_nm,0);
  EXPECT_EQ(virgin.diagnostics.translation_stiffness_n_m,0);
  auto current=virgin;
  current.stamp.sample_index=1;
  current.diagnostics.translation_stiffness_n_m=123;
  current.kinematics.nodal_factor[0]=.8;
  current.kinematics.nodal_factor[1]=1;
  mapped::NodalStiffness packet;
  ASSERT_TRUE(mapped::AcceptedStiffness(element,current,packet));
  EXPECT_EQ(packet.translation[0],123*.8);
  EXPECT_EQ(packet.translation[2],123*.8);
  EXPECT_EQ(packet.translation[1],123);
  current.diagnostics.translation_stiffness_n_m=0;
  current.history.element_active=false;
  ASSERT_TRUE(mapped::AcceptedStiffness(element,current,packet));
  for (double x:packet.translation) EXPECT_EQ(x,0);
  const std::size_t nodes[4]{3,1,0,4};
  std::array<double,5> translation{1,2,3,4,std::numeric_limits<double>::max()},rotation{};
  const auto original=translation;
  packet.translation[3]=std::numeric_limits<double>::max();
  EXPECT_FALSE(mapped::AddStiffness(nodes,packet,translation.data(),rotation.data(),translation.size()));
  EXPECT_EQ(translation,original);
  packet.translation[3]=0;
  ASSERT_TRUE(mapped::AddStiffness(nodes,packet,translation.data(),rotation.data(),translation.size()));
}
} // namespace qbat_mapped_test
