// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GeneralFixture.h"
#include "FullLedgerRig.h"
#include "MixedRuntimeFixture.h"
#include "lib_src/collision/radioss_type25/runtime/InitialSeed.h"
#include <gtest/gtest.h>
#include <cstring>
#include <array>
#include <algorithm>
#include <cmath>
namespace type25_general_test {
namespace {
std::uint64_t Bits(double x){std::uint64_t out;std::memcpy(&out,&x,sizeof(out));return out;}
void SameRow(const n::NativeGeometryHistory& a,const n::NativeGeometryHistory& b) {
  EXPECT_EQ(a.secondary_source_id,b.secondary_source_id);EXPECT_EQ(a.generation,b.generation);
  for(unsigned k=0;k<4;++k)EXPECT_EQ(a.row.irtlm[k],b.row.irtlm[k]);
  const auto scalars=[](const n::NativeContactRow& r) {
    return std::array<double,15>{r.history.normal.previous_penetration,r.history.normal.previous_stiffness,
      r.history.normal.staged_penetration,r.history.normal.staged_stiffness,r.history.normal.damping_half_force,
      r.history.previous_force.x,r.history.previous_force.y,r.history.previous_force.z,
      r.history.staged_force.x,r.history.staged_force.y,r.history.staged_force.z,r.penetration_auxiliary,r.penetration_offset,
      r.selection_metric[0],r.selection_metric[1]};
  };
  const auto x=scalars(a.row),y=scalars(b.row);for(unsigned k=0;k<x.size();++k)EXPECT_EQ(Bits(x[k]),Bits(y[k]));
}
void SameInitial(const n::runtime_qualification::InitializationObservation& actual,const ReadSeed::Result& expected) {
  for(unsigned slab=0;slab<2;++slab) {
    ASSERT_EQ(actual.rows[slab].size(),expected.history.size());ASSERT_EQ(actual.secondary[slab].size(),expected.flags.size());
    for(std::size_t row=0;row<expected.history.size();++row){SameRow(actual.rows[slab][row],expected.history[row]);EXPECT_EQ(actual.secondary[slab][row].initial_contact_flag,expected.flags[row]);}
  }
  ASSERT_EQ(4*actual.mains.size(),expected.gaps.size());
  for(std::size_t m=0;m<actual.mains.size();++m)for(unsigned k=0;k<4;++k)EXPECT_EQ(Bits(actual.mains[m].gap[k]),Bits(expected.gaps[4*m+k]));
}
ReadSeed::Result Expected(const is::PreparedSource& prepared,tl::fea::FENodalState& owner,std::uint64_t& warm,is::Diagnostics* diagnostics=nullptr) {
  cudaStream_t stream=nullptr;moving_cache_test::Check(owner.BorrowOwnerStream(&stream));is::DeviceSeed seed;
  const auto report=is::Prepare(prepared,stream,seed);
  if(report.status!=is::Status::Ok)throw std::runtime_error("Owning expected private seed production failed");
  warm=report.diagnostics.warm_after_tied;if(diagnostics)*diagnostics=report.diagnostics;return ReadSeed::Read(seed,stream);
}
}
TEST(NativeGeneralInitialize, GenuineWarmSeedUsesBothExistingSlabsAndOnePhysicalClock) {
  Source source;
  ASSERT_NO_THROW(source.Prepare());
  EXPECT_FALSE(source.rig.owner.accepted().owner_id);
  n::GeneralTransactionForecast forecast;
  auto limits=moving_cache_test::Rig::Limits();
  ASSERT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,
      source.rig.source.physical.physical,forecast,limits).status,n::TransactionStatus::Ok);
  EXPECT_EQ(forecast.peak_device_bytes,std::max(forecast.transaction.device_bytes,
      forecast.transaction.runtime_device_bytes+forecast.initializer.peak_device_bytes));
  EXPECT_LT(forecast.peak_device_bytes,forecast.transaction.device_bytes+forecast.initializer.peak_device_bytes);
  EXPECT_FALSE(source.rig.owner.accepted().owner_id);
  limits.max_device_bytes=forecast.peak_device_bytes;limits.max_host_bytes=forecast.peak_host_bytes;
  ASSERT_NO_THROW(source.InitializeOwner());
  std::uint64_t warm=0;const auto expected=Expected(source.prepared,source.rig.owner,warm);ASSERT_GT(warm,0u);
  ASSERT_EQ(source.InitializeContact(limits).status,n::TransactionStatus::Ok);
  const auto diagnostics=source.rig.contact.initialization_diagnostics();ASSERT_TRUE(diagnostics.available);
  EXPECT_EQ(diagnostics.values.warm_after_tied,warm);EXPECT_EQ(diagnostics.identity.source.topology,7u);
  EXPECT_EQ(diagnostics.identity.source.runtime_topology,3u);
  ASSERT_NO_THROW(source.BindRoster());
  n::runtime_qualification::InitializationObservation observed;
  ASSERT_TRUE(n::runtime_qualification::Access::ReadInitialization(source.rig.contact,&observed));SameInitial(observed,expected);
  EXPECT_EQ(source.rig.owner.accepted().epoch,0u);EXPECT_EQ(source.rig.contact.allocations().device_bytes,forecast.transaction.device_bytes);
  moving_cache_test::Attempt attempt;ASSERT_NO_THROW(source.rig.Begin(attempt));
  ASSERT_EQ(source.rig.contact.AssembleAccepted(source.rig.owner,attempt.token,attempt.assembly).status,n::TransactionStatus::Ok);
  ASSERT_NO_THROW(source.rig.Prepare(attempt));
  ASSERT_EQ(source.rig.Commit(attempt).status,tl::fea::ShellPublicationStatus::Success);
  EXPECT_EQ(source.rig.owner.accepted().epoch,1u);EXPECT_TRUE(source.rig.contact.accepted().available);
  std::vector<n::NativeGeometryHistory> before(source.runtime.selection.secondary_count),after(before.size());
  std::vector<int> before_flags(before.size()),after_flags(before.size());tl::fea::NativeContactPublicationSnapshot before_stamp,after_stamp;
  ASSERT_EQ(source.rig.contact.CopyAccepted({before.data(),before_flags.data(),before.size()},&before_stamp).status,n::TransactionStatus::Ok);
  const auto before_position=source.rig.Positions();
  moving_cache_test::Attempt rejected;ASSERT_NO_THROW(source.rig.Begin(rejected));
  ASSERT_EQ(source.rig.contact.AssembleAccepted(source.rig.owner,rejected.token,rejected.assembly).status,n::TransactionStatus::Ok);
  ASSERT_GT(source.rig.contact.last_diagnostics().active_forces,0u);
  ASSERT_NO_THROW(source.rig.Prepare(rejected));
  EXPECT_NE(source.rig.Commit(rejected,false).status,tl::fea::ShellPublicationStatus::Success);source.rig.Discard();
  ASSERT_EQ(source.rig.contact.CopyAccepted({after.data(),after_flags.data(),after.size()},&after_stamp).status,n::TransactionStatus::Ok);
  EXPECT_EQ(source.rig.owner.accepted().epoch,1u);EXPECT_EQ(source.rig.Positions(),before_position);
  for(std::size_t row=0;row<before.size();++row){SameRow(after[row],before[row]);EXPECT_EQ(after_flags[row],before_flags[row]);}
  moving_cache_test::Attempt retry;ASSERT_NO_THROW(source.rig.Begin(retry));
  ASSERT_EQ(source.rig.contact.AssembleAccepted(source.rig.owner,retry.token,retry.assembly).status,n::TransactionStatus::Ok);
  ASSERT_GT(source.rig.contact.last_diagnostics().active_forces,0u);
  ASSERT_NO_THROW(source.rig.Prepare(retry));
  ASSERT_EQ(source.rig.Commit(retry).status,tl::fea::ShellPublicationStatus::Success);EXPECT_EQ(source.rig.owner.accepted().epoch,2u);
}
TEST(NativeGeneralInitialize, CoexistenceCapsAndWrongSourceRejectBeforeOwnerAllocation) {
  Source source;ASSERT_NO_THROW(source.Prepare());
  auto limits=moving_cache_test::Rig::Limits();n::GeneralTransactionForecast exact;
  ASSERT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,source.rig.source.physical.physical,exact,limits).status,n::TransactionStatus::Ok);
  auto result=exact;limits.max_device_bytes=exact.peak_device_bytes-1;
  EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,source.rig.source.physical.physical,result,limits).status,n::TransactionStatus::ResourceLimit);
  EXPECT_EQ(result.peak_device_bytes,exact.peak_device_bytes);
  limits=moving_cache_test::Rig::Limits();limits.max_host_bytes=exact.peak_host_bytes-1;
  EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,source.rig.source.physical.physical,result,limits).status,n::TransactionStatus::ResourceLimit);
  limits=moving_cache_test::Rig::Limits();auto runtime=source.runtime;++runtime.topology_generation;
  EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,runtime,source.prepared,source.rig.source.physical.physical,result,limits).status,n::TransactionStatus::SourceMismatch);
  runtime=source.runtime;runtime.selection.generation++;
  EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,runtime,source.prepared,source.rig.source.physical.physical,result,limits).status,n::TransactionStatus::SourceMismatch);
  auto config=source.rig.config;config.units.length_m=.001;
  EXPECT_EQ(n::Transaction::GeneralPreflight(config,source.runtime,source.prepared,source.rig.source.physical.physical,result,limits).status,n::TransactionStatus::SourceMismatch);
  runtime=source.runtime;runtime.margin=std::nextafter(runtime.margin,1.);
  EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,runtime,source.prepared,source.rig.source.physical.physical,result,limits).status,n::TransactionStatus::SourceMismatch);
  EXPECT_EQ(result.peak_device_bytes,exact.peak_device_bytes);EXPECT_FALSE(source.rig.owner.accepted().owner_id);
}
TEST(NativeGeneralInitialize, SequentialDevicePeakBranchesAdmitExactCapsAndRejectOneByteShort) {
  for(bool seed_dominant:{false,true}) {
    SCOPED_TRACE(seed_dominant);
    Source source;auto initial=Source::InitialLimits();auto limits=moving_cache_test::Rig::Limits();
    if(seed_dominant){initial.max_tasks=32768;initial.max_pairs=32768;}
    else {limits.inventory.max_tasks=32768;limits.inventory.max_pairs=32768;}
    ASSERT_NO_THROW(source.Prepare(source.Input(),initial));
    n::GeneralTransactionForecast plan;
    ASSERT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,
        source.rig.source.physical.physical,plan,limits).status,n::TransactionStatus::Ok);
    const auto overlap=plan.transaction.runtime_device_bytes+plan.initializer.peak_device_bytes;
    if(seed_dominant)ASSERT_GT(overlap,plan.transaction.device_bytes);
    else ASSERT_GT(plan.transaction.device_bytes,overlap);
    EXPECT_EQ(plan.peak_device_bytes,std::max(plan.transaction.device_bytes,overlap));
    EXPECT_LT(plan.peak_device_bytes,plan.transaction.device_bytes+plan.initializer.peak_device_bytes);
    EXPECT_EQ(plan.peak_host_bytes,plan.transaction.startup_host_bytes+plan.initializer.retained_host_bytes);
    ASSERT_NO_THROW(source.InitializeOwner());
    const auto initial_owner=source.rig.owner.accepted();
    auto exact=limits;exact.max_device_bytes=plan.peak_device_bytes;exact.max_host_bytes=plan.peak_host_bytes;
    for(bool host_short:{false,true}) {
      SCOPED_TRACE(host_short);
      auto short_limit=exact;
      if(host_short)--short_limit.max_host_bytes;else --short_limit.max_device_bytes;
      auto unchanged=plan;
      EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,
          source.rig.source.physical.physical,unchanged,short_limit).status,n::TransactionStatus::ResourceLimit);
      EXPECT_EQ(unchanged.peak_device_bytes,plan.peak_device_bytes);EXPECT_EQ(unchanged.peak_host_bytes,plan.peak_host_bytes);
      EXPECT_EQ(source.InitializeContact(short_limit).status,n::TransactionStatus::ResourceLimit);
      EXPECT_FALSE(source.rig.contact.source_info().available);EXPECT_FALSE(source.rig.contact.initialization_diagnostics().available);
      EXPECT_EQ(source.rig.owner.accepted().owner_id,initial_owner.owner_id);EXPECT_EQ(source.rig.owner.accepted().epoch,0u);
    }
    std::uint64_t warm=0;const auto expected=Expected(source.prepared,source.rig.owner,warm);ASSERT_GT(warm,0u);
    ASSERT_EQ(source.InitializeContact(exact).status,n::TransactionStatus::Ok);
    EXPECT_EQ(source.rig.contact.allocations().device_bytes,plan.transaction.device_bytes);
    ASSERT_NO_THROW(source.BindRoster());
    n::runtime_qualification::InitializationObservation observed;
    ASSERT_TRUE(n::runtime_qualification::Access::ReadInitialization(source.rig.contact,&observed));SameInitial(observed,expected);
    moving_cache_test::Attempt attempt;
    ASSERT_NO_THROW(source.rig.Begin(attempt));
    ASSERT_EQ(source.rig.contact.AssembleAccepted(source.rig.owner,attempt.token,attempt.assembly).status,n::TransactionStatus::Ok);
    ASSERT_NO_THROW(source.rig.Prepare(attempt));
    ASSERT_EQ(source.rig.Commit(attempt).status,tl::fea::ShellPublicationStatus::Success);
    EXPECT_EQ(source.rig.owner.accepted().epoch,1u);
  }
}
TEST(NativeGeneralInitialize, NumericalPreparationDoesNotInventRuntimeHandoffAuthority) {
  Source source;auto input=source.Input();input.engine_handoff=is::EngineHandoff::Unspecified;
  ASSERT_NO_THROW(source.Prepare(input));
  n::GeneralTransactionForecast output;output.peak_device_bytes=987;
  EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,source.rig.source.physical.physical,output,moving_cache_test::Rig::Limits()).status,n::TransactionStatus::UnsupportedProfile);
  EXPECT_EQ(output.peak_device_bytes,987u);EXPECT_FALSE(source.rig.owner.accepted().owner_id);
  Source other;auto foreign=other.Input();++foreign.stamp.physical_domain;
  ASSERT_NO_THROW(other.Prepare(foreign));
  EXPECT_EQ(n::Transaction::GeneralPreflight(other.rig.config,other.runtime,other.prepared,other.rig.source.physical.physical,output,moving_cache_test::Rig::Limits()).status,n::TransactionStatus::SourceMismatch);
  EXPECT_EQ(output.peak_device_bytes,987u);EXPECT_FALSE(other.rig.owner.accepted().owner_id);
}
TEST(NativeGeneralInitialize, ExistingColdInitializerRemainsExplicitlyUnavailableForSeedDiagnostics) {
  moving_cache_test::Rig legacy;ASSERT_NO_THROW(legacy.Initialize());
  EXPECT_FALSE(legacy.contact.initialization_diagnostics().available);
  n::runtime_qualification::InitializationObservation actual;
  ASSERT_TRUE(n::runtime_qualification::Access::ReadInitialization(legacy.contact,&actual));
  for(unsigned slab=0;slab<2;++slab)for(const auto& row:actual.rows[slab]) {
    for(auto key:row.row.irtlm)EXPECT_EQ(key,0);
    EXPECT_EQ(Bits(row.row.penetration_offset),Bits(0.));EXPECT_EQ(Bits(row.row.history.normal.staged_stiffness),Bits(0.));
  }
}
TEST(NativeGeneralInitialize, FixedReadyPhaseIsAuthenticatedSeparatelyFromStarterAndRetryPreservesOwner) {
  FixedSource source;ASSERT_NO_THROW(source.Prepare());
  auto& rig=source.base.rig;n::GeneralTransactionForecast forecast;
  ASSERT_EQ(n::Transaction::GeneralPreflight(rig.config,source.runtime,source.ready,source.prepared,
      rig.source.physical.physical,forecast,moving_cache_test::Rig::Limits()).status,n::TransactionStatus::Ok);
  ASSERT_NO_THROW(source.base.InitializeOwner());
  auto wrong=source.ready;wrong.normals=source.starter.starter;bool different=false;
  for(std::size_t i=0;i<wrong.normals.reference_count;++i)different=different||wrong.normals.references[i].boundary!=source.ready.normals.references[i].boundary;
  ASSERT_TRUE(different);
  auto report=rig.contact.GeneralInitialize(rig.config,source.runtime,wrong,source.prepared,rig.owner,rig.publication,
      rig.source.physical.physical,rig.Participants(),rig.Identity(),moving_cache_test::Rig::Limits());
  EXPECT_EQ(report.status,n::TransactionStatus::SourceMismatch);EXPECT_FALSE(rig.contact.source_info().available);EXPECT_EQ(rig.owner.accepted().epoch,0u);
  std::uint64_t warm=0;const auto expected=Expected(source.prepared,rig.owner,warm);ASSERT_GT(warm,0u);
  report=rig.contact.GeneralInitialize(rig.config,source.runtime,source.ready,source.prepared,rig.owner,rig.publication,
      rig.source.physical.physical,rig.Participants(),rig.Identity(),moving_cache_test::Rig::Limits());
  ASSERT_EQ(report.status,n::TransactionStatus::Ok);
  ASSERT_NO_THROW(source.base.BindRoster());
  n::runtime_qualification::InitializationObservation observed;
  ASSERT_TRUE(n::runtime_qualification::Access::ReadInitialization(rig.contact,&observed));SameInitial(observed,expected);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}
TEST(NativeGeneralInitialize, GenuineMixedSupportsAndCinRosterBindCompletePhysicalOwner) {
  for(bool negative:{false,true}) {
  SCOPED_TRACE(negative);
  type25_source_test::FullLedgerRig rig(true);
  ASSERT_NO_THROW(rig.Initialize(false));
  type25_source_test::MixedRuntimeSource mixed(rig.fixture);auto& f=rig.fixture;
  // This synthetic source explicitly selects every physical node for NSV,
  // independently of the older fixture's T3-only mechanical load coupon.
  f.secondary.clear();for(std::size_t i=0;i<f.nodes.size();++i)f.secondary.push_back({std::uint32_t(i),1e6,.001,0});
  std::sort(f.secondary.begin(),f.secondary.end(),[&](auto a,auto b){return f.nodes[a.node].source_id<f.nodes[b.node].source_id;});
  f.removal_offsets.assign(f.secondary.size()+1,0);
  if(negative) {
    for(auto& main:f.mains){main.coefficient=-1e6;main.maximum_gap=.0012;for(auto& gap:main.gap)gap=.0012;}
    for(auto& row:f.secondary)row.gap=.0012;
  }
  auto runtime=mixed.Source();st::Input mesh;mesh.profile=st::Profile::MixedSurface;mesh.topology=st::TopologyPolicy::NativeMixedSurface;
  mesh.node_source_ids=f.ids.data();mesh.node_count=f.ids.size();mesh.positions={f.positions.data(),std::uint32_t(f.ids.size()),3,1};
  mesh.primary=f.primary.data();mesh.primary_count=f.primary.size();mesh.source_generation=mixed.starter.source_generation;
  mesh.primary_identities=mixed.identities.data();mesh.primary_identity_count=mixed.identities.size();mesh.shell_primary_count=2;
  mesh.raw_origins=mixed.origins.data();mesh.raw_origin_count=mixed.origins.size();mesh.raw_origin_to_primary=mixed.origin_map.data();
  std::vector<is::EightSlotSolid> solids;
  const auto add=[&](const auto& physical,unsigned corners) {
    is::EightSlotSolid solid;solid.native_source_id=physical.source_element_id;solid.part_source_id=physical.source_part_id;
    constexpr unsigned penta[]{0,1,2,0,3,4,5,3};
    for(unsigned k=0;k<8;++k){const auto slot=corners==6?penta[k]:k;const auto node=f.domain.Find(physical.source_node_id[slot]);
      if(node==SIZE_MAX)throw std::runtime_error("Missing authentic raw solid node");solid.nodes[k]=std::uint32_t(node);}
    solids.push_back(solid);
  };
  add(f.source.a,8);add(f.source.b,8);add(f.source.c,6);
  n::tied_removal::Main tied_main;const auto attachment=rig.cin.rows().data[0];
  for(unsigned k=0;k<4;++k)tied_main.nodes[k]=std::uint32_t(attachment.master_domain_nodes[k]);
  const n::tied_removal::Row tied_row{std::uint32_t(attachment.secondary_domain_node),1,0};
  const n::tied_removal::Interface tie{881,1,28,&tied_main,1,&tied_row,1};
  const is::InterfaceIdentity interfaces[]{{881,1,is::InterfaceKind::Type2,is::InterfaceOrigin::OriginalDefinition},
    {201,2,is::InterfaceKind::Type25,is::InterfaceOrigin::OriginalDefinition}};
  std::vector<double> gaps(f.mains.size(),negative?.0012:.001);std::vector<std::uint32_t> main_nodes;std::vector<bool> seen(f.nodes.size());
  for(std::size_t m=0;m<mesh.primary_count;++m)for(auto node:mixed.starter.mains[m].nodes)
    if(!seen[node]){seen[node]=true;main_nodes.push_back(node);}
  is::Input input;input.phase=is::Phase::StarterNormalsAndPreBucGaps;
  input.stamp={runtime.source_id,runtime.selection.generation,f.domain.source_instance_id(),runtime.topology_generation};input.units={1,1,1};
  input.engine_handoff=is::EngineHandoff::SourceProvedFreshSerialSearchAtZero;
  input.controls={1,1,5,1,1,0,1,1,0,0,2,1,0,4,0,false,double(.20f),0,0,8000000,0,128};
  input.interface_phase=is::InterfaceCensusPhase::CompleteOriginalAndDeclaredAdditions;input.interfaces=interfaces;input.interface_count=2;input.native_interface_id=201;
  input.mesh=mesh;input.starter=mixed.starter;input.contact=runtime.selection;input.contact.removed_main_by_secondary={};
  input.main_nodes=main_nodes.data();input.main_node_count=main_nodes.size();input.main_search_gap=gaps.data();input.main_search_gap_count=gaps.size();input.global_search_gap=negative?.0012+.0012:.001+.001;
  input.solid_scope=is::SolidScope::CompleteEightSlotModel;input.solids=solids.data();input.solid_count=solids.size();
  input.contributors={n::search_startup::Census::CompleteDeclaredModel,f.nodes.size(),f.shells.qeph_count()+f.shells.t3_count()+f.shells.qbat_count(),1,0,1};
  input.tied_phase=n::tied_removal::Finalization::CompactedAfterKinChk;input.tied_interfaces=&tie;input.tied_interface_count=1;
  is::PreparedSource prepared;ASSERT_EQ(is::PrepareSource(input,Source::InitialLimits(),prepared).status,is::Status::Ok);
  runtime.selection.removed_main_by_secondary={};runtime.margin=prepared.removals().engine_margin;runtime.primary_curvature=prepared.removals().primary_extent;
  n::GeneralTransactionForecast plan;ASSERT_EQ(n::Transaction::GeneralPreflight(mixed.Config(),runtime,prepared,f.physical,plan).status,n::TransactionStatus::Ok);
  std::uint64_t warm=0;is::Diagnostics diagnostics;const auto expected=Expected(prepared,rig.owner,warm,&diagnostics);
  const auto report=rig.contact.GeneralInitialize(mixed.Config(),runtime,prepared,rig.owner,rig.publication,f.physical,rig.Participants(),rig.Identity());
  if(negative) {
    ASSERT_GT(diagnostics.warm_negative_main,0u);EXPECT_EQ(report.status,n::TransactionStatus::UnsupportedProfile);
    EXPECT_EQ(rig.owner.accepted().epoch,0u);EXPECT_FALSE(rig.contact.source_info().available);
    EXPECT_FALSE(rig.contact.initialization_diagnostics().available);
    for(const auto& main:f.mains)EXPECT_EQ(main.coefficient,-1e6);
    continue;
  }
  ASSERT_EQ(report.status,n::TransactionStatus::Ok)<<report.message;
  ASSERT_EQ(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,f.physical,rig.Participants(),rig.Identity(),
      {{},rig.contact.roster_entry()}).status,tl::fea::ShellPublicationStatus::Success);
  n::runtime_qualification::InitializationObservation actual;ASSERT_TRUE(n::runtime_qualification::Access::ReadInitialization(rig.contact,&actual));SameInitial(actual,expected);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);EXPECT_EQ(rig.contact.source_info().expanded_mains,5u);
  EXPECT_EQ(rig.contact.initialization_diagnostics().identity.primaries,3u);
  }
}
TEST(NativeGeneralInitialize, GenuineOrderedMsrControlsMaintenanceBudgetAndRejectsIncompletePrivateRoster) {
  Source source;
  // Explicit source-order coupon: no planner may silently sort this complete
  // roster. The producer owns the declared order through the General handoff.
  std::rotate(source.main_nodes.begin(),source.main_nodes.begin()+1,source.main_nodes.end());
  ASSERT_NO_THROW(source.Prepare());
  auto limits=moving_cache_test::Rig::Limits();
  const auto roles=source.runtime.selection.secondary_count+source.main_nodes.size();
  ASSERT_LT(roles,source.runtime.selection.secondary_count+4*source.runtime.primary_main_count);
  limits.maintenance.max_role_entries=roles;
  n::GeneralTransactionForecast forecast;
  ASSERT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,
      source.rig.source.physical.physical,forecast,limits).status,n::TransactionStatus::Ok);
  auto legacy=source.runtime;legacy.selection.removed_main_by_secondary=source.prepared.removals().by_secondary;
  n::TransactionForecast legacy_forecast;
  EXPECT_EQ(n::Transaction::Preflight(source.rig.config,legacy,source.rig.source.physical.physical,legacy_forecast,limits).status,n::TransactionStatus::ResourceLimit);
  namespace rd=n::runtime_detail;
  rd::Plan plan(limits);n::MovingMainSource bound;n::GeneralTransactionForecast exact;
  ASSERT_EQ(rd::PrepareGeneralPlan(source.rig.config,source.runtime,source.prepared,source.rig.source.physical.physical,
      limits,n::runtime_qualification::Access::FixedHostBytes(),plan,bound,exact).status,n::TransactionStatus::Ok);
  EXPECT_EQ(plan.upload.main_nodes,source.main_nodes);EXPECT_EQ(plan.upload.maintenance.mains,source.main_nodes.size());
  EXPECT_EQ(plan.upload.maintenance.main_nodes,plan.upload.main_nodes.data());
  for(unsigned mode:{0u,1u,2u}) {
    SCOPED_TRACE(mode);
    auto invalid=source.main_nodes;
    if(mode==0)invalid.back()=invalid.front();
    if(mode==1)invalid.pop_back();
    if(mode==2)invalid.back()=std::uint32_t(source.runtime.selection.node_count);
    rd::Plan rejected(limits);
    EXPECT_EQ(rd::PreparePlan(source.rig.config,bound,source.rig.source.physical.physical,limits,
        n::runtime_qualification::Access::FixedHostBytes(),rejected,{invalid.data(),invalid.size()}).status,n::TransactionStatus::SourceMismatch);
  }
  auto short_limit=limits;--short_limit.maintenance.max_role_entries;auto untouched=forecast;
  EXPECT_EQ(n::Transaction::GeneralPreflight(source.rig.config,source.runtime,source.prepared,
      source.rig.source.physical.physical,untouched,short_limit).status,n::TransactionStatus::ResourceLimit);
  EXPECT_EQ(untouched.peak_device_bytes,forecast.peak_device_bytes);
  ASSERT_NO_THROW(source.InitializeOwner());
  ASSERT_EQ(source.InitializeContact(limits).status,n::TransactionStatus::Ok);
  EXPECT_EQ(source.rig.contact.allocations().maintenance_device_bytes,forecast.transaction.maintenance_device_bytes);
}
}
