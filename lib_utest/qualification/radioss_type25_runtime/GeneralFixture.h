// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "MovingCacheRig.h"
#include "lib_src/collision/RadiossType25InitialState.h"
#include "lib_utest/qualification/radioss_type25_initial_source/Access.h"
namespace type25_general_test {
namespace n=tlfea::contact::radioss_type25;
namespace is=n::initial_source;
namespace st=n::startup;
using ReadSeed=n::qualification::InitialSourceAccess;
// Genuine physical source and lower-level Starter/initial-state producers.
// The small synthetic coefficient/gap packet is an explicit test operand; this
// helper makes no vehicle parser or reference-trajectory assertion.
struct Source {
  moving_cache_test::Rig rig;
  st::Input mesh;
  std::vector<double> main_gap;
  std::vector<std::uint32_t> main_nodes;
  is::InterfaceIdentity interface{201,1,is::InterfaceKind::Type25,is::InterfaceOrigin::OriginalDefinition};
  is::PreparedSource prepared;
  n::MovingMainSource runtime;
  explicit Source():rig(true) {
    auto& source=rig.source;
    for(auto& main:source.mains){main.maximum_gap=.0012;for(auto& gap:main.gap)gap=.0012;}
    for(auto& row:source.secondary)row.gap=.0012;
    mesh.profile=source.starter.profile;mesh.topology=source.starter.topology;mesh.coordinates=st::Coordinates::Native;
    mesh.source_generation=source.starter.source_generation;mesh.node_source_ids=source.ids.data();mesh.node_count=source.ids.size();
    mesh.positions={source.physical.positions.data(),std::uint32_t(source.ids.size()),3,1};mesh.units=rig.config.units;
    mesh.primary=source.primary.data();mesh.primary_count=source.primary.size();
    std::vector<bool> seen(mesh.node_count);
    for(const auto& face:source.primary)for(auto node:face.nodes)if(!seen[node]){seen[node]=true;main_nodes.push_back(node);}
    main_gap.assign(source.mains.size(),.0012);
    runtime=source.Source();runtime.contact_thickness_update=0;
  }
  is::Input Input() const {
    is::Input in;in.phase=is::Phase::StarterNormalsAndPreBucGaps;
    in.stamp={runtime.source_id,runtime.selection.generation,rig.source.physical.domain.source_instance_id(),runtime.topology_generation};
    in.units=rig.config.units;in.engine_handoff=is::EngineHandoff::SourceProvedFreshSerialSearchAtZero;
    in.controls={1,1,5,1,1,0,1,1,0,0,2,1,0,4,0,false,double(.20f),0,0,8000000,0,128};
    in.interface_phase=is::InterfaceCensusPhase::CompleteOriginalAndDeclaredAdditions;
    in.interfaces=&interface;in.interface_count=1;in.native_interface_id=interface.source_id;
    in.mesh=mesh;in.starter=rig.source.starter;in.contact=runtime.selection;in.contact.removed_main_by_secondary={};
    in.main_nodes=main_nodes.data();in.main_node_count=main_nodes.size();in.main_search_gap=main_gap.data();in.main_search_gap_count=main_gap.size();
    in.global_search_gap=.0012+.0012;in.solid_scope=is::SolidScope::ExplicitNoSolids;
    in.contributors={n::search_startup::Census::CompleteDeclaredModel,mesh.node_count,mesh.primary_count};return in;
  }
  static is::Limits InitialLimits() {
    is::Limits limits;limits.max_tasks=128;limits.max_pairs=128;limits.max_host_bytes=16u<<20;limits.max_device_bytes=16u<<20;
    limits.geometric.max_removals=128;limits.tied.search=limits.geometric;return limits;
  }
  void Prepare(is::Input input) {
    const auto report=is::PrepareSource(input,InitialLimits(),prepared);
    if(report.status!=is::Status::Ok)throw std::runtime_error("Genuine initial source preparation failed");
    runtime.margin=prepared.removals().engine_margin;runtime.primary_curvature=prepared.removals().primary_extent;
    runtime.selection.removed_main_by_secondary={}; // General binder uses the producer's exact final CSR.
  }
  void Prepare(){Prepare(Input());}
  void InitializeOwner() {
    auto& r=rig;auto& f=r.source.physical;moving_cache_test::Check(f.Initialize(r.owner));
    tl::fea::qeph::QephBatchConfig q;q.owner=r.owner.accepted();q.configuration_id=901;q.qualification_id=f.Qualification;
    q.element_count=f.shells.qeph_count();q.usage=tl::fea::qeph::BatchUsage::CoupledForces;q.startup=f.startup;
    moving_cache_test::Check(r.quad.InitializeMapped(q,f.physical,r.owner,f.Witnesses()));
    tl::fea::t3::T3BatchConfig t;t.owner=r.owner.accepted();t.configuration_id=901;t.qualification_id=f.Qualification;
    t.element_count=f.shells.t3_count();t.usage=tl::fea::t3::BatchUsage::CoupledForces;t.startup=f.startup;
    moving_cache_test::Check(r.triangle.InitializeMapped(t,f.physical,r.owner,f.Witnesses()));
    moving_cache_test::Attempt proof;r.Begin(proof);r.owner.Discard();r.quad.DiscardTrial();r.triangle.DiscardTrial();
    moving_cache_test::Check(r.publication.InitializePhysical(r.owner,f.physical,f.rigid,f.Witnesses(),r.Participants(),r.Identity()));
  }
  n::TransactionReport InitializeContact(n::TransactionLimits limits=moving_cache_test::Rig::Limits()) {
    auto& r=rig;return r.contact.GeneralInitialize(r.config,runtime,prepared,r.owner,r.publication,r.source.physical.physical,r.Participants(),r.Identity(),limits);
  }
  void BindRoster() {
    auto& r=rig;moving_cache_test::Check(r.publication.ConfigurePhysicalScratchParticipation(r.owner,r.source.physical.physical,r.Participants(),r.Identity(),{{},r.contact.roster_entry()}));
  }
};
struct FixedSource {
  Source base;
  st::Input mesh;st::Snapshot starter;st::FixedMainView ready;
  tl::util::HostArena output,scratch,ready_output,ready_scratch;
  std::vector<n::lifecycle::Main> initial_mains,runtime_mains;
  std::vector<double> coefficients,main_gap;
  std::vector<std::uint32_t> main_nodes;
  is::PreparedSource prepared;n::FixedMainSource runtime;
  FixedSource() {
    mesh=base.mesh;mesh.profile=st::Profile::OrdinaryExteriorFixedMain;mesh.topology=st::TopologyPolicy::ManifoldTwoSided;mesh.primary_count=1;
    const auto forecast=st::Preflight(mesh);
    if(forecast.status!=st::Status::Ok||!output.Initialize(forecast.output_bytes)||!scratch.Initialize(forecast.scratch_bytes)||
        !ready_output.Initialize(forecast.ready_output_bytes)||!ready_scratch.Initialize(forecast.ready_scratch_bytes))
      throw std::runtime_error("Fixed general source arenas rejected");
    if(st::BuildStarter(mesh,{},output,scratch,&starter).status!=st::Status::Ok)throw std::runtime_error("Fixed general Starter rejected");
    coefficients.assign(2,1e6);main_gap.assign(2,.0012);
    if(st::BuildFixedMain(mesh,starter,{coefficients.data(),coefficients.size()},{},ready_output,ready_scratch,&ready).status!=st::Status::Ok)
      throw std::runtime_error("Fixed ready source rejected");
    initial_mains.resize(2);
    for(std::size_t i=0;i<2;++i) {
      const auto& original=starter.mains[i];auto& main=initial_mains[i];main.global_id=original.global_id;main.segment_type=original.segment_type;
      main.coefficient=1e6;main.maximum_gap=.0012;
      for(unsigned k=0;k<4;++k){main.nodes[k]=original.nodes[k];main.neighbors[k]=original.neighbors[k];main.normal_reference[k]=original.normal_reference[k];
        main.normal_slot[k]=starter.starter.face_normals[4*i+k];main.gap[k]=.0012;}
    }
    for(auto node:starter.mains[0].nodes)main_nodes.push_back(node);
  }
  is::Input Input() const {
    auto in=base.Input();in.mesh=mesh;in.starter=starter;in.contact.mains=initial_mains.data();in.contact.main_count=2;
    in.contact.normals=starter.starter.references;in.contact.normal_count=starter.starter.reference_count;
    in.contact.normal_to_main={starter.normal_offsets,in.contact.normal_count+1,starter.normal_mains,starter.normal_incidence_count};
    in.main_nodes=main_nodes.data();in.main_node_count=main_nodes.size();in.main_search_gap=main_gap.data();in.main_search_gap_count=2;
    return in;
  }
  void Prepare() {
    const auto in=Input();const auto report=is::PrepareSource(in,Source::InitialLimits(),prepared);
    if(report.status!=is::Status::Ok)throw std::runtime_error("Fixed initial source preparation rejected");
    static_cast<n::ContactSourceInput&>(runtime)=base.runtime;
    runtime.selection=in.contact;runtime_mains=initial_mains;
    for(std::size_t i=0;i<2;++i)for(unsigned k=0;k<4;++k)runtime_mains[i].normal_slot[k]=ready.normals.face_normals[4*i+k];
    runtime.selection.mains=runtime_mains.data();runtime.selection.normals=ready.normals.references;
    runtime.primary_main_count=1;runtime.primary_parent_ids=base.rig.source.parents.data();
    runtime.margin=prepared.removals().engine_margin;runtime.primary_curvature=prepared.removals().primary_extent;
  }
};
}
