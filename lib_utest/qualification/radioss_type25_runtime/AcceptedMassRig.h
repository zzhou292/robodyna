// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "MovingCacheRig.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <array>
namespace accepted_mass_runtime_test {
namespace n=tlfea::contact::radioss_type25;namespace fe=tl::fea;
namespace tied=tl::constraints::tied_shell;namespace cin=tied::cin;
using moving_cache_test::Check;
using moving_cache_test::Attempt;
// Synthetic source/publication coupon, not a V5 parser or matched Engine scene.
// The declared TYPE25 secondary roster excludes the CIN secondary. No positive
// zero-MSI contact is forced past an unresolved native tied-removal policy.
struct CinRig: moving_cache_test::Rig {
  cin::WitnessRange range{0,1};cin::ActiveWitness witness;
  CinRig() {
    config.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
    auto& f=source.physical;const auto nodes=f.domain.nodes();
    for(unsigned node=0;node<5;++node) {
      f.fixed[node]=0;f.rotation_fixed[node]=0;
      f.inverse_mass[node]=1/f.mass[node];f.inverse_inertia[node]=1/f.inertia[node];
      const auto v=f.startup.uniform_velocity;
      f.velocities[3*node]=v.x;f.velocities[3*node+1]=v.y;f.velocities[3*node+2]=v.z;
      source.nodes[node].constraint=0;
    }
    f.inverse_mass[4]=f.inverse_inertia[4]=0;
    tied::CinAttachmentDeclaration declaration;declaration.original_nsv_row=1;
    declaration.ordered_master_rank=1;declaration.master_source={tied::CinMasterSourceKind::DeclaredShellElement,100,1};
    declaration.secondary_source_id=nodes[4].source_id;declaration.reference_positions[0]=nodes[4].position;
    for(unsigned slot=0;slot<4;++slot) {
      declaration.master_source_ids[slot]=nodes[slot].source_id;
      declaration.reference_positions[slot+1]=nodes[slot].position;witness.nodes[slot]=slot;
    }
    witness.source_element_id=100;witness.native_parent_index=0;witness.family=cin::WitnessFamily::ShellQuad;
    tied::KinChkSlave slave{std::uint32_t(nodes[4].source_id),0,{2,7,7,0,0}};
    std::array<std::int32_t,8192> decode{};
    for(std::size_t i=0;i<decode.size();++i)decode[i]=(i&2)!=0;
    tied::PostKinChkResult post;
    f.Require(bool(tied::PostKinChk({tied::KinChkProfile::NoWallRbeOrCyclic,
        tied::ClassificationPhase::InterfaceTaggedBeforeKinChk,f.domain.source_instance_id(),881,
        {&slave,1},{decode.data(),decode.size()}},&post)),"Synthetic CIN source classification");
    tied::TiedCinAttachmentModel model;
    f.Require(bool(tied::PrepareCinAttachments(post,f.domain,{&declaration,1},&model)),"Synthetic CIN attachment");
    f.cin=model;
    source.secondary.erase(source.secondary.begin()+4);
    source.removal_offsets.assign(source.secondary.size()+1,0);
  }
  fe::NodalCinWitnessSource Witnesses() const{return {&source.physical.cin,&range,&witness,1,1};}
  void Initialize() {
    auto& f=source.physical;const fe::NodalCinStartup cin_source{&f.cin,f.mass.data(),f.inertia.data(),&range,&witness,1,f.Qualification};
    Check(owner.Initialize(f.Config(),f.Kinematics(),f.inverse_mass.data(),f.Dofs(),f.rigid,&cin_source));
    fe::qeph::QephBatchConfig q;q.owner=owner.accepted();q.configuration_id=901;q.qualification_id=f.Qualification;
    q.element_count=f.shells.qeph_count();q.usage=fe::qeph::BatchUsage::CoupledForces;q.startup=f.startup;
    Check(quad.InitializeMapped(q,f.physical,owner,Witnesses()));
    fe::t3::T3BatchConfig t;t.owner=owner.accepted();t.configuration_id=901;t.qualification_id=f.Qualification;
    t.element_count=f.shells.t3_count();t.usage=fe::t3::BatchUsage::CoupledForces;t.startup=f.startup;
    Check(triangle.InitializeMapped(t,f.physical,owner,Witnesses()));
    Attempt proof;Begin(proof);owner.Discard();quad.DiscardTrial();triangle.DiscardTrial();
    Check(publication.InitializePhysical(owner,f.physical,f.rigid,Witnesses(),Participants(),Identity()));
    Check(contact.Initialize(config,source.Source(),owner,publication,f.physical,Participants(),Identity(),Limits()));
    Check(publication.ConfigurePhysicalScratchParticipation(owner,f.physical,Participants(),Identity(),{{},contact.roster_entry()}));
  }
  void Prepare(Attempt& a) {
    const auto require=[](fe::NodalReport r,const char* stage) {
      if(r.status!=fe::NodalStatus::Ok)throw std::runtime_error(std::string(stage)+": "+r.message+
          " status="+std::to_string(unsigned(r.status))+" node="+std::to_string(r.node)+" dt="+std::to_string(r.stable_dt));
    };
    require(owner.SealAssembly(a.token),"CIN seal");
    require(fe::AdvanceStaggeredCin(owner,a.token,{a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
      nodal_empty_test::Fixture::Qualification,source.physical.fixed_dt,.2,true,
      {fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}}),"CIN advance");
    require(owner.BorrowPrepared(a.token,&a.prepared),"CIN candidate borrow");
    Check(quad.EvaluateCandidate(owner,a.token,a.prepared,&a.material.qeph));
    Check(triangle.EvaluateCandidate(owner,a.token,a.prepared,&a.material.t3));
    Check(publication.PreparePhysical(owner,a.token,{&a.material.qeph,&a.material.t3},&a.common));
    Check(contact.SealCandidate(owner,a.token,a.prepared,a.common,&a.contact));
    Check(publication.SealPhysicalScratchParticipation(owner,a.token,{nullptr,&a.contact}));
  }
  std::vector<double> Raw() {
    const auto count=source.nodes.size();std::vector<double> out(2*count+3);fe::NodalStamp stamp;
    Check(owner.CopyAcceptedCin({out.data(),out.data()+count,out.data()+2*count,out.data()+2*count+1,
      out.data()+2*count+2,count,1},&stamp));return out;
  }
};
// A genuine PART aggregate with three physical T3 members, not a fabricated
// point mass or primary in the contact domain. Separate from the CIN fixture.
struct RigidRig: moving_cache_test::Rig {
  fe::rigid::NodalRigidPartTopology topology;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidAssemblyBinding rigid;
  fe::ShellBatchPlasticityBinding catalog;fe::ShellBatchFailureBinding failure;
  fe::ShellExecutionBinding execution;fe::ShellPhysicalBinding physical;
  RigidRig():moving_cache_test::Rig(true) {
    config.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
    auto& f=source.physical;std::array<std::uint64_t,3> ids{};
    for(unsigned node=4;node<7;++node) {
      ids[node-4]=f.domain.nodes()[node].source_id;
      f.fixed[node]=0;f.rotation_fixed[node]=0;source.nodes[node].constraint=0;
      f.inverse_mass[node]=1/f.mass[node];f.inverse_inertia[node]=1/f.inertia[node];
      const auto v=f.startup.uniform_velocity;
      f.velocities[3*node]=v.x;f.velocities[3*node+1]=v.y;f.velocities[3*node+2]=v.z;
    }
    const fe::rigid::PartTopologyPartInput part{2,ids.data(),ids.size()};
    fe::rigid::PartTopologyInput input;input.source_instance_id=f.domain.source_instance_id();
    input.parts=&part;input.part_count=1;input.expected_members=ids.data();input.expected_member_count=ids.size();
    f.Require(bool(topology.Initialize(input)),"Synthetic rigid topology");
    f.Require(bool(parts.Initialize(topology,f.ledger,{1,1})),"Physical rigid aggregate");
    f.Require(bool(rigid.Initialize(parts)),"Physical rigid binding");
    auto declared=nodal_empty_test::SmallSource();
    auto& law1=declared.materials[0];law1.law=fe::ShellSectionLaw::LayeredLaw1Nip3;
    law1.hardening=tl::material::ShellPlasticityHardeningKind::Tabulated;law1.curve_id=0;law1.linear={};law1.rate={};
    declared.parents[0].execution={fe::ShellParentExecutionPolicy::GlobalLaw1Npt0,{fe::ShellLaw1Thickness::Accepted,1.}};
    auto skin=law1;skin.material_id=2;skin.law=fe::ShellSectionLaw::RigidSkin;declared.materials.push_back(skin);
    declared.sections.push_back({2,.002,0,fe::ShellSectionFormulation::Nonconstitutive});
    declared.parents[1].material_id=2;declared.parents[1].section_id=2;
    f.Require(catalog.InitializeExecutionCatalog(f.shells,declared.Catalog()).status==fe::ShellPlasticityBindingStatus::Success,"Declared rigid-skin catalog");
    std::array<fe::ShellFailureParentInput,2> no_failures{};
    for(unsigned i=0;i<2;++i){no_failures[i].source=declared.parents[i];no_failures[i].policy=fe::ShellFailurePolicy::None;}
    f.Require(failure.InitializeExecution(catalog,no_failures.data(),no_failures.size()).status==fe::ShellPlasticityBindingStatus::Success,"Rigid-skin failure declarations");
    f.Require(execution.Initialize(catalog,f.ledger,rigid).status==fe::ShellPlasticityBindingStatus::Success,"Rigid execution roles");
    f.Require(bool(physical.InitializeExecution({&f.shells,&catalog,&failure,nullptr},f.ledger,execution)),"Rigid physical binding");
  }
  n::TransactionReport Initialize() {
    auto& f=source.physical;const auto cin_source=f.Cin();
    Check(owner.Initialize(f.Config(),f.Kinematics(),f.inverse_mass.data(),f.Dofs(),rigid,&cin_source));
    fe::qeph::QephBatchConfig q;q.owner=owner.accepted();q.configuration_id=901;q.qualification_id=f.Qualification;
    q.element_count=f.shells.qeph_count();q.usage=fe::qeph::BatchUsage::CoupledForces;q.startup=f.startup;
    Check(quad.InitializeMapped(q,physical,owner,f.Witnesses()));
    fe::t3::T3BatchConfig t;t.owner=owner.accepted();t.configuration_id=901;t.qualification_id=f.Qualification;
    t.element_count=f.shells.t3_count();t.usage=fe::t3::BatchUsage::CoupledForces;t.startup=f.startup;
    Check(triangle.InitializeMapped(t,physical,owner,f.Witnesses()));
    Attempt proof;Begin(proof);owner.Discard();quad.DiscardTrial();triangle.DiscardTrial();
    Check(publication.InitializePhysical(owner,physical,rigid,f.Witnesses(),Participants(),Identity()));
    const auto result=contact.Initialize(config,source.Source(),owner,publication,physical,Participants(),Identity(),Limits());
    if(result.status==n::TransactionStatus::Ok)
      Check(publication.ConfigurePhysicalScratchParticipation(owner,physical,Participants(),Identity(),{{},contact.roster_entry()}));
    return result;
  }
};
} // namespace accepted_mass_runtime_test
