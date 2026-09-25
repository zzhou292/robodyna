// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SourceAdmissionFixture.h"
#include "RuntimeObservation.h"
#include "lib_src/elements/qeph/QephBatch.h"
#include "lib_src/elements/t3/T3Batch.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
namespace moving_cache_test {
namespace n=tlfea::contact::radioss_type25;namespace fe=tl::fea;
inline void Check(fe::NodalReport r){if(r.status!=fe::NodalStatus::Ok)throw std::runtime_error(r.message);}
inline void Check(fe::ShellPublicationReport r){if(r.status!=fe::ShellPublicationStatus::Success)throw std::runtime_error(r.message);}
inline void Check(fe::qeph::BatchReport r){if(r.status!=fe::qeph::BatchStatus::Success)throw std::runtime_error(r.message);}
inline void Check(fe::t3::BatchReport r){if(r.status!=fe::t3::BatchStatus::Success)throw std::runtime_error(r.message);}
inline void Check(n::TransactionReport r){if(r.status!=n::TransactionStatus::Ok)throw std::runtime_error(std::string(r.message)+" row="+std::to_string(r.row));}
struct Attempt {
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics material,common;fe::ShellPhysicalScratchParticipationReceipt contact;
};
// Reuses the complete physical source/startup fixture and actual mapped batches.
// Only this qualification helper chooses the tiny synthetic contact coefficients.
struct Rig {
  type25_source_test::Fixture source;
  fe::FENodalState owner;fe::qeph::QephBatch quad;fe::t3::T3Batch triangle;
  fe::ShellBatchPublication publication;n::Transaction contact;
  n::TransactionConfig config=type25_source_test::Fixture::Config();
  Rig() {
    const auto& bits=source.physical.fixed;
    for(std::size_t i=0;i<bits.size();++i)source.nodes[i].constraint=((bits[i]&1)<<2)|(bits[i]&2)|((bits[i]&4)>>2);
    config.lifecycle.maximum_coefficient=1e30;
  }
  fe::ShellPhysicalParticipants Participants(){return {&quad,&triangle};}
  fe::ShellPhysicalPublicationIdentity Identity() const{return {901,nodal_empty_test::Fixture::Qualification,source.physical.startup};}
  static n::TransactionLimits Limits() {
    n::TransactionLimits l;l.inventory.max_nodes=7;l.inventory.max_mains=2;l.inventory.max_secondaries=7;
    l.inventory.max_pairs=128;l.inventory.max_tasks=128;l.optimized_candidates=128;l.sliding_entries=256;
    l.max_device_bytes=16u<<20;l.max_host_bytes=16u<<20;return l;
  }
  void Initialize(n::TransactionLimits limits=Limits()) {
    auto& f=source.physical;Check(f.Initialize(owner));
    fe::qeph::QephBatchConfig q;q.owner=owner.accepted();q.configuration_id=901;q.qualification_id=f.Qualification;
    q.element_count=f.shells.qeph_count();q.usage=fe::qeph::BatchUsage::CoupledForces;q.startup=f.startup;
    Check(quad.InitializeMapped(q,f.physical,owner,f.Witnesses()));
    fe::t3::T3BatchConfig t;t.owner=owner.accepted();t.configuration_id=901;t.qualification_id=f.Qualification;
    t.element_count=f.shells.t3_count();t.usage=fe::t3::BatchUsage::CoupledForces;t.startup=f.startup;
    Check(triangle.InitializeMapped(t,f.physical,owner,f.Witnesses()));
    Attempt proof;Begin(proof);owner.Discard();quad.DiscardTrial();triangle.DiscardTrial();
    Check(publication.InitializePhysical(owner,f.physical,f.rigid,f.Witnesses(),Participants(),Identity()));
    Check(contact.Initialize(config,source.Source(),owner,publication,f.physical,Participants(),Identity(),limits));
    Check(publication.ConfigurePhysicalScratchParticipation(owner,f.physical,Participants(),Identity(),{{},contact.roster_entry()}));
  }
  void Begin(Attempt& a) {
    Check(owner.BeginTrial(&a.token,&a.assembly));Check(quad.AssembleMappedAccepted(owner,a.token,a.assembly));
    Check(triangle.AssembleMappedAccepted(owner,a.token,a.assembly));
  }
  void Prepare(Attempt& a) {
    Check(owner.SealAssembly(a.token));
    Check(fe::AdvanceStaggeredCin(owner,a.token,{a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
      nodal_empty_test::Fixture::Qualification,source.physical.fixed_dt,.2,true,
      {fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}}));
    Check(owner.BorrowPrepared(a.token,&a.prepared));
    Check(quad.EvaluateCandidate(owner,a.token,a.prepared,&a.material.qeph));
    Check(triangle.EvaluateCandidate(owner,a.token,a.prepared,&a.material.t3));
    Check(publication.PreparePhysical(owner,a.token,{&a.material.qeph,&a.material.t3},&a.common));
    Check(contact.SealCandidate(owner,a.token,a.prepared,a.common,&a.contact));
    Check(publication.SealPhysicalScratchParticipation(owner,a.token,{nullptr,&a.contact}));
  }
  fe::ShellPublicationReport Commit(Attempt& a,bool admitted=true) {
    return publication.CommitPhysical(owner,a.token,a.common,{a.prepared.owner_id,a.prepared.kinematics.base_epoch,
      a.prepared.attempt,nodal_empty_test::Fixture::Qualification,admitted});
  }
  void Discard(){contact.DiscardTrial();quad.DiscardTrial();triangle.DiscardTrial();}
  std::vector<double> Positions() {
    const auto n=source.nodes.size();std::vector<double> x(3*n),v(3*n);fe::NodalStamp stamp;
    Check(owner.CopyAccepted({x.data(),v.data(),n},&stamp));return x;
  }
};
} // namespace moving_cache_test
