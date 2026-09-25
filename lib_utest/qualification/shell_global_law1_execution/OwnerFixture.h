// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../nodal_empty_cin/Fixture.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/ShellBatchFailure.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
namespace global_law1_execution_test {
namespace fe=tl::fea;namespace q=fe::qeph;namespace t=fe::t3;
using Profile=fe::ShellGlobalLaw1Profile;
using Thickness=fe::ShellLaw1Thickness;
inline fe::ShellParentExecution Global(Profile profile) {return {fe::ShellParentExecutionPolicy::GlobalLaw1Npt0,profile};}
template<class R> void Check(const R& report) {
  if(report.status!=decltype(report.status)::Success)throw std::runtime_error(report.message);
}
inline void Check(const fe::NodalReport& report) {
  if(report.status!=fe::NodalStatus::Ok)throw std::runtime_error(report.message);
}
inline nodal_empty_test::Source MixedSource(Profile profile) {
  const auto original=nodal_empty_test::SmallSource();auto source=original;
  source.nodes=3*original.nodes;source.quads.clear();source.triangles.clear();source.parents.clear();
  auto elastic=original.materials[0];elastic.material_id=1;elastic.curve_id=0;
  elastic.hardening=tl::material::ShellPlasticityHardeningKind::Tabulated;
  elastic.linear={};elastic.rate={};elastic.law=fe::ShellSectionLaw::LayeredLaw1Nip3;
  auto plastic=original.materials[0];plastic.material_id=2;source.materials={elastic,plastic};
  for(unsigned block=0;block<3;++block) {
    auto quad=original.quads[0];auto triangle=original.triangles[0];
    quad.source_parent_id+=1000*block;triangle.source_parent_id+=1000*block;
    for(unsigned j=0;j<4;++j) {
      quad.nodes[j]+=7*block;quad.reference.node_ids[j]+=100*block;
      quad.reference.position[j].x+=.12*block;
    }
    for(unsigned j=0;j<3;++j) {
      triangle.nodes[j]+=7*block;triangle.reference.node_ids[j]+=100*block;
      triangle.reference.position[j].x+=.12*block;
    }
    source.quads.push_back(quad);source.triangles.push_back(triangle);
    fe::ShellPlasticityParentInput qp{fe::ShellBindingFamily::Qeph,block,quad.source_parent_id,100+2*block,block==2?2u:1u,1};
    fe::ShellPlasticityParentInput tp{fe::ShellBindingFamily::T3,block,triangle.source_parent_id,101+2*block,block==2?2u:1u,1};
    if(block==0){qp.execution=Global(profile);tp.execution=Global(profile);}
    source.parents.push_back(qp);source.parents.push_back(tp);
  }
  return source;
}
struct Snapshot {
  fe::NodalStamp stamp;fe::ShellPhysicalDiagnostics common;
  std::vector<double> x,v,w,orientation,reaction,couple,m,j;
  std::vector<q::ForceTrial> quads;std::vector<t::ForceTrial> triangles;
  std::vector<fe::ShellBatchLayeredSection> qsections,tsections;
  std::vector<fe::ShellBatchFailureState> qfailure,tfailure;
  explicit Snapshot(std::size_t nodes,std::size_t nq=3,std::size_t nt=3):
      x(3*nodes),v(3*nodes),w(3*nodes),orientation(4*nodes),reaction(3*nodes),couple(3*nodes),m(nodes),j(nodes),
      quads(nq),triangles(nt),qsections(nq),tsections(nt),qfailure(nq),tfailure(nt){}
};
struct Attempt {
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics candidate,common;
};
struct OwnerFixture {
  nodal_empty_test::Source source;
  nodal_empty_test::Fixture physical;
  fe::FENodalState owner;q::QephBatch quad;t::T3Batch triangle;
  fe::ShellBatchPublication publication; // Destroy before borrowed participants.
  static constexpr std::uint64_t ConfigId=1307;
  explicit OwnerFixture(Profile profile):source(MixedSource(profile)){}
  ~OwnerFixture(){Discard();}
  fe::ShellPhysicalParticipants Participants(){return {&quad,&triangle};}
  fe::ShellPhysicalPublicationIdentity Identity() const{return {ConfigId,nodal_empty_test::Fixture::Qualification,physical.startup};}
  void Initialize() {
    std::vector<std::uint8_t> fixed(source.nodes),rotations(source.nodes);
    for(unsigned b=0;b<3;++b){fixed[7*b]=1;fixed[7*b+4]=2;}
    physical.Prepare(source,std::move(fixed),std::move(rotations),{.3,-.4,.2});physical.fixed_dt=1e-8;
    Check(physical.Initialize(owner));
    q::QephBatchConfig qc;qc.owner=owner.accepted();qc.configuration_id=ConfigId;
    qc.qualification_id=nodal_empty_test::Fixture::Qualification;qc.element_count=3;
    qc.usage=q::BatchUsage::CoupledForces;qc.startup=physical.startup;
    t::T3BatchConfig tc;tc.owner=owner.accepted();tc.configuration_id=ConfigId;
    tc.qualification_id=nodal_empty_test::Fixture::Qualification;tc.element_count=3;
    tc.usage=t::BatchUsage::CoupledForces;tc.startup=physical.startup;
    Check(quad.InitializeMapped(qc,physical.physical,owner,physical.Witnesses()));
    Check(triangle.InitializeMapped(tc,physical.physical,owner,physical.Witnesses()));
    Attempt proof;Begin(proof);Discard();
    Check(publication.InitializePhysical(owner,physical.physical,physical.rigid,physical.Witnesses(),Participants(),Identity()));
  }
  void Begin(Attempt& a) {
    Check(owner.BeginTrial(&a.token,&a.assembly));
    Check(quad.AssembleMappedAccepted(owner,a.token,a.assembly));
    Check(triangle.AssembleMappedAccepted(owner,a.token,a.assembly));
  }
  void Prepare(Attempt& a) {
    Check(owner.SealAssembly(a.token));
    Check(fe::AdvanceStaggeredCin(owner,a.token,{a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
        nodal_empty_test::Fixture::Qualification,physical.fixed_dt,.2,true,
        {fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}}));
    Check(owner.BorrowPrepared(a.token,&a.prepared));
    Check(quad.EvaluateCandidate(owner,a.token,a.prepared,&a.candidate.qeph));
    Check(triangle.EvaluateCandidate(owner,a.token,a.prepared,&a.candidate.t3));
    Check(publication.PreparePhysical(owner,a.token,{&a.candidate.qeph,&a.candidate.t3},&a.common));
  }
  fe::ShellPublicationReport Commit(const Attempt& a,bool valid=true) {
    return publication.CommitPhysical(owner,a.token,a.common,{a.prepared.owner_id,a.prepared.kinematics.base_epoch,
      a.prepared.attempt,nodal_empty_test::Fixture::Qualification,valid});
  }
  void Discard(){publication.DiscardTrial();owner.Discard();quad.DiscardTrial();triangle.DiscardTrial();}
  Snapshot Read() {
    Snapshot result(source.nodes);Check(owner.CopyAccepted({result.x.data(),result.v.data(),source.nodes,
      result.orientation.data(),result.w.data(),result.reaction.data(),result.couple.data()},&result.stamp));
    double numerical=0;fe::NodalStamp raw;
    Check(owner.CopyAcceptedCin({result.m.data(),result.j.data(),nullptr,nullptr,&numerical,source.nodes,0},&raw));
    if(numerical!=0||raw.epoch!=result.stamp.epoch)throw std::runtime_error("Raw physical coefficients changed");
    q::BatchDiagnostics qd;t::BatchDiagnostics td;
    Check(quad.CopyAcceptedResults(result.stamp,result.quads.data(),3,&qd));
    Check(triangle.CopyAcceptedResults(result.stamp,result.triangles.data(),3,&td));
    Check(quad.CopyAcceptedLayeredSectionHistory(result.stamp,result.qsections.data(),3,&qd));
    Check(triangle.CopyAcceptedLayeredSectionHistory(result.stamp,result.tsections.data(),3,&td));
    Check(quad.CopyAcceptedFailureHistory(result.stamp,result.qfailure.data(),3,&qd));
    Check(triangle.CopyAcceptedFailureHistory(result.stamp,result.tfailure.data(),3,&td));
    Check(publication.CopyAcceptedPhysicalDiagnostics(result.stamp,&result.common));return result;
  }
};
} // namespace global_law1_execution_test
