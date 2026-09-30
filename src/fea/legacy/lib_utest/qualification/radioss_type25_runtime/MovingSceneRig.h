// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ObservedFixture.h"
#include "RuntimeObservation.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include <stdexcept>
#include <string>
namespace native_runtime_test {
inline void Check(const fe::NodalReport& r){if(r.status!=fe::NodalStatus::Ok)throw std::runtime_error(r.message);}
inline void Check(const fe::ShellPublicationReport& r){if(r.status!=fe::ShellPublicationStatus::Success)throw std::runtime_error(r.message);}
inline void Check(const n::TransactionReport& r){if(r.status!=n::TransactionStatus::Ok)throw std::runtime_error(std::string(r.message)+" row="+std::to_string(r.row)+" status="+std::to_string(int(r.status))+" selection="+std::to_string(int(r.selection_status)));}
inline void Check(const fe::qeph::BatchReport& r){if(r.status!=fe::qeph::BatchStatus::Success)throw std::runtime_error(r.message);}
inline void Check(const fe::t3::BatchReport& r){if(r.status!=fe::t3::BatchStatus::Success)throw std::runtime_error(r.message);}
inline void Check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
struct Attempt {
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalCinAssemblyView cin;
  fe::NodalPreparedView prepared;fe::ShellPhysicalDiagnostics material,common;
  fe::ShellPhysicalScratchParticipationReceipt contact;
};
struct State {
  std::array<double,54> x{},v{},omega{},reaction{},couple{};
  std::array<double,72> q{};std::array<double,18> mass{},inertia{};
  std::array<n::NativeGeometryHistory,18> history;std::array<int,18> initial_contact{};
  fe::NodalStamp stamp;fe::NativeContactPublicationSnapshot contact;
};
struct Rig {
  nodal_empty_test::Fixture fixture;ObservedSource source;
  fe::FENodalState owner;fe::qeph::QephBatch quad;fe::t3::T3Batch triangle;
  std::unique_ptr<fe::ShellBatchPublication> publication;
  n::Transaction contact; // Destroy before publisher, batches, stream and owner.
  fe::ShellPhysicalParticipants Participants(){return {&quad,&triangle};}
  fe::ShellPhysicalPublicationIdentity Identity() const{return {901,nodal_empty_test::Fixture::Qualification,fixture.startup};}
  void Initialize(n::TransactionLimits limits=ObservedSource::Limits(),bool attach_contact=true) {
    auto physical=ModelSource();std::vector<std::uint8_t> fixed(18),rotation(18);
    for(unsigned i=0;i<18;++i){const auto code=observed::ConstraintCodes[i];fixed[i]=((code&1)<<2)|(code&2)|((code&4)>>2);rotation[i]=fixed[i]==7;}
    fixture.Prepare(physical,std::move(fixed),std::move(rotation),{observed::VelocityMmS[0]*.001,observed::VelocityMmS[1]*.001,observed::VelocityMmS[2]*.001});
    fixture.fixed_dt=observed::TimeAndThickness[0];Check(fixture.Initialize(owner));
    fe::qeph::QephBatchConfig q;q.owner=owner.accepted();q.configuration_id=901;q.qualification_id=nodal_empty_test::Fixture::Qualification;
    q.element_count=4;q.usage=fe::qeph::BatchUsage::CoupledForces;q.startup=fixture.startup;
    Check(quad.InitializeMapped(q,fixture.physical,owner,fixture.Witnesses()));
    fe::t3::T3BatchConfig t;t.owner=owner.accepted();t.configuration_id=901;t.qualification_id=nodal_empty_test::Fixture::Qualification;
    t.element_count=8;t.usage=fe::t3::BatchUsage::CoupledForces;t.startup=fixture.startup;
    Check(triangle.InitializeMapped(t,fixture.physical,owner,fixture.Witnesses()));
    // Authenticate initial zero-stress cache from real owner fields. Discard the
    // startup proof without advancing time or material/contact history.
    Attempt proof;BeginMaterials(proof);owner.Discard();quad.DiscardTrial();triangle.DiscardTrial();
    publication=std::make_unique<fe::ShellBatchPublication>();
    Check(publication->InitializePhysical(owner,fixture.physical,fixture.rigid,fixture.Witnesses(),Participants(),Identity()));
    if(attach_contact) {
      Check(contact.Initialize(ObservedSource::Config(),source.View(),owner,*publication,fixture.physical,Participants(),Identity(),limits));
      Check(publication->ConfigurePhysicalScratchParticipation(owner,fixture.physical,Participants(),Identity(),{{},contact.roster_entry()}));
    }
  }
  void BeginMaterials(Attempt& a){Check(owner.BeginTrial(&a.token,&a.assembly));
    Check(quad.AssembleMappedAccepted(owner,a.token,a.assembly));Check(triangle.AssembleMappedAccepted(owner,a.token,a.assembly));
    Check(owner.BorrowCinAssembly(a.token,&a.cin));}
  void Prepare(Attempt& a) {
    Check(owner.SealAssembly(a.token));
    Check(fe::AdvanceStaggeredCin(owner,a.token,{a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
      nodal_empty_test::Fixture::Qualification,fixture.fixed_dt,.2,true,
      {fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}}));
    Check(owner.BorrowPrepared(a.token,&a.prepared));
    Check(quad.EvaluateCandidate(owner,a.token,a.prepared,&a.material.qeph));
    Check(triangle.EvaluateCandidate(owner,a.token,a.prepared,&a.material.t3));
    Check(publication->PreparePhysical(owner,a.token,{&a.material.qeph,&a.material.t3},&a.common));
    Check(contact.SealCandidate(owner,a.token,a.prepared,a.common,&a.contact));
    Check(publication->SealPhysicalScratchParticipation(owner,a.token,{nullptr,&a.contact}));
  }
  fe::ShellPublicationReport Commit(Attempt& a,bool accepted=true) {
    return publication->CommitPhysical(owner,a.token,a.common,{a.prepared.owner_id,a.prepared.kinematics.base_epoch,
      a.prepared.attempt,nodal_empty_test::Fixture::Qualification,accepted});
  }
  void Step(){Attempt a;BeginMaterials(a);Check(contact.AssembleAccepted(owner,a.token,a.assembly));Prepare(a);Check(Commit(a));}
  State Read() {
    State s;Check(owner.CopyAccepted({s.x.data(),s.v.data(),18,s.q.data(),s.omega.data(),s.reaction.data(),s.couple.data()},&s.stamp));
    double numerical=0;fe::NodalStamp coefficient;
    Check(owner.CopyAcceptedCin({s.mass.data(),s.inertia.data(),nullptr,nullptr,&numerical,18,0},&coefficient));
    if(numerical!=0)throw std::runtime_error("Unexpected numerical mass in fixed unscaled coupon");
    Check(contact.CopyAccepted({s.history.data(),s.initial_contact.data(),18},&s.contact));return s;
  }
  void Force(const Attempt& a,std::array<double,54>& values,std::array<double,18>& stiffness) {
    std::array<double,54> component{};
    struct Drain{cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{a.assembly.stream};
    Check(cudaMemcpyAsync(component.data(),a.assembly.forces.force_x,18*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
    Check(cudaMemcpyAsync(component.data()+18,a.assembly.forces.force_y,18*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
    Check(cudaMemcpyAsync(component.data()+36,a.assembly.forces.force_z,18*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
    Check(cudaMemcpyAsync(stiffness.data(),a.cin.translational_stiffness,18*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
    Check(cudaStreamSynchronize(a.assembly.stream));
    for(unsigned i=0;i<18;++i)for(unsigned j=0;j<3;++j)values[3*i+j]=component[18*j+i];
  }
};
} // namespace native_runtime_test
