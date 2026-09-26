// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FullLedgerFixture.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/collision/RadiossType25Transaction.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_src/elements/solids/Model.h"
#include "lib_src/materials/law36/Prepare.h"
#include "lib_src/materials/law42/Prepare.h"

namespace type25_source_test {
namespace fe=tl::fea;
template<class Report> void Check(const Report& report) {
  using Status=decltype(report.status);
  if(report.status!=Status::Success)throw std::runtime_error(report.message);
}
inline void Check(const fe::NodalReport& report) {
  if(report.status!=fe::NodalStatus::Ok)throw std::runtime_error(report.message);
}
inline void Check(const n::TransactionReport& report) {
  if(report.status!=n::TransactionStatus::Ok)
    throw std::runtime_error(std::string(report.message)+" row="+std::to_string(report.row));
}
inline void Check(cudaError_t error) {
  if(error!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(error));
}
struct FullLedgerAttempt {
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics candidates,common;
  fe::ShellPhysicalScratchParticipationReceipt contact;
};
// Real heterogeneous participants; the source fixture is synthetic, not a
// vehicle parser or a CPU/GPU crash comparison.
struct FullLedgerRig {
  static constexpr std::uint64_t Configuration=7191,Qualification=7192;
  static constexpr double Dt=1e-9;
  explicit FullLedgerRig(bool with_qbat=false,double failure_strain=2.5):fixture(with_qbat,true,failure_strain){}
  FullLedgerFixture fixture;
  fe::solids::Model solid_model;
  tl::constraints::tied_shell::PostKinChkResult classified;
  tl::constraints::tied_shell::TiedCinAttachmentModel cin;
  std::array<tl::constraints::tied_shell::cin::WitnessRange,1> ranges{{{0,2}}};
  std::array<tl::constraints::tied_shell::cin::ActiveWitness,3> witnesses;
  std::vector<double> x,v,w,q,m,j,im,ij;
  std::vector<std::uint8_t> fixed,rotation_fixed,present;
  fe::ShellBatchStartup startup{fe::ShellBatchStartupKind::ReferenceUniformTranslation,{0,0,0}};
  fe::FENodalState owner;
  fe::qeph::QephBatch qeph;
  fe::t3::T3Batch t3;
  fe::qbat::Batch qbat;
  fe::type25::Batch welds;
  fe::type13::Batch beams;
  fe::solids::Batch solids;
  fe::ShellBatchPublication publication;
  n::Transaction contact; // Retires before its borrowed publisher/owner.

  fe::ShellPhysicalParticipants Participants() {return {&qeph,&t3,fixture.shells.qbat_count()?&qbat:nullptr,&welds,&beams,&solids};}
  fe::ShellPhysicalPublicationIdentity Identity() const {return {Configuration,Qualification,startup};}
  std::size_t WitnessCount() const {return fixture.shells.qbat_count()?3:2;}
  fe::NodalCinWitnessSource Witnesses() const {return {&cin,ranges.data(),witnesses.data(),ranges.size(),WitnessCount()};}
  n::TransactionConfig Config() const {
    auto result=fixture.Config();
    if(fixture.shells.qbat_count())result.activity=n::ContactActivityPolicy::AllActivePrefix;
    return result;
  }
  n::MovingMainSource Source() const {
    n::MovingMainSource result;
    static_cast<n::ContactSourceInput&>(result)=fixture.Contact();
    // Explicit constant-thickness contact in this synthetic declaration.
    if(fixture.shells.qbat_count())result.contact_thickness_update=0;
    result.starter=fixture.starter;
    result.activation={0,0,1,2,1,n::normal_activation::FreeRosterPolicy::FreshComplete};
    return result;
  }
  void Initialize(bool attach_contact=true);
  void Begin(FullLedgerAttempt&);
  void Prepare(FullLedgerAttempt&);
  void Seal(FullLedgerAttempt&);
  fe::ShellPublicationReport Commit(FullLedgerAttempt&,bool accept=true);
  void Discard();
  std::vector<double> Force(const FullLedgerAttempt&);
 private:
  void PrepareSolidModel();
  void PrepareOwner();
  void PrepareConstraint();
};
} // namespace type25_source_test
