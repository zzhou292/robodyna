#pragma once
#include "../NodalMassCudaFixture.h"
#include "../../type25/EvaluationValues.h"

namespace nodal_mass_test::joined_connector {
using PS=fe::ShellPublicationStatus;
using CS=spring::BatchStatus;
struct Candidate {
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView view;
  q::BatchDiagnostics qd;
  t::BatchDiagnostics td;
  spring::BatchDiagnostics cd;
  fe::ShellBatchDiagnostics common;
};
struct Accepted {
  temporal::Snapshot nodes;
  q::ForceTrial qeph;
  t::ForceTrial t3;
  std::array<spring::Evaluation,2> connectors;
  fe::ShellBatchDiagnostics common;
};
struct Rig:Joined {
  spring::Batch connector;
  fe::ShellBatchPublication publication; // Dies before all borrowed batches.
  bool Start(bool moving=false);
  void Discard();
  bool Read(Accepted&);
  bool Begin(Candidate&,const temporal::Loads&,bool with_connector=true);
  bool Evaluate(Candidate&,bool with_connector=true);
  bool Commit(Candidate&);
};
void SameAccepted(const Accepted&,const Accepted&);
void CheckKinetic(const Rig&,const temporal::Snapshot&,const fe::ShellBatchKinetic&);
void CheckCacheScatter(Rig&,const Accepted&,const Candidate&,const temporal::Loads&);
} // namespace nodal_mass_test::joined_connector
