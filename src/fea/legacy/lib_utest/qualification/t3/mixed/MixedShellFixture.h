#pragma once
// MB1 prescribed transaction qualification only; no mixed force feedback.
#include "../mixed_binding/ShellBatchBindingFixture.h"
#include "../T3ForcePortFixture.h"
#include "../../qeph/QephForceFixture.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"

namespace mixed_shell_test {
namespace fe=tl::fea;
namespace q=fe::qeph;
namespace t=fe::t3;
namespace qnative=tl::qualification::qeph;
namespace tnative=tl::qualification::t3;
namespace qo=qeph_force_port_test;
namespace to=t3_force_port_test;
namespace temporal=tl_test::nodal_temporal;
using temporal::Snapshot;
using temporal::Loads;
using temporal::Read;
using temporal::SameState;
using temporal::SameStamp;
using temporal::Noop;
using shell_binding_test::Bytes;
using MixedShellCuda=temporal::NodalTemporalCuda;
constexpr unsigned Nodes=5;
constexpr double H=1./1024,Targets[4]{1,0,-1,0};
constexpr long double EnergyScale=1e-6L;
constexpr std::uint64_t Qualification=0x4d42315052455331ULL;
constexpr std::uint64_t Configuration=0x4d42314d4f444c31ULL;

struct Rig {
  fe::ShellBatchBindingInput input=shell_binding_test::Edge();
  fe::ShellBatchBinding binding;
  temporal::Initial initial;
  fe::FENodalState owner;
  q::QephBatch qeph;
  t::T3Batch t3;
  fe::ShellBatchPublication publication;
  bool PrepareReference();
  bool InitializeParticipants();
  bool Initialize();
  bool Bind();
  void Discard() { owner.Discard(); publication.DiscardTrial(); qeph.DiscardTrial(); t3.DiscardTrial(); }
};
struct Prepared {
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  Snapshot endpoint;
  Loads load;
};
struct Staged {
  q::ForceTrial qeph;
  t::ForceTrial t3;
  fe::ShellBatchDiagnostics diagnostics;
};
struct NativeTrials { qnative::ForceTrial qeph; tnative::ForceTrial t3; };
struct NativePair {
  qnative::Reference qeph; tnative::Reference t3;
  qnative::History qeph_history; tnative::History t3_history;
  bool Initialize(const Rig&);
  bool Check(const Rig&,const Prepared&,const Staged&,NativeTrials&) const;
  // Reference history advances by value only after the actual mixed commit.
  void Accept(const NativeTrials& trial) { qeph_history=trial.qeph.proposed_history; t3_history=trial.t3.proposed_history; }
};
Loads Schedule(const Rig&,unsigned interval);
bool Prepare(Rig&,const Loads&,Prepared&);
bool Endpoint(const fe::NodalPreparedView&,Snapshot&);
bool Evaluate(Rig&,const Prepared&,Staged&,bool t3_first=false);
// Reads both accepted slabs and typed family diagnostics. The common kinetic
// cache is read separately through publication.CopyAcceptedDiagnostics.
bool Accepted(Rig&,Staged&);
bool Publish(Rig&,const Prepared&,const Staged&);
fe::NodalValidationReceipt Receipt(const Staged&);
q::PrescribedInterval QephInterval(const Rig&,const Prepared&);
t::PrescribedInterval T3Interval(const Rig&,const Prepared&);
void Identity(const Rig&,const Prepared&,const Staged&);
void ExactResults(const Staged&,const Staged&);
void Property(const std::string&,double);
} // namespace mixed_shell_test
