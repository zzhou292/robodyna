#pragma once
// Qualification composition only: existing owner, typed batches and native
// one-cell oracles. No alternative solver, contact or runtime admission.
#include "../mixed/MixedShellFixture.h"
#include "lib_src/elements/ShellCollectionLimits.h"
#include <memory>

namespace resident_collection_test {
namespace fe=tl::fea;
namespace q=fe::qeph;
namespace t=fe::t3;
namespace qn=tl::qualification::qeph;
namespace tn=tl::qualification::t3;
namespace qo=qeph_force_port_test;
namespace to=t3_force_port_test;
namespace temporal=tl_test::nodal_temporal;
using CudaTest=temporal::NodalTemporalCuda;
using shell_binding_test::Bytes;
constexpr std::size_t Capacity=fe::MaxShellCollectionNodes,Nodes=117,QCount=88,TCount=16;
constexpr double H=1./8192;
constexpr long double EnergyScale=1e-6L;
constexpr std::uint64_t Qualification=0x4d434f4c46444231ULL,Configuration=0x4d434f4c4d4f4431ULL;
static_assert(Nodes<=Capacity&&QCount+TCount<=fe::MaxShellCollectionParents);
static_assert(Capacity<=fe::MaxTranslationNodes);

struct Snapshot {
  std::array<double,3*Capacity> x{},v{},omega{},reaction{},couple{};
  std::array<double,4*Capacity> orientation{};
  fe::NodalStamp stamp;
  fe::NodalSnapshotBuffer buffer() {
    return {x.data(),v.data(),Capacity,orientation.data(),omega.data(),reaction.data(),couple.data()};
  }
};
struct Staged {
  std::array<q::ForceTrial,QCount> qeph{};
  std::array<t::ForceTrial,TCount> t3{};
  fe::ShellBatchDiagnostics diagnostics;
};
// Components are interleaved at the test boundary. Actual owner RHS readback
// remains component-major, exactly as the production assembly contract.
struct Loads { std::array<double,6*Capacity> values{}; };
struct Prepared {
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  Snapshot endpoint;
  std::array<double,6*Capacity> rhs{};
};
struct Rig {
  std::array<fe::ShellQephBindingInput,QCount> qinput{};
  std::array<fe::ShellT3BindingInput,TCount> tinput{};
  fe::ShellBatchBinding binding;
  std::array<double,3*Capacity> x{},zero{};
  std::array<double,4*Capacity> orientation{};
  std::array<double,Capacity> inverse{},inverse_j{};
  std::array<std::uint8_t,Capacity> fixed{};
  fe::FENodalState owner;
  q::QephBatch qeph;
  t::T3Batch t3;
  fe::ShellBatchPublication publication;
  double* device_loads=nullptr; // One test-only allocation, never a kernel packet.
  ~Rig();
  bool BuildReference();
  bool InitializeOwner();
  bool InitializeParticipants();
  bool Bind();
  bool Initialize();
  q::QephBatchConfig QConfig() const;
  t::T3BatchConfig TConfig() const;
  void Discard() { owner.Discard(); publication.DiscardTrial(); qeph.DiscardTrial(); t3.DiscardTrial(); }
};
struct NativeSequence {
  std::array<qn::Reference,QCount> qr;
  std::array<tn::Reference,TCount> tr;
  std::array<qn::History,QCount> qhistory;
  std::array<tn::History,TCount> thistory;
  std::array<qn::ForceTrial,QCount> qtrial;
  std::array<tn::ForceTrial,TCount> ttrial;
  bool Initialize(const Rig&);
  bool Check(const Rig&,const Prepared&,const Staged&);
  void Accept();
};
bool Read(fe::FENodalState&,Snapshot&);
bool Accepted(Rig&,Staged&);
bool Prepare(Rig&,const Loads&,const Staged&,Prepared&);
bool Evaluate(Rig&,const Prepared&,Staged&,bool t3_first=false);
bool Publish(Rig&,const Prepared&,const Staged&);
Loads Pulse(const Rig&);
q::PrescribedInterval QInterval(const Rig&,std::size_t,const Prepared&);
t::PrescribedInterval TInterval(const Rig&,std::size_t,const Prepared&);
void CheckReference(const Rig&);
void CheckLedgers(const Rig&,const Snapshot&,const Staged&,const Prepared&,const Staged&);
void SameState(const Snapshot&,const Snapshot&,bool same_owner=true);
void ExactResults(const Staged&,const Staged&);
void Property(const char*,double);
} // namespace resident_collection_test
