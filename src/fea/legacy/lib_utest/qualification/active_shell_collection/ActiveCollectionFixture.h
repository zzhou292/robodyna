#pragma once
#include "../host_shell_collection/HostShellFixture.h"
#include "../nodal/NodalTemporalFixture.h"
#include "StorageExpectations.h"

namespace active_shell_test {
namespace q=fe::qeph;
namespace t=fe::t3;
using CudaTest=tl_test::nodal_temporal::NodalTemporalCuda;
using host_shell_test::QCount;
using host_shell_test::TCount;
using host_shell_test::NodeCount;
constexpr double H=1./1048576;
constexpr std::uint64_t Configuration=0x41435449564531ULL,Qualification=0x41435451554131ULL;
struct Snapshot {
  std::vector<double> x=std::vector<double>(3*NodeCount),v=x,w=x,orientation=std::vector<double>(4*NodeCount);
  fe::NodalStamp stamp;
  fe::NodalSnapshotBuffer buffer() { return {x.data(),v.data(),NodeCount,orientation.data(),w.data()}; }
};
struct Results {
  std::vector<q::ForceTrial> qr=std::vector<q::ForceTrial>(QCount);
  std::vector<t::ForceTrial> tr=std::vector<t::ForceTrial>(TCount);
  std::vector<fe::ShellBatchSectionState> qs=std::vector<fe::ShellBatchSectionState>(QCount),ts=std::vector<fe::ShellBatchSectionState>(TCount);
  fe::ShellBatchDiagnostics diagnostics;
};
struct Prepared { fe::NodalTrialToken token; fe::NodalPreparedView view; Snapshot endpoint; };
struct Rig {
  host_shell_test::Fixture source;
  fe::ShellBatchBinding binding;
  fe::ShellBatchPlasticityBinding catalog;
  Snapshot initial;
  std::vector<double> inverse=std::vector<double>(NodeCount),inverse_j=inverse;
  std::vector<std::uint8_t> fixed=std::vector<std::uint8_t>(NodeCount);
  fe::FENodalState owner;
  q::QephBatch qeph;
  t::T3Batch t3;
  fe::ShellBatchPublication publication;
  bool plastic=false;
  Rig();
  bool Initialize(bool with_plastic);
  bool BuildReference();
  q::QephBatchConfig QConfig() const;
  t::T3BatchConfig TConfig() const;
  void Discard() { owner.Discard(); publication.DiscardTrial(); qeph.DiscardTrial(); t3.DiscardTrial(); }
};
bool Read(Rig&,Snapshot&);
bool Accepted(Rig&,Results&);
bool Prepare(Rig&,Prepared&,bool pulse=false);
bool Evaluate(Rig&,Prepared&,Results&);
bool Publish(Rig&,const Prepared&,const Results&);
void Collapse(const Prepared&,std::size_t target,std::size_t source);
void SameResults(const Results&,const Results&,bool plastic);
void CheckTailHost(const Rig&,const Prepared&,const Results& base,const Results& next);
}
