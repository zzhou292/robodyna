#pragma once
#include "../vehicle_plasticity_catalog/VehicleCatalogFixture.h"
#include "../nodal_vehicle/VehicleOwnerFixture.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/elements/ShellResidentHostAccounting.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace vehicle_resident_test {
namespace fe=tl::fea;namespace q=fe::qeph;namespace t=fe::t3;
using CudaTest=tl::fea::vehicle_test::VehicleOwnerCuda;
constexpr double H=1./1048576;
constexpr std::uint64_t Configuration=0x565245534944,Qualification=0x565245535141;
struct Snapshot {
  std::vector<double> x,v,w,orientation,reaction,couple;fe::NodalStamp stamp;
  explicit Snapshot(std::size_t n):x(3*n),v(3*n),w(3*n),orientation(4*n),reaction(3*n),couple(3*n){}
  fe::NodalSnapshotBuffer buffer(){return {x.data(),v.data(),x.size()/3,orientation.data(),w.data(),reaction.data(),couple.data()};}
};
struct Results {
  std::vector<q::ForceTrial> qr;std::vector<t::ForceTrial> tr;
  std::vector<fe::ShellBatchSectionState> qs,ts;fe::ShellBatchDiagnostics diagnostics;
  Results(std::size_t nq,std::size_t nt,bool plastic):qr(nq),tr(nt),qs(plastic?nq:0),ts(plastic?nt:0){}
};
struct Prepared {fe::NodalTrialToken token;fe::NodalPreparedView view;Snapshot endpoint;explicit Prepared(std::size_t n):endpoint(n){}};
struct Rig {
  const std::size_t nq,nt,n;
  vehicle_catalog_test::Fixture source;fe::ShellBatchBinding binding;fe::ShellBatchPlasticityBinding catalog;
  Snapshot initial;std::vector<double> inverse,inverse_j;std::vector<std::uint8_t> fixed;
  fe::FENodalState owner;q::QephBatch qeph;t::T3Batch t3;fe::ShellBatchPublication publication;
  bool plastic=false;
  Rig(std::size_t q=vehicle_shell_test::Fixture::SourceQ,std::size_t t=vehicle_shell_test::Fixture::SourceT,
      std::size_t nodes=vehicle_shell_test::Fixture::SourceNodes);
  bool Initialize(bool);
  q::QephBatchConfig QConfig() const;t::T3BatchConfig TConfig() const;
  void Discard(){owner.Discard();publication.DiscardTrial();qeph.DiscardTrial();t3.DiscardTrial();}
};
bool Read(Rig&,Snapshot&);bool Accepted(Rig&,Results&);
bool Prepare(Rig&,Prepared&,bool pulse=false);bool Evaluate(Rig&,Prepared&,Results&);
bool Publish(Rig&,const Prepared&,const Results&);void Collapse(const Prepared&,std::size_t,std::size_t);
void SameResults(const Results&,const Results&);void SameSnapshot(const Snapshot&,const Snapshot&);
void CheckNativeTail(const Rig&,const Prepared&,const Results&,const Results&);
void CheckCompleteFields(const Rig&,const Prepared&,const Results&);
void ReportStorage(const Rig&);
} // namespace vehicle_resident_test
