// SPDX-License-Identifier: MIT
#pragma once
#include "lib_utest/qualification/qeph_candidate_diagnostics/Fixture.h"
#include "lib_src/elements/qeph/mapped/ObserverValues.h"
#include <array>
#include <vector>

namespace qeph_observer_test {
using namespace qeph_diagnostics_test;
struct Fixture {
  qeph_diagnostics_test::Fixture source;
  b::Layout layout;
  tl::util::HostArena arena;
  b::Storage* host=nullptr;
  std::vector<fe::ShellSectionLaw> roles;
  explicit Fixture(unsigned parents=Parents,unsigned epoch=1):roles(parents) {
    source.Reset(epoch);
    EXPECT_TRUE(layout.InitializeMapped(parents,Nodes,std::size_t{1}<<30));
    EXPECT_TRUE(arena.Initialize(layout.bytes));
    host=layout.Construct(arena);
    EXPECT_NE(host,nullptr);
    host->model.config=source.host->model.config;
    host->model.config.element_count=parents;
    host->model.joined=true;host->model.mapped=true;
    for(unsigned n=0;n<Nodes;++n) host->model.initial_position[n]=source.host->model.initial_position[n];
    for(unsigned p=0;p<parents;++p) {
      const auto original=p%Parents;
      host->model.element[p]=source.host->model.element[original];
      for(unsigned s=0;s<2;++s)host->slab[s].element[p]=source.host->slab[s].element[original];
      host->candidate_status[p]=q::Status::kSuccess;
      roles[p]=source.input.law[original];
    }
  }
  fe::NodalPreparedView View(unsigned epoch=1) {return source.fields.View(source.input,epoch);}
  void Prepare(unsigned epoch=1) {
    const auto view=View(epoch);
    for(unsigned p=0;p<roles.size();++p)
      m::PrepareDiagnosticParent(host->model,host->slab[1].element[p],host->candidate_status[p],roles.data(),p,view,host->assembly.parent[p]);
    for(unsigned n=0;n<Nodes;++n)m::PrepareDiagnosticNode(host->model,view,n,host->assembly.node[n]);
  }
  b::Control Serial(unsigned epoch=1,bool assembled=true,bool catalog=true) {
    fe::shell_batch_plasticity_detail::MixedDeviceStorage mixed;mixed.law=roles.data();
    frozen::FinalizeCandidate(host,&host->slab[0],&host->slab[1],View(epoch),Identity(epoch,assembled),catalog?&mixed:nullptr);
    return host->control;
  }
  m::ObserverSummary Reduce(unsigned epoch=1,bool assembled=true,bool catalog=true) {
    Prepare(epoch);
    const auto blocks=m::ObserverBlocks(roles.size(),Nodes);
    const auto view=View(epoch);
    const auto* law=catalog?roles.data():nullptr;
    std::array<m::ObserverSummary,m::ObserverThreads> lanes{};
    for(unsigned block=0;block<blocks;++block) {
      lanes={};
      for(unsigned lane=0;lane<m::ObserverThreads;++lane) {
        const auto first=block*m::ObserverThreads+lane;
        for(unsigned p=first;p<roles.size();p+=blocks*m::ObserverThreads)
          m::ObserveParent(*host,host->slab[0],host->slab[1],view,law,assembled,p,lanes[lane]);
        for(unsigned n=first;n<Nodes;n+=blocks*m::ObserverThreads)m::ObserveNode(*host,n,lanes[lane]);
      }
      for(unsigned d=m::ObserverThreads/2;d;d/=2)
        for(unsigned lane=0;lane<d;++lane)m::MergeObservations(lanes[lane],lanes[lane+d]);
      host->assembly.observer[block]=lanes[0];
    }
    lanes={};
    for(unsigned lane=0;lane<m::ObserverThreads;++lane)
      for(unsigned block=lane;block<blocks;block+=m::ObserverThreads)
        m::MergeObservations(lanes[lane],host->assembly.observer[block]);
    for(unsigned d=m::ObserverThreads/2;d;d/=2)
      for(unsigned lane=0;lane<d;++lane)m::MergeObservations(lanes[lane],lanes[lane+d]);
    return lanes[0];
  }
  b::Control Staged(unsigned epoch=1,bool assembled=true,bool catalog=true) {
    const auto summary=Reduce(epoch,assembled,catalog);
    b::Control out;
    m::FinalizeObservations(*host,host->slab[0],host->slab[1],View(epoch),Identity(epoch,assembled),
        catalog?roles.data():nullptr,summary,out);
    return out;
  }
};
inline std::array<double,m::ObserverChannels> Sums(const q::BatchDiagnostics& d) {
  return {d.internal_work[0],d.internal_work[1],d.internal_work_increment[0],d.internal_work_increment[1],
    d.hourglass_viscous_work,d.hourglass_viscous_work_increment,d.internal_kick_work,d.internal_drift_work};
}
inline void SameExceptSums(b::Control actual,const b::Control& serial) {
  const auto& s=serial.diagnostics;auto& a=actual.diagnostics;
  for(unsigned c=0;c<2;++c) {a.internal_work[c]=s.internal_work[c];a.internal_work_increment[c]=s.internal_work_increment[c];}
  a.hourglass_viscous_work=s.hourglass_viscous_work;a.hourglass_viscous_work_increment=s.hourglass_viscous_work_increment;
  a.internal_kick_work=s.internal_kick_work;a.internal_drift_work=s.internal_drift_work;
  SameControl(actual,serial);
}
} // namespace qeph_observer_test
