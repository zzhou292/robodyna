#pragma once
#include "lib_src/collision/nodal_wall_mapped/ObserverValues.h"
#include "lib_src/collision/nodal_wall_mapped/Layout.h"
#include "lib_src/collision/NodalWallContactDiagnosticIdentity.h"
#include "SerialObservers.h"
#include <gtest/gtest.h>
#include <array>
#include <vector>
#include <limits>
namespace wall_observer_test {
namespace c=tlfea::contact;
namespace m=c::nodal_wall_mapped;
namespace d=c::nodal_wall_device_detail;
using Code=c::NodalWallDeviceStatus;
struct Fixture {
  d::Storage storage;
  std::vector<c::NodalWallPointResult> nodes;
  std::vector<double> positions;
  explicit Fixture(unsigned count=263):nodes(count),positions(3*count) {
    storage.model.node_count=count;storage.model.parent_count=7;storage.model.rate=.025;
    storage.result.nodes=nodes.data();
    for(unsigned i=0;i<count;++i) {
      auto& node=nodes[i];node.node=count-1-i;node.valid=true;
      const double f=::ldexp(1.0,int(i%29)-14),u=.125*f;
      EXPECT_TRUE(c::q4_bounds::Certify(f,{::nextafter(f,0),::nextafter(f,INFINITY)},&node.force));
      EXPECT_TRUE(c::q4_bounds::Certify(u,{::nextafter(u,0),::nextafter(u,INFINITY)},&node.potential));
      node.stiffness={1,1,1,0};node.wall_reaction={f,0,0};node.force_world={-f,0,0};
      node.wall_point={0,double(int(i%3)-1),double(int(i%5)-2)};
      node.wall_moment={0,node.wall_point.z*f,-node.wall_point.y*f};
      node.surface_power=-f*double(int(i%7)-3);
      positions[3*node.node]=i%5==0?-.001:1e-4*i;
    }
  }
  tl::fea::DeviceNodalKinematicsView Kinematics() const {
    tl::fea::DeviceNodalKinematicsView k;k.node_count=nodes.size();k.position_xyz=positions.data();return k;
  }
  static c::NodalWallDiagnostics Identity() {
    c::NodalWallDiagnostics out;out.owner_id=7;out.configuration_id=11;out.qualification_id=13;
    out.wall_binding_id=17;out.base_epoch=2;out.attempt=3;out.phase=c::NodalWallDevicePhase::AcceptedBase;return out;
  }
  m::ObserverSummary Reduce() const {
    const auto blocks=m::ObserverBlocks(nodes.size());
    std::vector<m::ObserverSummary> partial(blocks);
    std::array<m::ObserverSummary,m::ObserverThreads> lane{};
    for(unsigned b=0;b<blocks;++b) {
      lane={};
      for(unsigned t=0;t<m::ObserverThreads;++t)
        for(unsigned n=b*m::ObserverThreads+t;n<nodes.size();n+=blocks*m::ObserverThreads)
          m::ObserveNode(nodes[n],positions[3*nodes[n].node]-storage.model.config.law.wall_x,lane[t]);
      for(unsigned offset=m::ObserverThreads/2;offset;offset/=2)
        for(unsigned t=0;t<offset;++t)m::MergeObservers(lane[t],lane[t+offset]);
      partial[b]=lane[0];
    }
    lane={};
    for(unsigned t=0;t<m::ObserverThreads;++t)
      for(unsigned b=t;b<blocks;b+=m::ObserverThreads)m::MergeObservers(lane[t],partial[b]);
    for(unsigned offset=m::ObserverThreads/2;offset;offset/=2)
      for(unsigned t=0;t<offset;++t)m::MergeObservers(lane[t],lane[t+offset]);
    return lane[0];
  }
  c::NodalWallDiagnostics Serial(c::NodalWallDiagnostics seed=Identity()) {
    storage.control={};storage.result.diagnostics=seed;
    c::wall_observer_frozen::ReduceNodes<true>(storage,Kinematics());return storage.result.diagnostics;
  }
  c::NodalWallDiagnostics Staged(c::NodalWallDiagnostics seed=Identity()) {
    storage.control={};storage.result.diagnostics=seed;auto next=seed;
    if(!m::ApplyObservers(Reduce(),nodes.size(),next))c::wall_observer_frozen::ReduceNodes<true>(storage,Kinematics());
    else {next.node_count=storage.model.node_count;next.parent_count=storage.model.parent_count;
      next.stiffness_rate_bound=storage.model.rate;storage.result.diagnostics=next;}
    return storage.result.diagnostics;
  }
};
inline std::array<double,7> Signed(const c::NodalWallDiagnostics& d) {
  return {d.wall_reaction.x,d.wall_reaction.y,d.wall_reaction.z,d.wall_moment.x,d.wall_moment.y,d.wall_moment.z,d.surface_power};
}
} // namespace wall_observer_test
