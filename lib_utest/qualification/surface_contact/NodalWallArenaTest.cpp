#include "NodalWallCapacityFixture.h"
#include "lib_src/collision/NodalWallContactResultIO.h"

namespace {
using namespace nodal_wall_capacity_test;
using nodal_wall_owner_test::Bytes;
TEST(NodalWallArena, ExactAlignedRegionsHardBoundsOverflowAndByteFailuresAreAtomic) {
  detail::ArenaLayout l;
  ASSERT_TRUE(detail::BuildArenaLayout(Parents,Nodes,Nodes,sc::MaxActiveNodalWallDeviceBytes,l));
  EXPECT_GT(l.bytes,sc::MaxNodalWallDeviceBytes); EXPECT_LE(l.bytes,sc::MaxActiveNodalWallDeviceBytes);
  const auto saved=Bytes(l);
  for(unsigned fault=0;fault<7;++fault) {
    const auto p=fault==0?0:fault==1?1025:fault==2?SIZE_MAX:Parents;
    const auto n=fault==3?2049:Nodes,g=fault==4?Nodes-1:Nodes;
    const auto cap=fault==5?l.bytes-1:fault==6?SIZE_MAX:sc::MaxActiveNodalWallDeviceBytes;
    EXPECT_FALSE(detail::BuildArenaLayout(p,n,g,cap,l)); EXPECT_EQ(Bytes(l),saved);
  }
  const tl::util::ArenaRegion regions[]{l.header,l.parents,l.nodes,l.positions,l.inverse,l.fixed,l.status,l.shares,
      l.force,l.error,l.base.parents,l.base.nodes,l.base.wall_face,l.result.parents,l.result.nodes,l.result.wall_face};
  std::size_t end=0;
  for(const auto& r:regions) { EXPECT_GE(r.offset,end); EXPECT_LE(r.offset+r.bytes,l.bytes); end=r.offset+r.bytes; }
  EXPECT_EQ(end,l.bytes); EXPECT_EQ(l.force.count,6*Nodes); EXPECT_EQ(l.shares.count,4*Parents);
  EXPECT_EQ(l.result.parents.count,Parents); EXPECT_EQ(l.result.nodes.count,Nodes);
  EXPECT_TRUE(detail::BuildArenaLayout(1024,2048,2048,sc::MaxActiveNodalWallDeviceBytes,l));
  RecordProperty("maximum_layout_bytes",std::to_string(l.bytes));
}
TEST(NodalWallArena, CompleteNativePreparedModelRebasesEveryPointerAndPreservesLateFailures) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare()); detail::PreparedModel out;
  auto c=f->Config();
  const auto prepare=[&](const sc::NodalWallDeviceConfig& config,sc::VectorView x,const double* mass) {
    return detail::PrepareModel(config,f->wall.view(),f->weights,x,mass,f->fixed.data(),f->motion,&out); };
  ASSERT_EQ(prepare(c,f->Positions(),f->inverse.data()).status,Code::Ok);
  const auto& l=out.layout(); const auto saved=Bytes(out);
  const auto* begin=static_cast<const unsigned char*>(out.data());
  const std::vector<unsigned char> payload(begin,begin+l.bytes);
  auto bad=c; bad.limits={};
  const sc::VectorView unreadable{reinterpret_cast<const double*>(std::uintptr_t{8}),Nodes,3,1};
  EXPECT_EQ(prepare(bad,unreadable,unreadable.data).status,Code::ResourceLimit);
  bad=c; bad.max_device_bytes=l.bytes-1;
  EXPECT_EQ(prepare(bad,unreadable,unreadable.data).status,Code::ResourceLimit);
  bad=c; bad.max_host_bytes=detail::HostPreparationBytes(l)-1;
  EXPECT_EQ(prepare(bad,unreadable,unreadable.data).status,Code::ResourceLimit);
  f->inverse.back()=0; const auto late=prepare(c,f->Positions(),f->inverse.data());
  EXPECT_EQ(late.status,Code::InvalidMass); EXPECT_EQ(late.node,Nodes-1);
  EXPECT_EQ(Bytes(out),saved); EXPECT_EQ(std::memcmp(out.data(),payload.data(),payload.size()),0);
  f->inverse.back()=1/f->mass.back();
  ASSERT_EQ(prepare(c,f->Positions(),f->inverse.data()).status,Code::Ok);
  EXPECT_EQ(out.model().node_count,Nodes); EXPECT_EQ(out.model().parent_count,Parents);
  EXPECT_EQ(out.model().nodes[Nodes-1].node,Nodes-1);
  tl::util::HostArena remote; ASSERT_TRUE(remote.Initialize(out.layout().bytes));
  const auto shadow=out.Rebase(remote.data()); const auto& h=out.storage();
  const auto rebased=[&](const void* actual,const void* host) {
    EXPECT_EQ(static_cast<const unsigned char*>(actual)-static_cast<const unsigned char*>(remote.data()),
              static_cast<const unsigned char*>(host)-static_cast<const unsigned char*>(out.data())); };
  rebased(shadow.model.parents,h.model.parents); rebased(shadow.model.nodes,h.model.nodes);
  rebased(shadow.model.initial_position,h.model.initial_position); rebased(shadow.model.inverse_mass,h.model.inverse_mass);
  rebased(shadow.model.fixed,h.model.fixed); rebased(shadow.node_status,h.node_status); rebased(shadow.shares,h.shares);
  rebased(shadow.staged_force,h.staged_force); rebased(shadow.addition_error,h.addition_error);
  rebased(shadow.base.parents,h.base.parents); rebased(shadow.base.nodes,h.base.nodes); rebased(shadow.base.wall_face,h.base.wall_face);
  rebased(shadow.result.parents,h.result.parents); rebased(shadow.result.nodes,h.result.nodes); rebased(shadow.result.wall_face,h.result.wall_face);
  EXPECT_TRUE(shadow.model.query.prepared()); EXPECT_EQ(shadow.model.rate,h.model.rate);
  RecordProperty("active_device_bytes",std::to_string(out.layout().bytes));
  RecordProperty("host_admission_bytes",std::to_string(detail::HostPreparationBytes(out.layout())));
  // The late-scatter overflow fixture needs finite point forces/rate first;
  // its deliberately extreme load is only assembled, never time integrated.
  c.law.stiffness_per_area=1e308; c.law.parent_force_error=1e305; c.law.parent_energy_error=1e302;
  ASSERT_EQ(prepare(c,f->Positions(),f->inverse.data()).status,Code::Ok);
  EXPECT_TRUE(std::isfinite(out.model().rate)); EXPECT_GT(out.model().rate,1e305);
}
TEST(NodalWallArena, ActiveOutputPreflightRejectsEveryAliasedRangeAndWrongCapacityWithoutReads) {
  Results out; sc::NodalWallDiagnostics expected; auto v=out.View();
  ASSERT_TRUE(detail::ValidResultView(v,Parents,Nodes,&expected));
  const auto bytes=Bytes(out.diagnostics);
  for(unsigned i=0;i<4;++i) for(unsigned j=0;j<i;++j) {
    auto bad=v; void* p[]{bad.diagnostics,bad.parents,bad.nodes,bad.wall_face}; p[i]=p[j];
    bad.diagnostics=static_cast<sc::NodalWallDiagnostics*>(p[0]); bad.parents=static_cast<sc::NodalWallParentResult*>(p[1]);
    bad.nodes=static_cast<sc::NodalWallPointResult*>(p[2]); bad.wall_face=static_cast<std::uint64_t*>(p[3]);
    EXPECT_FALSE(detail::ValidResultView(bad,Parents,Nodes,&expected));
  }
  auto bad=v; bad.diagnostics=&expected; EXPECT_FALSE(detail::ValidResultView(bad,Parents,Nodes,&expected));
  bad=v; bad.nodes=reinterpret_cast<sc::NodalWallPointResult*>(UINTPTR_MAX-1);
  EXPECT_FALSE(detail::ValidResultView(bad,Parents,Nodes,&expected));
  bad=v; --bad.parent_capacity; bad.parents=reinterpret_cast<sc::NodalWallParentResult*>(std::uintptr_t{8});
  EXPECT_FALSE(detail::ValidResultView(bad,Parents,Nodes,reinterpret_cast<const sc::NodalWallDiagnostics*>(std::uintptr_t{8})));
  bad=v; ++bad.node_capacity; EXPECT_FALSE(detail::ValidResultView(bad,Parents,Nodes,&expected));
  bad=v; bad.parents=reinterpret_cast<sc::NodalWallParentResult*>(reinterpret_cast<unsigned char*>(v.parents)+1);
  EXPECT_FALSE(detail::ValidResultView(bad,Parents,Nodes,&expected));
  EXPECT_EQ(Bytes(out.diagnostics),bytes);
}
} // namespace
