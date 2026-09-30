#pragma once
#include "GroupOwnerFixture.h"
#include <cstring>

namespace force_stage_capture_test {
using namespace rigid_owner_test;
inline fe::NodalStateConfig Config(const Fixture& fixture,bool capture=true) {
  fe::NodalStateConfig c;c.node_count=fixture.input.n;c.fixed_dt=fixture.input.h;
  c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;c.capture_force_stage_accelerations=capture;return c;
}
inline fe::NodalReport Initialize(const Fixture& f,fe::FENodalState& owner,fe::NodalStateConfig c) {
  return owner.Initialize(c,{f.input.x.data(),f.input.v.data(),f.input.omega.data(),f.input.n,f.input.q.data()},
    f.input.inverse.data(),{f.input.fixed.data(),f.input.rotation_fixed.data(),f.input.inverse_inertia.data()},f.model);
}
struct Capture {
  std::array<double,3*nt::Capacity> a{},ar{};
  std::array<fe::NodalRigidGroupAccelerationSnapshot,2> groups{};
  fe::NodalPreparedView prepared{};
  Capture() {a.fill(-123);ar.fill(-456);prepared.owner_id=919;for(auto& g:groups)g.source_group_id=717;}
  fe::NodalForceStageSnapshotBuffer buffer() {return {a.data(),ar.data(),nt::Capacity,groups.data(),groups.size()};}
};
template<class T> auto Image(const T& value) {return rigid_step_test::Bytes(value);}
inline void SameCapture(const Capture& a,const Capture& b) {
  EXPECT_EQ(a.a,b.a);EXPECT_EQ(a.ar,b.ar);EXPECT_EQ(Image(a.groups),Image(b.groups));
  EXPECT_EQ(Image(a.prepared),Image(b.prepared));
}
inline Vec3 CaptureNode(const double* x,std::size_t n) {return {x[3*n],x[3*n+1],x[3*n+2]};}
} // namespace force_stage_capture_test
