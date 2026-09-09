#pragma once
// Shared bounded actual-owner fixtures; no mechanics or integration policy.
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/ExplicitTranslationStep.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>

namespace tl_test::nodal_temporal {
namespace fe=tl::fea;
using Code=fe::NodalStatus;
using Scheme=fe::NodalTemporalScheme;
using Phase=fe::NodalVelocityPhase;
constexpr std::size_t Capacity=fe::MaxTranslationNodes;
constexpr double ArithmeticTolerance=2e-13; // SI fixtures with O(1) motion/load.
struct Initial {
  std::size_t n=1;
  double h=.125;
  std::array<double,3*Capacity> x{},v{},omega{};
  std::array<double,4*Capacity> q{};
  std::array<double,Capacity> inverse{},inverse_inertia{};
  std::array<std::uint8_t,Capacity> fixed{},rotation_fixed{};
  Initial() {
    inverse.fill(.5); inverse_inertia.fill(.25);
    for(std::size_t i=0;i<Capacity;++i) q[4*i]=1;
  }
  fe::NodalReport Initialize(fe::FENodalState& owner,Scheme scheme=Scheme::StaggeredHalfKickStart) const {
    fe::NodalStateConfig config; config.node_count=n; config.fixed_dt=h; config.temporal_scheme=scheme;
    return owner.Initialize(config,{x.data(),v.data(),omega.data(),n,q.data()},inverse.data(),
                            fe::NodalDofConfig{fixed.data(),rotation_fixed.data(),inverse_inertia.data()});
  }
};
struct Loads { double force[3*Capacity]{},couple[3*Capacity]{}; };
static_assert(sizeof(Loads)+sizeof(fe::NodalAssemblyView)+sizeof(double)<4096,
              "The tiny test kernel argument packet must stay below 4 KiB");
struct Snapshot {
  std::array<double,3*Capacity> x{},v{},omega{},reaction{},couple{};
  std::array<double,4*Capacity> q{};
  fe::NodalStamp stamp;
  fe::NodalSnapshotBuffer buffer() {
    return {x.data(),v.data(),Capacity,q.data(),omega.data(),reaction.data(),couple.data()};
  }
};
inline void SameStamp(const fe::NodalStamp& a,const fe::NodalStamp& b) {
  EXPECT_EQ(a.owner_id,b.owner_id); EXPECT_EQ(a.epoch,b.epoch); EXPECT_EQ(a.node_count,b.node_count);
  EXPECT_EQ(a.time,b.time); EXPECT_EQ(a.fixed_dt,b.fixed_dt); EXPECT_EQ(a.has_rotations,b.has_rotations);
  EXPECT_EQ(a.reactions_valid,b.reactions_valid); EXPECT_EQ(a.reaction_base_epoch,b.reaction_base_epoch);
  EXPECT_EQ(a.reaction_time,b.reaction_time); EXPECT_EQ(a.reaction_kick_dt,b.reaction_kick_dt);
  EXPECT_EQ(a.temporal_scheme,b.temporal_scheme); EXPECT_EQ(a.velocity_phase,b.velocity_phase);
  EXPECT_EQ(a.velocity_time,b.velocity_time);
}
inline void SameState(const Snapshot& a,const Snapshot& b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.q,b.q); EXPECT_EQ(a.omega,b.omega);
  EXPECT_EQ(a.reaction,b.reaction); EXPECT_EQ(a.couple,b.couple); SameStamp(a.stamp,b.stamp);
}
inline bool Read(fe::FENodalState& owner,Snapshot& out) {
  const auto r=owner.CopyAccepted(out.buffer(),&out.stamp);
  EXPECT_EQ(r.status,Code::Ok)<<r.message; return r.status==Code::Ok;
}
static __global__ void AddLoads(fe::NodalAssemblyView view,Loads load,double scale) {
  for(unsigned i=0;i<view.accepted.node_count;++i) {
    view.forces.force_x[i]+=scale*load.force[3*i];
    view.forces.force_y[i]+=scale*load.force[3*i+1];
    view.forces.force_z[i]+=scale*load.force[3*i+2];
    view.forces.couple_x[i]+=scale*load.couple[3*i];
    view.forces.couple_y[i]+=scale*load.couple[3*i+1];
    view.forces.couple_z[i]+=scale*load.couple[3*i+2];
  }
}
static __global__ void AddBound(fe::NodalAssemblyView view) { view.bounds->stiffness[0]=1; }
static __global__ void Noop() {}
inline fe::NodalStaggeredPrescribedAdmission Admission(const fe::FENodalState& owner,const fe::NodalAssemblyView& view) {
  return {view.owner_id,view.accepted.base_epoch,view.attempt,owner.accepted().fixed_dt,2};
}
inline bool BeginLoad(fe::FENodalState& owner,const Loads& loads,fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
  const auto r=owner.BeginTrial(&token,&view); EXPECT_EQ(r.status,Code::Ok)<<r.message;
  if(r.status!=Code::Ok) return false;
  AddLoads<<<1,1,0,view.stream>>>(view,loads,.5);
  AddLoads<<<1,1,0,view.stream>>>(view,loads,.5);
  const auto e=cudaPeekAtLastError(); EXPECT_EQ(e,cudaSuccess); return e==cudaSuccess;
}
inline bool Prepare(fe::FENodalState& owner,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view,
             fe::NodalPreparedView& prepared) {
  auto r=owner.SealAssembly(token); EXPECT_EQ(r.status,Code::Ok)<<r.message;
  if(r.status!=Code::Ok) return false;
  r=fe::AdvanceStaggeredPrescribed(owner,token,Admission(owner,view)); EXPECT_EQ(r.status,Code::Ok)<<r.message;
  if(r.status!=Code::Ok) return false;
  r=owner.BorrowPrepared(token,&prepared); EXPECT_EQ(r.status,Code::Ok)<<r.message;
  return r.status==Code::Ok;
}
inline bool StepConstant(fe::FENodalState& owner,const Loads& loads) {
  fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
  if(!BeginLoad(owner,loads,token,view)||!Prepare(owner,token,view,prepared)) return false;
  const auto r=owner.Commit(token); EXPECT_EQ(r.status,Code::Ok)<<r.message; return r.status==Code::Ok;
}
class NodalTemporalCuda:public ::testing::Test {
  void SetUp() override { int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0); }
};

} // namespace tl_test::nodal_temporal
