#include "lib_src/solvers/ExplicitTranslationStep.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <new>

namespace {
namespace fe = tl::fea;
namespace sc = tlfea::contact;
namespace st = tl::fea::stability;
using NS = fe::NodalStatus;
struct Initial {
  std::array<double, 3*fe::MaxTranslationNodes> x{}, v{}, omega{};
  std::array<double, fe::MaxTranslationNodes> inverse{};
  std::array<std::uint8_t, fe::MaxTranslationNodes> fixed{};
  std::size_t n = 1;
  Initial() { inverse.fill(1); }
  fe::HostNodalKinematicsView view() const { return {x.data(), v.data(), omega.data(), n}; }
};
struct Snapshot {
  std::array<double, 3*fe::MaxTranslationNodes> x{}, v{};
  fe::NodalStamp stamp;
};
fe::NodalStateConfig Config(const Initial& in, double h = .01) {
  fe::NodalStateConfig c; c.node_count = in.n; c.fixed_dt = h; c.max_device_bytes = 32768; return c;
}
void ExpectSame(const Snapshot& a, const Snapshot& b) {
  EXPECT_EQ(std::memcmp(a.x.data(), b.x.data(), sizeof(a.x)), 0);
  EXPECT_EQ(std::memcmp(a.v.data(), b.v.data(), sizeof(a.v)), 0);
  EXPECT_EQ(a.stamp.owner_id, b.stamp.owner_id); EXPECT_EQ(a.stamp.epoch, b.stamp.epoch);
  EXPECT_EQ(a.stamp.time, b.stamp.time); EXPECT_EQ(a.stamp.fixed_dt, b.stamp.fixed_dt);
  EXPECT_EQ(a.stamp.node_count, b.stamp.node_count);
}
bool Read(fe::FENodalState& state, Snapshot* result) {
  const auto report = state.CopyAccepted({result->x.data(), result->v.data(), fe::MaxTranslationNodes}, &result->stamp);
  EXPECT_EQ(report.status, NS::Ok) << report.message; return report.status == NS::Ok;
}
bool Finish(fe::FENodalState& state, const fe::NodalTrialToken& token) {
  auto report = state.SealAssembly(token); EXPECT_EQ(report.status, NS::Ok) << report.message;
  if (report.status != NS::Ok) return false;
  report = fe::AdvanceTranslations(state, token); EXPECT_EQ(report.status, NS::Ok) << report.message;
  if (report.status != NS::Ok) return false;
  report = state.Commit(token); EXPECT_EQ(report.status, NS::Ok) << report.message;
  return report.status == NS::Ok;
}
__device__ bool Current(fe::NodalAssemblyView view) {
  if (view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->sealed || !view.bounds->valid) {
    fe::RecordNodalAssemblyFailure(view, sc::Status::kStaleTrial); return false;
  }
  return true;
}
__global__ void AddForce(fe::NodalAssemblyView view, unsigned node, sc::Vec3 force) {
  if (!Current(view)) return;
  if (node >= view.accepted.node_count) {
    fe::RecordNodalAssemblyFailure(view, sc::Status::kOutOfRange, node); return;
  }
  view.forces.force_x[node] += force.x;
  view.forces.force_y[node] += force.y;
  view.forces.force_z[node] += force.z;
}
__global__ void AddSpring(fe::NodalAssemblyView view, unsigned node, double k, double c) {
  if (!Current(view)) return;
  sc::NormalJacobian jacobian;
  const sc::SignedNodeWeight weight{node, 1};
  auto result = sc::BuildNormalJacobian(view.mass, &weight, 1, {1,0,0}, view.attempt, &jacobian);
  st::RowContribution contribution;
  if (result == sc::Status::kOk) result = st::MakeRankOneContribution(jacobian, k, c, &contribution);
  if (result == sc::Status::kOk) result = st::AccumulateRows(view.bounds, contribution);
  if (result != sc::Status::kOk) { fe::RecordNodalAssemblyFailure(view, result, node); return; }
  view.forces.force_x[node] -= k*view.accepted.position_xyz[3*node] + c*view.accepted.velocity_xyz[3*node];
}
__global__ void AddCouple(fe::NodalAssemblyView view) { view.forces.couple_z[0] = 1; }
__global__ void FailContributor(fe::NodalAssemblyView view) {
  fe::RecordNodalAssemblyFailure(view, sc::Status::kOutOfRange, 17);
}
__global__ void Noop() {}
class NodalStepCuda : public ::testing::Test {
  void SetUp() override { int n=0; ASSERT_EQ(cudaGetDeviceCount(&n), cudaSuccess); ASSERT_GT(n,0); }
};

TEST_F(NodalStepCuda, TwoContributorsDriveSharedMassesAndPreserveFixedNodes) {
  Initial in; in.n=3; in.inverse[0]=.5; in.inverse[1]=.25; in.inverse[2]=0; in.fixed[2]=1;
  in.x[6]=3; in.x[7]=-2; in.x[8]=5;
  fe::FENodalState state; ASSERT_EQ(state.Initialize(Config(in,.1),in.view(),in.inverse.data(),in.fixed.data()).status, NS::Ok);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status, NS::Ok);
  EXPECT_EQ(view.owner_id,state.accepted().owner_id);
  for (double magnitude : {4.,6.}) {
    AddForce<<<1,1,0,view.stream>>>(view,0,{magnitude,0,0});
    AddForce<<<1,1,0,view.stream>>>(view,1,{-magnitude,0,0});
  }
  AddForce<<<1,1,0,view.stream>>>(view,2,{17,-9,4});
  ASSERT_TRUE(Finish(state,token));
  Snapshot result; ASSERT_TRUE(Read(state,&result));
  EXPECT_NEAR(result.v[0],.5,1e-15); EXPECT_NEAR(result.v[3],-.25,1e-15);
  EXPECT_NEAR(result.x[0],.05,1e-15); EXPECT_NEAR(result.x[3],-.025,1e-15);
  EXPECT_NEAR(2*result.v[0]+4*result.v[3],0,1e-15); // Independent momentum/impulse balance.
  EXPECT_NEAR(.5*2*result.v[0]*result.v[0]+.5*4*result.v[3]*result.v[3],.375,1e-15);
  for (unsigned a=0;a<3;++a) { EXPECT_EQ(result.x[6+a],in.x[6+a]); EXPECT_EQ(result.v[6+a],0); }
  ASSERT_EQ(state.BeginTrial(&token,&view).status, NS::Ok); // No loads: previous forces must be cleared.
  ASSERT_TRUE(Finish(state,token)); ASSERT_TRUE(Read(state,&result));
  EXPECT_NEAR(result.x[0],.1,1e-15); EXPECT_NEAR(result.x[3],-.05,1e-15);
  EXPECT_NEAR(result.v[0],.5,1e-15); EXPECT_EQ(result.stamp.epoch,2u);
}

TEST_F(NodalStepCuda, FreeFlightAtLegacyContributorCapacityKeepsOneSharedNodeSpace) {
  Initial in; in.n=fe::MaxTranslationNodes;
  for (std::size_t i=0;i<3*in.n;++i) { in.x[i]=double(i)/100; in.v[i]=(int(i%3)-1)*.2; }
  fe::FENodalState state; ASSERT_EQ(state.Initialize(Config(in,.125),in.view(),in.inverse.data(),in.fixed.data()).status, NS::Ok);
  const auto allocated=state.allocations(); EXPECT_EQ(allocated.device_allocations,6u); EXPECT_LE(allocated.device_bytes,32768u);
  for (int step=0;step<4;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view; ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
    EXPECT_EQ(view.accepted.node_count,in.n); EXPECT_EQ(view.forces.base_epoch,std::uint64_t(step));
    ASSERT_TRUE(Finish(state,token));
  }
  Snapshot result; ASSERT_TRUE(Read(state,&result));
  for (std::size_t i=0;i<3*in.n;++i) { EXPECT_NEAR(result.x[i],in.x[i]+.5*in.v[i],2e-15); EXPECT_EQ(result.v[i],in.v[i]); }
  EXPECT_EQ(state.allocations().device_bytes,allocated.device_bytes); EXPECT_EQ(state.allocations().device_allocations,allocated.device_allocations);
}

TEST_F(NodalStepCuda, ForceDrivenOscillatorConvergesAndPreservesItsDiscreteInvariant) {
  std::array<double,3> error{};
  for (int level=0;level<3;++level) {
    const double h=.05/(1<<level); const int steps=20*(1<<level);
    Initial in; in.x[0]=1; fe::FENodalState state;
    ASSERT_EQ(state.Initialize(Config(in,h),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
    const auto allocations=state.allocations(); const double* slab[2]{};
    Snapshot result;
    for (int step=0;step<steps;++step) {
      fe::NodalTrialToken token; fe::NodalAssemblyView view; ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
      if (step<2) slab[step]=view.accepted.position_xyz;
      else EXPECT_EQ(view.accepted.position_xyz,slab[step%2]);
      // Independent structural batches share one force reset and bound sum.
      AddSpring<<<1,1,0,view.stream>>>(view,0,1.5,0);
      AddSpring<<<1,1,0,view.stream>>>(view,0,2.5,0);
      ASSERT_TRUE(Finish(state,token)); ASSERT_TRUE(Read(state,&result));
      const double x=result.x[0],v=result.v[0];
      const double energy=.5*v*v+2*x*x;
      EXPECT_NEAR(energy-2*h*x*v,2,3e-12); // Analytically derived symplectic-Euler invariant.
      EXPECT_LE(std::fabs(energy-2),6*h);
      EXPECT_EQ(state.allocations().device_bytes,allocations.device_bytes);
      EXPECT_EQ(state.allocations().device_allocations,allocations.device_allocations);
    }
    EXPECT_NE(slab[0],slab[1]); EXPECT_NEAR(result.stamp.time,1,2e-15);
    error[level]=std::hypot(result.x[0]-std::cos(2.),result.v[0]+2*std::sin(2.));
  }
  EXPECT_LT(error[1],.6*error[0]); EXPECT_LT(error[2],.6*error[1]); EXPECT_LT(error[2],.013);
}

TEST_F(NodalStepCuda, DampedMotionMatchesIndependentContinuousReferenceUnderRefinement) {
  double previous=1;
  for (int level=0;level<3;++level) {
    Initial in; in.x[0]=1; const double h=.02/(1<<level); const int steps=25*(1<<level);
    fe::FENodalState state; ASSERT_EQ(state.Initialize(Config(in,h),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
    for (int i=0;i<steps;++i) {
      fe::NodalTrialToken token; fe::NodalAssemblyView view; ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
      AddSpring<<<1,1,0,view.stream>>>(view,0,4,1); ASSERT_TRUE(Finish(state,token));
    }
    Snapshot out; ASSERT_TRUE(Read(state,&out)); const double w=std::sqrt(3.75),t=.5,e=std::exp(-.5*t);
    const double x=e*(std::cos(w*t)+.5/w*std::sin(w*t));
    const double v=-4/w*e*std::sin(w*t);
    const double error=std::hypot(out.x[0]-x,out.v[0]-v);
    if (level) EXPECT_LT(error,.6*previous); previous=error;
    EXPECT_LT(.5*out.v[0]*out.v[0]+2*out.x[0]*out.x[0],2);
  }
}

TEST_F(NodalStepCuda, InvalidStartupIsRejectedBeforeOwnerPublication) {
  Initial in; const auto good=Config(in);
  auto reject=[&](fe::NodalStateConfig c, NS expected) {
    fe::FENodalState state; EXPECT_EQ(state.Initialize(c,in.view(),in.inverse.data(),in.fixed.data()).status,expected);
    EXPECT_EQ(state.accepted().owner_id,0u); EXPECT_EQ(state.allocations().device_bytes,0u);
  };
  auto c=good; c.node_count=fe::MaxNodalStateNodes+1; reject(c,NS::ResourceLimit);
  c=good; c.node_count=0; reject(c,NS::ResourceLimit);
  c=good; c.max_device_bytes=1; reject(c,NS::ResourceLimit);
  c=good; c.max_device_bytes=fe::MaxActiveNodalStateDeviceBytes+1; reject(c,NS::ResourceLimit);
  for (double h : {0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    c=good; c.fixed_dt=h; reject(c,NS::InvalidInput);
  }
  c=good; c.minimum_dt=.1; reject(c,NS::InvalidInput);
  c=good; c.timestep_safety=1; reject(c,NS::InvalidInput);
  in.inverse[0]=0; reject(good,NS::InvalidInput); in.inverse[0]=-1; reject(good,NS::InvalidInput);
  in.inverse[0]=1; in.fixed[0]=2; reject(good,NS::InvalidInput);
  in.fixed[0]=1; reject(good,NS::InvalidInput); in.inverse[0]=0; in.v[0]=1; reject(good,NS::InvalidInput);
  in.fixed[0]=0; in.inverse[0]=1; in.v[0]=0; in.omega[1]=.1; reject(good,NS::UnsupportedRotation);
  in.omega[1]=0; in.x[0]=std::numeric_limits<double>::quiet_NaN(); reject(good,NS::InvalidInput);
}

TEST_F(NodalStepCuda, TokenPhaseForeignOwnerAndReinitializationRejectionsPreserveAcceptedState) {
  Initial in; fe::FENodalState a,b;
  ASSERT_EQ(a.Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  ASSERT_EQ(b.Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  Snapshot before,after; ASSERT_TRUE(Read(a,&before)); fe::NodalTrialToken t,foreign; fe::NodalAssemblyView view;
  ASSERT_EQ(a.BeginTrial(&t,&view).status,NS::Ok);
  EXPECT_EQ(fe::AdvanceTranslations(a,t).status,NS::WrongPhase);
  ASSERT_EQ(a.BeginTrial(&t,&view).status,NS::Ok); auto stale=t;
  ASSERT_EQ(a.BeginTrial(&t,&view).status,NS::Ok); EXPECT_EQ(a.SealAssembly(stale).status,NS::StaleTrial);
  ASSERT_EQ(b.BeginTrial(&foreign,&view).status,NS::Ok);
  ASSERT_EQ(a.BeginTrial(&t,&view).status,NS::Ok); EXPECT_EQ(a.SealAssembly(foreign).status,NS::StaleTrial);
  ASSERT_EQ(a.BeginTrial(&t,&view).status,NS::Ok); ASSERT_EQ(a.SealAssembly(t).status,NS::Ok);
  EXPECT_EQ(a.Commit(t).status,NS::WrongPhase);
  ASSERT_EQ(a.BeginTrial(&t,&view).status,NS::Ok); a.Discard(); EXPECT_EQ(a.SealAssembly(t).status,NS::WrongPhase);
  EXPECT_EQ(a.Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::InvalidInput);
  ASSERT_TRUE(Read(a,&after)); ExpectSame(before,after);
  ASSERT_EQ(a.BeginTrial(&t,&view).status,NS::Ok); ASSERT_TRUE(Finish(a,t));
  EXPECT_EQ(a.Commit(t).status,NS::StaleTrial); EXPECT_EQ(a.accepted().epoch,1u);
}

TEST_F(NodalStepCuda, ReusedOwnerAddressDoesNotReviveAnOldToken) {
  alignas(fe::FENodalState) unsigned char storage[sizeof(fe::FENodalState)];
  Initial in; fe::NodalTrialToken old,current; fe::NodalAssemblyView view;
  const auto destroy=[](fe::FENodalState* state) { state->~FENodalState(); };
  std::unique_ptr<fe::FENodalState,decltype(destroy)> lifetime(new(storage) fe::FENodalState,destroy);
  auto* first=lifetime.get();
  ASSERT_EQ(first->Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  ASSERT_EQ(first->BeginTrial(&old,&view).status,NS::Ok); const auto identity=first->accepted().owner_id;
  lifetime.reset();
  lifetime.reset(new(storage) fe::FENodalState); auto* replacement=lifetime.get();
  ASSERT_EQ(replacement->Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  EXPECT_NE(replacement->accepted().owner_id,identity);
  ASSERT_EQ(replacement->BeginTrial(&current,&view).status,NS::Ok);
  EXPECT_EQ(replacement->SealAssembly(old).status,NS::StaleTrial); EXPECT_EQ(replacement->accepted().epoch,0u);
}

TEST_F(NodalStepCuda, ContributorNonfiniteAndCoupleFailuresDiscardWithoutAdvancing) {
  Initial in; fe::FENodalState state; ASSERT_EQ(state.Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  Snapshot before,after; ASSERT_TRUE(Read(state,&before));
  for (int mode=0;mode<3;++mode) {
    fe::NodalTrialToken t; fe::NodalAssemblyView view; ASSERT_EQ(state.BeginTrial(&t,&view).status,NS::Ok);
    AddForce<<<1,1,0,view.stream>>>(view,0,{1,2,3});
    if (mode==0) FailContributor<<<1,1,0,view.stream>>>(view);
    if (mode==1) AddForce<<<1,1,0,view.stream>>>(view,0,{std::numeric_limits<double>::infinity(),0,0});
    if (mode==2) AddCouple<<<1,1,0,view.stream>>>(view);
    const auto report=state.SealAssembly(t);
    EXPECT_EQ(report.status,mode==0?NS::ContributorFailure:(mode==1?NS::InvalidOutput:NS::UnsupportedRotation));
    if (mode==0) EXPECT_EQ(report.node,17u);
    EXPECT_EQ(state.Commit(t).status,NS::WrongPhase); ASSERT_TRUE(Read(state,&after)); ExpectSame(before,after);
  }
  fe::NodalTrialToken t; fe::NodalAssemblyView view; ASSERT_EQ(state.BeginTrial(&t,&view).status,NS::Ok);
  AddForce<<<1,1,0,view.stream>>>(view,0,{1,0,0}); ASSERT_TRUE(Finish(state,t));
  ASSERT_TRUE(Read(state,&after)); EXPECT_NEAR(after.v[0],.01,1e-16);
}

TEST_F(NodalStepCuda, CombinedStiffnessRejectsAnOtherwiseIndividuallyStableFixedStep) {
  Initial in; in.x[0]=1; fe::FENodalState state;
  ASSERT_EQ(state.Initialize(Config(in,.15),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  Snapshot before,after; ASSERT_TRUE(Read(state,&before)); fe::NodalTrialToken t; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&t,&view).status,NS::Ok);
  AddSpring<<<1,1,0,view.stream>>>(view,0,100,0); AddSpring<<<1,1,0,view.stream>>>(view,0,100,0);
  const auto report=state.SealAssembly(t); EXPECT_EQ(report.status,NS::StepTooLarge);
  EXPECT_NEAR(report.stable_dt,.8*std::sqrt(4./200),2e-14); EXPECT_LT(report.stable_dt,.15);
  EXPECT_EQ(fe::AdvanceTranslations(state,t).status,NS::WrongPhase);
  ASSERT_TRUE(Read(state,&after)); ExpectSame(before,after); EXPECT_EQ(state.accepted().fixed_dt,.15);
  ASSERT_EQ(state.BeginTrial(&t,&view).status,NS::Ok); AddSpring<<<1,1,0,view.stream>>>(view,0,100,0);
  ASSERT_TRUE(Finish(state,t)); EXPECT_EQ(state.accepted().time,.15);
}

TEST_F(NodalStepCuda, ActualLateCandidateOverflowRollsBackAndRetryMatchesCleanOwner) {
  Initial in; in.n=2; in.x[3]=.75*std::numeric_limits<double>::max(); in.v[3]=.5*std::numeric_limits<double>::max();
  fe::FENodalState state,reference;
  ASSERT_EQ(state.Initialize(Config(in,1),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  ASSERT_EQ(reference.Initialize(Config(in,1),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  Snapshot before,after,expected; ASSERT_TRUE(Read(state,&before));
  auto stage_good=[&](fe::FENodalState& owner,fe::NodalTrialToken* token) {
    fe::NodalAssemblyView view; auto r=owner.BeginTrial(token,&view); if(r.status!=NS::Ok)return r;
    AddForce<<<1,1,0,view.stream>>>(view,0,{1,0,0});
    AddForce<<<1,1,0,view.stream>>>(view,1,{-in.v[3],0,0});
    r=owner.SealAssembly(*token); return r.status==NS::Ok?fe::AdvanceTranslations(owner,*token):r;
  };
  fe::NodalTrialToken good;
  ASSERT_EQ(stage_good(reference,&good).status,NS::Ok); ASSERT_EQ(reference.Commit(good).status,NS::Ok); ASSERT_TRUE(Read(reference,&expected));
  ASSERT_EQ(stage_good(state,&good).status,NS::Ok); // A valid candidate is invalidated by the next attempt.
  ASSERT_TRUE(Read(state,&after)); ExpectSame(before,after);
  fe::NodalTrialToken failed; fe::NodalAssemblyView view; ASSERT_EQ(state.BeginTrial(&failed,&view).status,NS::Ok);
  AddForce<<<1,1,0,view.stream>>>(view,0,{1,0,0}); ASSERT_EQ(state.SealAssembly(failed).status,NS::Ok);
  const auto report=fe::AdvanceTranslations(state,failed); EXPECT_EQ(report.status,NS::InvalidOutput); EXPECT_EQ(report.node,1u);
  EXPECT_EQ(state.Commit(good).status,NS::StaleTrial); EXPECT_EQ(state.Commit(failed).status,NS::WrongPhase);
  ASSERT_TRUE(Read(state,&after)); ExpectSame(before,after);
  ASSERT_EQ(stage_good(state,&good).status,NS::Ok); ASSERT_EQ(state.Commit(good).status,NS::Ok); ASSERT_TRUE(Read(state,&after));
  EXPECT_EQ(after.x,expected.x); EXPECT_EQ(after.v,expected.v); EXPECT_EQ(after.stamp.time,1); EXPECT_EQ(after.stamp.epoch,1u);
}

TEST_F(NodalStepCuda, SnapshotValidationNeverPartiallyPublishesAndTrialIsInvisible) {
  Initial in; in.n=2; in.x[0]=.125; fe::FENodalState state;
  ASSERT_EQ(state.Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  Snapshot before,out; ASSERT_TRUE(Read(state,&before)); out.x.fill(71); out.v.fill(82); out.stamp={9,8,7,6,5}; const auto untouched=out;
  EXPECT_EQ(state.CopyAccepted({out.x.data(),out.v.data(),1},&out.stamp).status,NS::ResourceLimit); ExpectSame(untouched,out);
  EXPECT_EQ(state.CopyAccepted({out.x.data(),out.x.data()+1,2},&out.stamp).status,NS::InvalidInput); ExpectSame(untouched,out);
  EXPECT_EQ(state.CopyAccepted({out.x.data(),out.v.data(),2},reinterpret_cast<fe::NodalStamp*>(out.x.data())).status,NS::InvalidInput); ExpectSame(untouched,out);
  EXPECT_EQ(state.CopyAccepted({out.x.data(),nullptr,2},&out.stamp).status,NS::InvalidInput); ExpectSame(untouched,out);
  auto* overflowing=reinterpret_cast<double*>(UINTPTR_MAX-7);
  EXPECT_EQ(state.CopyAccepted({overflowing,out.v.data(),2},&out.stamp).status,NS::InvalidInput); ExpectSame(untouched,out);
  fe::NodalTrialToken t; fe::NodalAssemblyView view; ASSERT_EQ(state.BeginTrial(&t,&view).status,NS::Ok);
  AddForce<<<1,1,0,view.stream>>>(view,0,{4,0,0}); ASSERT_EQ(state.SealAssembly(t).status,NS::Ok);
  ASSERT_EQ(fe::AdvanceTranslations(state,t).status,NS::Ok); out={}; ASSERT_TRUE(Read(state,&out)); ExpectSame(before,out);
  ASSERT_EQ(state.Commit(t).status,NS::Ok); ASSERT_TRUE(Read(state,&out)); EXPECT_GT(out.x[0],before.x[0]); EXPECT_EQ(out.stamp.epoch,1u);
}

TEST_F(NodalStepCuda, PreparedValidationViewCannotPublishOrReviveRejectedState) {
  Initial in; fe::FENodalState state,other;
  ASSERT_EQ(state.Initialize(Config(in,.1),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  ASSERT_EQ(other.Initialize(Config(in,.1),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  fe::NodalTrialToken token,foreign; fe::NodalAssemblyView assembly;
  fe::NodalPreparedView prepared;
  ASSERT_EQ(state.BeginTrial(&token,&assembly).status,NS::Ok);
  EXPECT_EQ(state.BorrowPrepared(token,&prepared).status,NS::WrongPhase);
  EXPECT_EQ(prepared.kinematics.position_xyz,nullptr);
  ASSERT_EQ(state.BeginTrial(&token,&assembly).status,NS::Ok);
  AddForce<<<1,1,0,assembly.stream>>>(assembly,0,{2,0,0});
  ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
  ASSERT_EQ(fe::AdvanceTranslations(state,token).status,NS::Ok);
  ASSERT_EQ(state.BorrowPrepared(token,&prepared).status,NS::Ok);
  EXPECT_EQ(prepared.owner_id,state.accepted().owner_id);
  EXPECT_EQ(prepared.kinematics.base_epoch,0u);
  EXPECT_DOUBLE_EQ(prepared.proposed_time,.1);
  std::array<double,3> candidate{};
  ASSERT_EQ(cudaMemcpyAsync(candidate.data(),prepared.kinematics.position_xyz,sizeof(candidate),cudaMemcpyDeviceToHost,prepared.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(prepared.stream),cudaSuccess);
  EXPECT_NEAR(candidate[0],.02,1e-16);
  Snapshot accepted; ASSERT_TRUE(Read(state,&accepted)); EXPECT_DOUBLE_EQ(accepted.x[0],0);
  // A case validator rejects the otherwise numerically finite candidate.
  state.Discard(); EXPECT_EQ(state.Commit(token).status,NS::WrongPhase);
  EXPECT_EQ(state.BorrowPrepared(token,&prepared).status,NS::WrongPhase);
  EXPECT_EQ(prepared.kinematics.position_xyz,nullptr);
  ASSERT_TRUE(Read(state,&accepted)); EXPECT_EQ(accepted.stamp.epoch,0u); EXPECT_DOUBLE_EQ(accepted.x[0],0);
  ASSERT_EQ(other.BeginTrial(&foreign,&assembly).status,NS::Ok);
  ASSERT_EQ(state.BeginTrial(&token,&assembly).status,NS::Ok);
  ASSERT_EQ(state.SealAssembly(token).status,NS::Ok); ASSERT_EQ(fe::AdvanceTranslations(state,token).status,NS::Ok);
  EXPECT_EQ(state.BorrowPrepared(foreign,&prepared).status,NS::StaleTrial);
  EXPECT_EQ(state.Commit(token).status,NS::WrongPhase);
  ASSERT_EQ(state.BeginTrial(&token,&assembly).status,NS::Ok);
  ASSERT_EQ(state.SealAssembly(token).status,NS::Ok); ASSERT_EQ(fe::AdvanceTranslations(state,token).status,NS::Ok);
  ASSERT_EQ(state.BorrowPrepared(token,&prepared).status,NS::Ok);
  ASSERT_EQ(state.Commit(token).status,NS::Ok);
  EXPECT_EQ(state.BorrowPrepared(token,&prepared).status,NS::StaleTrial);
  EXPECT_EQ(state.accepted().epoch,1u);
}

TEST_F(NodalStepCuda, ActualLaunchFailurePoisonsOnlyTheOwnerAndPreservesHostMetadata) {
  Initial in; fe::FENodalState state; ASSERT_EQ(state.Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  const auto stamp=state.accepted(); fe::NodalTrialToken t; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&t,&view).status,NS::Ok);
  ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  Noop<<<1,0,0,view.stream>>>(); // Invalid zero-thread launch: no unsafe kernel executes.
  const auto launch_error=cudaPeekAtLastError();
  // CUDA 13.2 reports InvalidValue for this invalid block size. Both this
  // argument-range error and InvalidConfiguration are documented runtime error
  // categories (driver_types.h); Peek documents both and does not clear them.
  // Do not admit unrelated failures merely because they are non-success.
  ASSERT_TRUE(launch_error==cudaErrorInvalidValue || launch_error==cudaErrorInvalidConfiguration)
      << cudaGetErrorString(launch_error);
  RecordProperty("rejected_launch_error",static_cast<int>(launch_error));
  ASSERT_EQ(cudaPeekAtLastError(),launch_error);
  const auto poisoned=state.SealAssembly(t);
  EXPECT_EQ(poisoned.status,NS::DeviceFailure);
  EXPECT_STREQ(poisoned.message,cudaGetErrorString(launch_error));
  EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess); // Seal consumed the pending runtime error.
  EXPECT_EQ(state.accepted().owner_id,stamp.owner_id); EXPECT_EQ(state.accepted().epoch,stamp.epoch); EXPECT_EQ(state.accepted().time,stamp.time);
  state.Discard(); EXPECT_EQ(state.BeginTrial(&t,&view).status,NS::DeviceFailure); EXPECT_EQ(state.Commit(t).status,NS::DeviceFailure);
  Snapshot out; out.x.fill(19); const auto before=out;
  EXPECT_EQ(state.CopyAccepted({out.x.data(),out.v.data(),fe::MaxTranslationNodes},&out.stamp).status,NS::DeviceFailure); ExpectSame(before,out);
  fe::FENodalState healthy; ASSERT_EQ(healthy.Initialize(Config(in),in.view(),in.inverse.data(),in.fixed.data()).status,NS::Ok);
  ASSERT_EQ(healthy.BeginTrial(&t,&view).status,NS::Ok); ASSERT_TRUE(Finish(healthy,t));
}
}  // namespace
