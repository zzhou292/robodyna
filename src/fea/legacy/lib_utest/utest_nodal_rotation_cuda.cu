// Independent isotropic-node dynamics and transaction checks. These prescribed
// loads do not qualify a nonlinear shell/contact timestep or a deforming coupon.
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/ExplicitTranslationStep.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
namespace fe = tl::fea;
using NS = fe::NodalStatus;
constexpr std::size_t Capacity = fe::MaxTranslationNodes;
struct Initial {
  std::size_t n = 1;
  double h = .01;
  std::array<double,3*Capacity> x{}, v{}, omega{};
  std::array<double,4*Capacity> q{};
  std::array<double,Capacity> inverse{}, inverse_inertia{};
  std::array<std::uint8_t,Capacity> translation_fixed{}, rotation_fixed{};
  Initial() {
    inverse.fill(1); inverse_inertia.fill(.5);
    for (std::size_t i=0;i<Capacity;++i) q[4*i]=1;
  }
  fe::HostNodalKinematicsView view() const { return {x.data(),v.data(),omega.data(),n,q.data()}; }
  fe::NodalDofConfig dofs() const { return {translation_fixed.data(),rotation_fixed.data(),inverse_inertia.data()}; }
  fe::NodalReport Initialize(fe::FENodalState& state) const {
    fe::NodalStateConfig c; c.node_count=n; c.fixed_dt=h;
    return state.Initialize(c,view(),inverse.data(),dofs());
  }
};
struct Snapshot {
  std::array<double,3*Capacity> x{}, v{}, omega{}, reaction{}, couple{};
  std::array<double,4*Capacity> q{};
  fe::NodalStamp stamp;
  fe::NodalSnapshotBuffer buffer() {
    return {x.data(),v.data(),Capacity,q.data(),omega.data(),reaction.data(),couple.data()};
  }
};
bool Read(fe::FENodalState& state, Snapshot& out) {
  auto report=state.CopyAccepted(out.buffer(),&out.stamp);
  EXPECT_EQ(report.status,NS::Ok) << report.message; return report.status==NS::Ok;
}
void ExpectSame(const Snapshot& a,const Snapshot& b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.q,b.q); EXPECT_EQ(a.omega,b.omega);
  EXPECT_EQ(a.reaction,b.reaction); EXPECT_EQ(a.couple,b.couple);
  EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id); EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);
  EXPECT_EQ(a.stamp.time,b.stamp.time); EXPECT_EQ(a.stamp.reactions_valid,b.stamp.reactions_valid);
  EXPECT_EQ(a.stamp.reaction_base_epoch,b.stamp.reaction_base_epoch);
  EXPECT_EQ(a.stamp.reaction_time,b.stamp.reaction_time);
}
fe::NodalStepAdmission Admission(const fe::FENodalState& state,const fe::NodalAssemblyView& v) {
  return {v.owner_id,v.accepted.base_epoch,v.attempt,state.accepted().fixed_dt,2,
          fe::NodalStepAdmissionKind::PrescribedConstantLoads};
}
bool Finish(fe::FENodalState& state,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& v) {
  auto r=state.SealAssembly(token); EXPECT_EQ(r.status,NS::Ok) << r.message;
  if(r.status!=NS::Ok) return false;
  r=fe::AdvanceNodal(state,token,Admission(state,v)); EXPECT_EQ(r.status,NS::Ok) << r.message;
  if(r.status!=NS::Ok) return false;
  r=state.Commit(token); EXPECT_EQ(r.status,NS::Ok) << r.message; return r.status==NS::Ok;
}
__global__ void AddLoad(fe::NodalAssemblyView v,unsigned n,double fx,double fy,double fz,
                        double cx,double cy,double cz) {
  v.forces.force_x[n]+=fx; v.forces.force_y[n]+=fy; v.forces.force_z[n]+=fz;
  v.forces.couple_x[n]+=cx; v.forces.couple_y[n]+=cy; v.forces.couple_z[n]+=cz;
}
__global__ void NonemptyBounds(fe::NodalAssemblyView v) { v.bounds->stiffness[0]=1; }
__global__ void Noop() {}
__global__ void ContributorFailure(fe::NodalAssemblyView v) {
  fe::RecordNodalAssemblyFailure(v,tlfea::contact::Status::kOutOfRange,1);
}
class NodalRotationCuda : public ::testing::Test {
  void SetUp() override { int n=0; ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess); ASSERT_GT(n,0); }
};

TEST_F(NodalRotationCuda, InvalidDofsQuaternionsAndMassFailBeforePublication) {
  auto reject=[](const Initial& in) {
    fe::FENodalState owner; EXPECT_NE(in.Initialize(owner).status,NS::Ok);
    EXPECT_EQ(owner.accepted().owner_id,0u); EXPECT_EQ(owner.allocations().device_bytes,0u);
  };
  Initial in; in.n=2;
  for(double bad:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    auto x=in; x.inverse_inertia[1]=bad; reject(x);
    x=in; x.inverse[1]=bad; reject(x);
  }
  for(double bad:{0.,1.001,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    auto x=in; x.q[4]=bad; reject(x);
  }
  auto x=in; x.translation_fixed[1]=8; reject(x);
  x=in; x.rotation_fixed[1]=2; reject(x);
  x=in; x.translation_fixed[1]=7; reject(x); // Fixed inverse mass must be zero.
  x=in; x.rotation_fixed[1]=1; reject(x); // Fixed inverse inertia must be zero.
  x=in; x.translation_fixed[1]=2; x.v[4]=1; reject(x);
  x=in; x.rotation_fixed[1]=1; x.inverse_inertia[1]=0; x.omega[5]=1; reject(x);
  fe::NodalStateConfig c; c.node_count=2;
  auto view=in.view(); view.orientation_wxyz=nullptr;
  fe::FENodalState owner; EXPECT_NE(owner.Initialize(c,view,in.inverse.data(),in.dofs()).status,NS::Ok);
  EXPECT_EQ(owner.allocations().device_bytes,0u);
}

TEST_F(NodalRotationCuda, LegacyContributorCapacitySpinMatchesExactWorldComposition) {
  Initial in; in.n=Capacity; const double r=std::sqrt(.5);
  for(std::size_t i=0;i<in.n;++i) {
    const double sign=i%2?-1:1;
    in.q[4*i]=sign*r; in.q[4*i+1]=sign*r; // Noncommuting initial x rotation.
    in.omega[3*i]=1; in.omega[3*i+1]=2; in.omega[3*i+2]=-2;
    in.v[3*i]=.125;
  }
  fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  EXPECT_TRUE(state.accepted().has_rotations);
  const auto allocation=state.allocations(); EXPECT_EQ(allocation.device_allocations,6u);
  const double* slabs[2]{};
  for(int step=0;step<100;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
    if(step<2) slabs[step]=view.accepted.orientation_wxyz;
    else EXPECT_EQ(view.accepted.orientation_wxyz,slabs[step%2]);
    ASSERT_TRUE(Finish(state,token,view));
  }
  Snapshot out; ASSERT_TRUE(Read(state,out)); EXPECT_NE(slabs[0],slabs[1]);
  const double c=std::cos(1.5),s=std::sin(1.5)/3;
  const double expected[4]={r*(c-s),r*(c+s),0,-4*r*s};
  for(std::size_t i=0;i<in.n;++i) {
    double norm=0;
    for(unsigned a=0;a<4;++a) {
      EXPECT_NEAR(out.q[4*i+a],(i%2?-1:1)*expected[a],8e-14);
      norm+=out.q[4*i+a]*out.q[4*i+a];
    }
    EXPECT_NEAR(norm,1,8e-16); EXPECT_NEAR(out.x[3*i],.125,2e-15);
    for(unsigned a=0;a<3;++a) EXPECT_EQ(out.omega[3*i+a],in.omega[3*i+a]);
  }
  EXPECT_EQ(out.stamp.epoch,100u); EXPECT_NEAR(out.stamp.time,1,2e-15);
  EXPECT_EQ(state.allocations().device_bytes,allocation.device_bytes);
}

TEST_F(NodalRotationCuda, ConstantTorqueConvergesAndBalancesAngularImpulseAndWork) {
  double previous=1;
  for(unsigned level=0;level<3;++level) {
    Initial in; in.h=.025/(1u<<level); in.inverse_inertia[0]=.25;
    fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
    const unsigned steps=20*(1u<<level);
    double work=0,previous_omega=0;
    for(unsigned step=0;step<steps;++step) {
      fe::NodalTrialToken token; fe::NodalAssemblyView view;
      ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
      AddLoad<<<1,1,0,view.stream>>>(view,0,0,0,0,0,0,4);
      ASSERT_TRUE(Finish(state,token,view));
      Snapshot accepted; ASSERT_TRUE(Read(state,accepted));
      work+=in.h*4*.5*(previous_omega+accepted.omega[2]);
      previous_omega=accepted.omega[2];
    }
    Snapshot out; ASSERT_TRUE(Read(state,out));
    const double time=.5,angle=2*std::atan2(out.q[3],out.q[0]);
    EXPECT_NEAR(4*out.omega[2],4*time,2e-14); // J omega = torque * time.
    EXPECT_NEAR(.5*4*out.omega[2]*out.omega[2],4*.5*time*time,2e-14);
    EXPECT_NEAR(work,.5*4*out.omega[2]*out.omega[2],2e-14);
    // Velocity-first orientation uses endpoint omega, so torque times the
    // accumulated rotation angle differs from midpoint velocity work by O(h).
    EXPECT_NEAR(4*angle-work,2*time*in.h,4e-14);
    const double error=std::fabs(angle-.5*time*time);
    EXPECT_NEAR(error,.5*time*in.h,2e-14); EXPECT_LT(error,.6*previous); previous=error;
  }
}

TEST_F(NodalRotationCuda, TinyIncrementAndQuaternionSignPreserveUnitState) {
  for(double angle:{0.,1e-18,1e-15,1e-12}) {
    Initial in; in.h=1; in.q[0]=-1; in.omega[0]=angle;
    fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_TRUE(Finish(state,token,view));
    Snapshot out; ASSERT_TRUE(Read(state,out));
    EXPECT_NEAR(out.q[0],-std::cos(angle/2),1e-16);
    EXPECT_NEAR(out.q[1],-std::sin(angle/2),1e-28);
  }
}

TEST_F(NodalRotationCuda, WorldComponentConstraintsHaveBaseStateReactionDiagnostics) {
  Initial in; in.n=2; in.h=.1; in.translation_fixed[0]=6; in.inverse[0]=.5;
  in.translation_fixed[1]=7; in.inverse[1]=0; in.rotation_fixed[1]=1; in.inverse_inertia[1]=0;
  in.x[1]=3; in.x[2]=5; in.x[3]=7;
  fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  Snapshot out; ASSERT_TRUE(Read(state,out)); EXPECT_FALSE(out.stamp.reactions_valid);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
  EXPECT_NE(view.translation_fixed_bits,nullptr); EXPECT_NE(view.inverse_inertia,nullptr);
  EXPECT_EQ(view.mass.model,tlfea::contact::TranslationMassModel::kUnspecified);
  for(unsigned i=0;i<2;++i) AddLoad<<<1,1,0,view.stream>>>(view,i,2,3,4,5,6,7);
  ASSERT_TRUE(Finish(state,token,view)); ASSERT_TRUE(Read(state,out));
  EXPECT_NEAR(out.x[0],.01,2e-17); EXPECT_NEAR(out.v[0],.1,2e-16);
  for(unsigned a=1;a<6;++a) { EXPECT_EQ(out.x[a],in.x[a]); EXPECT_EQ(out.v[a],0); }
  EXPECT_EQ(out.reaction[0],0); EXPECT_EQ(out.reaction[1],-3); EXPECT_EQ(out.reaction[2],-4);
  for(unsigned a=0;a<3;++a) {
    EXPECT_EQ(out.reaction[3+a],-double(2+a)); EXPECT_EQ(out.couple[a],0);
    EXPECT_EQ(out.couple[3+a],-double(5+a)); EXPECT_EQ(out.omega[3+a],0);
    EXPECT_NEAR(out.omega[a],.05*(5+a),1e-15);
  }
  EXPECT_TRUE(out.stamp.reactions_valid); EXPECT_EQ(out.stamp.reaction_base_epoch,0u);
  EXPECT_EQ(out.stamp.reaction_time,0); EXPECT_EQ(out.stamp.time,.1);
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_TRUE(Finish(state,token,view));
  ASSERT_TRUE(Read(state,out)); EXPECT_EQ(out.stamp.reaction_base_epoch,1u); EXPECT_EQ(out.stamp.reaction_time,.1);
  for(unsigned a=0;a<6;++a) { EXPECT_EQ(out.reaction[a],0); EXPECT_EQ(out.couple[a],0); }
}

TEST_F(NodalRotationCuda, TranslationOperationCannotBypassCombinedAdmission) {
  Initial in; fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  Snapshot before,after; ASSERT_TRUE(Read(state,before));
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
  EXPECT_NE(fe::AdvanceTranslations(state,token).status,NS::Ok);
  EXPECT_NE(state.Commit(token).status,NS::Ok); ASSERT_TRUE(Read(state,after)); ExpectSame(before,after);
}

TEST_F(NodalRotationCuda, MissingStaleAndInvalidAdmissionsCannotCommit) {
  Initial in; fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  Snapshot before,after; ASSERT_TRUE(Read(state,before));
  for(unsigned variant=0;variant<9;++variant) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
    auto a=Admission(state,view);
    switch(variant) {
      case 0:a={};break; case 1:++a.owner_id;break; case 2:++a.base_epoch;break; case 3:++a.attempt;break;
      case 4:a.maximum_dt=.5*in.h;break; case 5:a.maximum_rotation_increment=0;break;
      case 6:a.maximum_rotation_increment=4;break;
      case 7:a.maximum_dt=std::numeric_limits<double>::quiet_NaN();break;
      case 8:a.kind=static_cast<fe::NodalStepAdmissionKind>(77);break;
    }
    EXPECT_NE(fe::AdvanceNodal(state,token,a).status,NS::Ok); EXPECT_NE(state.Commit(token).status,NS::Ok);
    ASSERT_TRUE(Read(state,after)); ExpectSame(before,after);
  }
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_TRUE(Finish(state,token,view));
}

TEST_F(NodalRotationCuda, ConstantLoadPolicyRejectsNonemptyStructuralBounds) {
  Initial in; fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); NonemptyBounds<<<1,1,0,view.stream>>>(view);
  ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
  EXPECT_NE(fe::AdvanceNodal(state,token,Admission(state,view)).status,NS::Ok);
  EXPECT_NE(state.Commit(token).status,NS::Ok); EXPECT_EQ(state.accepted().epoch,0u);
}

TEST_F(NodalRotationCuda, LateRotationalFailurePreservesAllAcceptedFieldsAndCleanRetry) {
  Initial in; in.n=3; fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
  AddLoad<<<1,1,0,view.stream>>>(view,0,1,0,0,0,0,1); ASSERT_TRUE(Finish(state,token,view));
  Snapshot before,after; ASSERT_TRUE(Read(state,before));
  for(double torque:{1e6,1e308}) {
    ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
    AddLoad<<<1,1,0,view.stream>>>(view,0,1,2,3,1,2,3);
    AddLoad<<<1,1,0,view.stream>>>(view,2,0,0,0,0,0,torque);
    ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
    const auto failed=fe::AdvanceNodal(state,token,Admission(state,view));
    EXPECT_NE(failed.status,NS::Ok);
    if(failed.status==NS::StepTooLarge) EXPECT_EQ(failed.stable_dt,0);
    EXPECT_NE(state.Commit(token).status,NS::Ok); ASSERT_TRUE(Read(state,after)); ExpectSame(before,after);
  }
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_TRUE(Finish(state,token,view));
  ASSERT_TRUE(Read(state,after)); EXPECT_EQ(after.stamp.epoch,2u); EXPECT_EQ(after.omega,before.omega);
  fe::FENodalState clean; ASSERT_EQ(in.Initialize(clean).status,NS::Ok);
  ASSERT_EQ(clean.BeginTrial(&token,&view).status,NS::Ok);
  AddLoad<<<1,1,0,view.stream>>>(view,0,1,0,0,0,0,1); ASSERT_TRUE(Finish(clean,token,view));
  ASSERT_EQ(clean.BeginTrial(&token,&view).status,NS::Ok); ASSERT_TRUE(Finish(clean,token,view));
  Snapshot expected; ASSERT_TRUE(Read(clean,expected));
  expected.stamp.owner_id=after.stamp.owner_id; ExpectSame(expected,after);
}

TEST_F(NodalRotationCuda, EachWorldTranslationAxisConstraintPreservesOnlyItsComponent) {
  for(unsigned fixed_axis=0;fixed_axis<3;++fixed_axis) {
    Initial in; in.translation_fixed[0]=1u<<fixed_axis;
    fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok);
    AddLoad<<<1,1,0,view.stream>>>(view,0,1,2,3,0,0,0); ASSERT_TRUE(Finish(state,token,view));
    Snapshot out; ASSERT_TRUE(Read(state,out));
    for(unsigned a=0;a<3;++a) {
      EXPECT_NEAR(out.v[a],a==fixed_axis?0:in.h*(a+1),1e-16);
      EXPECT_NEAR(out.x[a],a==fixed_axis?0:in.h*in.h*(a+1),1e-18);
      EXPECT_EQ(out.reaction[a],a==fixed_axis?-double(a+1):0);
    }
  }
}

TEST_F(NodalRotationCuda, NoncoaxialTorqueAndSpinAreCovariantUnderCommonWorldRotation) {
  // Rotate all world data by +90 degrees about z. The exact component map and
  // left quaternion product are written independently of production helpers.
  const double r=std::sqrt(.5);
  Initial base,rotated;
  base.omega[0]=.3; base.omega[1]=-.2; base.omega[2]=.1;
  rotated.omega[0]=.2; rotated.omega[1]=.3; rotated.omega[2]=.1;
  rotated.q[0]=r; rotated.q[3]=r;
  fe::FENodalState a,b; ASSERT_EQ(base.Initialize(a).status,NS::Ok); ASSERT_EQ(rotated.Initialize(b).status,NS::Ok);
  for(unsigned step=0;step<20;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(a.BeginTrial(&token,&view).status,NS::Ok);
    AddLoad<<<1,1,0,view.stream>>>(view,0,0,0,0,.2,.4,-.6); ASSERT_TRUE(Finish(a,token,view));
    ASSERT_EQ(b.BeginTrial(&token,&view).status,NS::Ok);
    AddLoad<<<1,1,0,view.stream>>>(view,0,0,0,0,-.4,.2,-.6); ASSERT_TRUE(Finish(b,token,view));
  }
  Snapshot x,y; ASSERT_TRUE(Read(a,x)); ASSERT_TRUE(Read(b,y));
  EXPECT_NEAR(y.omega[0],-x.omega[1],1e-15); EXPECT_NEAR(y.omega[1],x.omega[0],1e-15);
  EXPECT_NEAR(y.omega[2],x.omega[2],1e-15);
  const double q[4]={r*(x.q[0]-x.q[3]),r*(x.q[1]-x.q[2]),r*(x.q[1]+x.q[2]),r*(x.q[0]+x.q[3])};
  for(unsigned i=0;i<4;++i) EXPECT_NEAR(y.q[i],q[i],3e-15);
}

TEST_F(NodalRotationCuda, ContributorFailureAndPreparedDiscardPreserveAcceptedRotations) {
  Initial in; in.n=2; in.omega[2]=1; fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  Snapshot before,after; ASSERT_TRUE(Read(state,before));
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ContributorFailure<<<1,1,0,view.stream>>>(view);
  EXPECT_EQ(state.SealAssembly(token).status,NS::ContributorFailure);
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
  ASSERT_EQ(fe::AdvanceNodal(state,token,Admission(state,view)).status,NS::Ok);
  fe::NodalPreparedView prepared; ASSERT_EQ(state.BorrowPrepared(token,&prepared).status,NS::Ok);
  EXPECT_NE(prepared.kinematics.orientation_wxyz,nullptr); EXPECT_NE(prepared.kinematics.angular_velocity_xyz,nullptr);
  std::array<double,8> trial{};
  ASSERT_EQ(cudaMemcpyAsync(trial.data(),prepared.kinematics.orientation_wxyz,sizeof(trial),cudaMemcpyDeviceToHost,prepared.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(prepared.stream),cudaSuccess); EXPECT_GT(trial[3],0);
  ASSERT_TRUE(Read(state,after)); ExpectSame(before,after);
  state.Discard(); EXPECT_NE(state.Commit(token).status,NS::Ok); ASSERT_TRUE(Read(state,after)); ExpectSame(before,after);
}

TEST_F(NodalRotationCuda, ExtendedReadbackRejectsAliasingBeforePublishingAnyOutput) {
  Initial in; fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  Snapshot out; out.x.fill(91); out.v.fill(92); out.q.fill(93); out.omega.fill(94); out.reaction.fill(95); out.couple.fill(96);
  const auto before=out;
  for(unsigned variant=0;variant<5;++variant) {
    auto b=out.buffer();
    if(variant==0) b.orientation_wxyz=b.position_xyz;
    if(variant==1) b.reaction_force_xyz=b.orientation_wxyz+1;
    if(variant==2) b.angular_velocity_xyz=b.reaction_couple_xyz;
    if(variant==3) b.capacity_nodes=0;
    if(variant==4) b.reaction_couple_xyz=reinterpret_cast<double*>(&out.stamp);
    EXPECT_NE(state.CopyAccepted(b,&out.stamp).status,NS::Ok); ExpectSame(before,out);
  }
  // Existing output adapters can request only x/v; extended arrays remain private.
  ASSERT_EQ(state.CopyAccepted({out.x.data(),out.v.data(),Capacity},&out.stamp).status,NS::Ok);
  EXPECT_EQ(out.q,before.q); EXPECT_EQ(out.omega,before.omega); EXPECT_TRUE(out.stamp.has_rotations);
}

TEST_F(NodalRotationCuda, ValidatorLaunchFailurePoisonsBeforeCommitPublication) {
  Initial in; in.omega[2]=1;
  fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  const auto before=state.accepted();
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
  ASSERT_EQ(fe::AdvanceNodal(state,token,Admission(state,view)).status,NS::Ok);
  fe::NodalPreparedView prepared; ASSERT_EQ(state.BorrowPrepared(token,&prepared).status,NS::Ok);
  // Invalid launch configuration is recoverable at CUDA-context level; it
  // exercises the module poison contract without an illegal device access.
  Noop<<<1,0,0,prepared.stream>>>();
  const auto launch_error=cudaPeekAtLastError();
  EXPECT_TRUE(launch_error==cudaErrorInvalidConfiguration || launch_error==cudaErrorInvalidValue);
  EXPECT_EQ(state.Commit(token).status,NS::DeviceFailure);
  EXPECT_EQ(state.accepted().epoch,before.epoch); EXPECT_EQ(state.accepted().time,before.time);
  EXPECT_FALSE(state.accepted().reactions_valid);
  Snapshot out; EXPECT_EQ(state.CopyAccepted(out.buffer(),&out.stamp).status,NS::DeviceFailure);
  EXPECT_EQ(cudaGetLastError(),cudaSuccess); // Commit consumed the launch error.
}

TEST_F(NodalRotationCuda, RestrictedElasticAdvanceCannotCommitBeforeCandidateReceipt) {
  Initial in; in.h=.001; in.omega[2]=1;
  fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  for(unsigned attempt=0;attempt<2;++attempt) {
    ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
    auto admission=Admission(state,view); admission.kind=fe::NodalStepAdmissionKind::RestrictedElasticTrajectory;
    admission.qualification_id=17; admission.stiffness_rate_envelope=100;
    ASSERT_EQ(fe::AdvanceNodal(state,token,admission).status,NS::Ok);
    fe::NodalPreparedView prepared; ASSERT_EQ(state.BorrowPrepared(token,&prepared).status,NS::Ok);
    EXPECT_EQ(prepared.base_kinematics.base_epoch,0u);
    EXPECT_NE(prepared.base_kinematics.orientation_wxyz,prepared.kinematics.orientation_wxyz);
    if(attempt==0) {
      EXPECT_EQ(state.Commit(token).status,NS::MissingCandidateValidation); EXPECT_EQ(state.accepted().epoch,0u);
    } else {
      ASSERT_EQ(fe::CompleteNodalValidation(state,token,{view.owner_id,0,view.attempt,17,true}).status,NS::Ok);
      ASSERT_EQ(state.Commit(token).status,NS::Ok); EXPECT_EQ(state.accepted().epoch,1u);
    }
  }
}

TEST_F(NodalRotationCuda, RestrictedPolicyRejectsBadEnvelopesAndStaleReceipts) {
  Initial in; in.h=.001; fe::FENodalState state; ASSERT_EQ(in.Initialize(state).status,NS::Ok);
  Snapshot before,after; ASSERT_TRUE(Read(state,before));
  for(unsigned variant=0;variant<9;++variant) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(state.BeginTrial(&token,&view).status,NS::Ok); ASSERT_EQ(state.SealAssembly(token).status,NS::Ok);
    auto a=Admission(state,view); a.kind=fe::NodalStepAdmissionKind::RestrictedElasticTrajectory;
    a.qualification_id=17; a.stiffness_rate_envelope=100;
    if(variant<4) {
      if(variant==0) a.qualification_id=0;
      if(variant==1) a.stiffness_rate_envelope=0;
      if(variant==2) a.stiffness_rate_envelope=std::numeric_limits<double>::quiet_NaN();
      if(variant==3) a.stiffness_rate_envelope=1e8;
      EXPECT_NE(fe::AdvanceNodal(state,token,a).status,NS::Ok);
    } else {
      ASSERT_EQ(fe::AdvanceNodal(state,token,a).status,NS::Ok);
      fe::NodalValidationReceipt receipt{view.owner_id,0,view.attempt,17,true};
      if(variant==4) ++receipt.owner_id;
      if(variant==5) ++receipt.base_epoch;
      if(variant==6) ++receipt.attempt;
      if(variant==7) ++receipt.qualification_id;
      if(variant==8) receipt.passed=false;
      EXPECT_NE(fe::CompleteNodalValidation(state,token,receipt).status,NS::Ok);
    }
    EXPECT_NE(state.Commit(token).status,NS::Ok); ASSERT_TRUE(Read(state,after)); ExpectSame(before,after);
  }
}
}  // namespace
