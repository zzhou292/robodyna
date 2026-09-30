// T2: restricted nodal admission, not QEPH material or coupled shell stability.
#include "qualification/nodal/NodalTemporalFixture.h"

namespace {
using namespace tl_test::nodal_temporal;
constexpr std::uint64_t Qualification=0x5432535052494e47ULL;
class NodalHistoryAdmissionCuda:public NodalTemporalCuda {};

fe::NodalStaggeredHistoryAdmission HistoryAdmission(const fe::FENodalState& owner,
                                                   const fe::NodalAssemblyView& view) {
  return {view.owner_id,view.accepted.base_epoch,view.attempt,owner.accepted().fixed_dt,2,Qualification};
}
fe::NodalValidationReceipt Receipt(const fe::NodalStaggeredHistoryAdmission& a) {
  return {a.owner_id,a.base_epoch,a.attempt,a.qualification_id,true};
}
bool PrepareHistory(fe::FENodalState& owner,const fe::NodalTrialToken& token,
                    const fe::NodalStaggeredHistoryAdmission& a,fe::NodalPreparedView& prepared) {
  auto r=owner.SealAssembly(token); EXPECT_EQ(r.status,Code::Ok);
  if(r.status!=Code::Ok) return false;
  r=fe::AdvanceStaggeredHistory(owner,token,a); EXPECT_EQ(r.status,Code::Ok);
  if(r.status!=Code::Ok) return false;
  r=owner.BorrowPrepared(token,&prepared); EXPECT_EQ(r.status,Code::Ok);
  return r.status==Code::Ok;
}
__global__ void AddSpring(fe::NodalAssemblyView view) {
  // Independently qualified scalar oscillator: m=2 kg, k=8 N/m, load=2 N.
  // The force depends on the accepted endpoint, so T1 is inapplicable.
  view.forces.force_x[0]+=2-8*view.accepted.position_xyz[0];
}

TEST_F(NodalHistoryAdmissionCuda, RestrictedSpringMatchesIndependentDiscreteSolution) {
  Initial in; fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  const auto allocation=owner.allocations();
  // Exact leapfrog solution after the physical-rest half kick:
  // x_n=(F/k)(1-cos(n theta)), cos(theta)=1-h^2 k/(2m).
  // h*sqrt(k/m)=1/4 <2 is this scalar recurrence's stability condition.
  const long double angle=std::acos(1.L-2.L*in.h*in.h);
  long double prior_x=0;
  for(unsigned step=1;step<=32;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
    ASSERT_TRUE(BeginLoad(owner,{},token,view));
    AddSpring<<<1,1,0,view.stream>>>(view); ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
    const auto a=HistoryAdmission(owner,view); ASSERT_TRUE(PrepareHistory(owner,token,a,prepared));
    EXPECT_EQ(prepared.kick_dt,step==1?.5*in.h:in.h);
    ASSERT_EQ(fe::CompleteNodalValidation(owner,token,Receipt(a)).status,Code::Ok);
    ASSERT_EQ(owner.Commit(token).status,Code::Ok);
    Snapshot state; ASSERT_TRUE(Read(owner,state));
    const long double x=.25L*(1-std::cos(step*angle));
    EXPECT_NEAR(state.x[0],static_cast<double>(x),ArithmeticTolerance);
    EXPECT_NEAR(state.v[0],static_cast<double>((x-prior_x)/in.h),ArithmeticTolerance);
    EXPECT_EQ(state.stamp.epoch,step); EXPECT_EQ(state.stamp.velocity_phase,Phase::PreviousMidpoint);
    EXPECT_EQ(state.stamp.velocity_time,(step-.5)*in.h);
    prior_x=x;
  }
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,6u);
}

TEST_F(NodalHistoryAdmissionCuda, MissingOrForeignReceiptPreservesAcceptedStateAndFirstKick) {
  Initial in; fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  Snapshot base; ASSERT_TRUE(Read(owner,base));
  for(unsigned bad=0;bad<4;++bad) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
    Loads loads; loads.force[0]=2; ASSERT_TRUE(BeginLoad(owner,loads,token,view));
    const auto a=HistoryAdmission(owner,view); ASSERT_TRUE(PrepareHistory(owner,token,a,prepared));
    EXPECT_EQ(prepared.kick_dt,.5*in.h);
    if(bad==0) EXPECT_EQ(owner.Commit(token).status,Code::MissingCandidateValidation);
    else {
      auto receipt=Receipt(a);
      if(bad==1) receipt.passed=false;
      if(bad==2) ++receipt.qualification_id;
      if(bad==3) ++receipt.attempt;
      EXPECT_EQ(fe::CompleteNodalValidation(owner,token,receipt).status,
                bad==3?Code::StaleTrial:Code::MissingCandidateValidation);
    }
    owner.Discard(); Snapshot held; ASSERT_TRUE(Read(owner,held)); SameState(held,base);
  }
  fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
  Loads loads; loads.force[0]=2; ASSERT_TRUE(BeginLoad(owner,loads,token,view));
  const auto a=HistoryAdmission(owner,view); ASSERT_TRUE(PrepareHistory(owner,token,a,prepared));
  ASSERT_EQ(fe::CompleteNodalValidation(owner,token,Receipt(a)).status,Code::Ok);
  ASSERT_EQ(owner.Commit(token).status,Code::Ok);
  Snapshot after; ASSERT_TRUE(Read(owner,after));
  EXPECT_EQ(after.v[0],.5*in.h); EXPECT_EQ(after.x[0],.5*in.h*in.h);
}

TEST_F(NodalHistoryAdmissionCuda, AdmissionIdentityAndBoundsFailClosedWithoutPublishing) {
  Initial in; fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  Snapshot base; ASSERT_TRUE(Read(owner,base));
  const Code expected[]={Code::MissingStepAdmission,Code::StaleTrial,Code::StaleTrial,
                         Code::StaleTrial,Code::StepTooLarge,Code::InvalidInput,Code::InvalidInput};
  for(unsigned bad=0;bad<7;++bad) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_TRUE(BeginLoad(owner,{},token,view)); ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
    auto a=HistoryAdmission(owner,view);
    switch(bad) {
      case 0:a.qualification_id=0;break;
      case 1:++a.owner_id;break;
      case 2:++a.base_epoch;break;
      case 3:++a.attempt;break;
      case 4:a.maximum_dt=.5*in.h;break;
      case 5:a.maximum_rotation_increment=0;break;
      case 6:a.maximum_dt=std::numeric_limits<double>::quiet_NaN();break;
    }
    EXPECT_EQ(fe::AdvanceStaggeredHistory(owner,token,a).status,expected[bad]);
    owner.Discard(); Snapshot held; ASSERT_TRUE(Read(owner,held)); SameState(held,base);
  }
}

TEST_F(NodalHistoryAdmissionCuda, LegacyStepCannotAcceptHistoryPolicyOrStaggeredOperation) {
  Initial in; fe::FENodalState legacy; ASSERT_EQ(in.Initialize(legacy,Scheme::VelocityFirst).status,Code::Ok);
  for(bool wrong_operation:{false,true}) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_TRUE(BeginLoad(legacy,{},token,view)); ASSERT_EQ(legacy.SealAssembly(token).status,Code::Ok);
    const auto a=HistoryAdmission(legacy,view);
    if(wrong_operation) EXPECT_EQ(fe::AdvanceStaggeredHistory(legacy,token,a).status,Code::UnsupportedTemporalScheme);
    else {
      const fe::NodalStepAdmission invalid{a.owner_id,a.base_epoch,a.attempt,a.maximum_dt,
          a.maximum_rotation_increment,fe::NodalStepAdmissionKind::RestrictedHistoryTrajectory,a.qualification_id,0};
      EXPECT_EQ(fe::AdvanceNodal(legacy,token,invalid).status,Code::UnsupportedTemporalScheme);
    }
    legacy.Discard(); EXPECT_EQ(legacy.accepted().epoch,0u);
  }
}

TEST_F(NodalHistoryAdmissionCuda, ValidatorCudaFailurePreventsPublicationAndPoisonsOwner) {
  Initial in; fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  const auto before=owner.accepted();
  fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
  ASSERT_TRUE(BeginLoad(owner,{},token,view)); const auto a=HistoryAdmission(owner,view);
  ASSERT_TRUE(PrepareHistory(owner,token,a,prepared));
  ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  Noop<<<1,0,0,prepared.stream>>>(); // Safe invalid launch, no bad memory access.
  // Same qualified fault-injection contract as T1: runtime versions may reject
  // the zero block as either an invalid value or invalid launch configuration.
  const auto error=cudaPeekAtLastError();
  ASSERT_TRUE(error==cudaErrorInvalidValue || error==cudaErrorInvalidConfiguration);
  EXPECT_EQ(fe::CompleteNodalValidation(owner,token,Receipt(a)).status,Code::DeviceFailure);
  EXPECT_EQ(owner.Commit(token).status,Code::DeviceFailure); SameStamp(owner.accepted(),before);
  EXPECT_EQ(cudaGetLastError(),cudaSuccess);
}
} // namespace
