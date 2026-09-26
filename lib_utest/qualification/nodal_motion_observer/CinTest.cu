#include "Fixture.h"
#include "lib_utest/qualification/tied_cin_runtime/OwnerFixture.h"
#include <limits>
namespace motion_observer_test {
namespace cin=cin_runtime_test;
TEST_F(Cuda,CompleteCinTailIsValidatedAndAcceptedCoefficientsSurviveRejectedObservation) {
  cin::Fixture fixture;fe::FENodalState owner;
  ASSERT_EQ(cin::Initialize(owner,fixture).status,Code::Ok);
  fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
  const auto n=fixture.mass.size(),rows=fixture.rows.size();
  cin::Snapshot accepted(n,rows),held(n,rows);fe::NodalStamp stamp,held_stamp;
  cin::Accepted(owner,accepted,stamp);
  for(unsigned attempt=0;attempt<2;++attempt) {
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalCinAssemblyView coefficient;
    cin::Fill(owner,fixture,token,assembly,coefficient);
    ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
    ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,cin::Admission(assembly)).status,Code::Ok);
    fe::NodalPreparedView p;ASSERT_EQ(owner.BorrowPrepared(token,&p).status,Code::Ok);
    auto out=Sentinel();const auto unchanged=out;
    if(!attempt) {
      // No rigid groups: genuine CIN tail is4N+2R+1 after the19N slab.
      const auto last=19*n+4*n+2*rows;
      const double nan=std::numeric_limits<double>::quiet_NaN();
      ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(p.kinematics.position_xyz)+last,&nan,
          sizeof(nan),cudaMemcpyHostToDevice,p.stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess);
      EXPECT_EQ(observer.ObservePrepared(owner,token,{},&out).status,Code::InvalidOutput);
      SameOutput(out,unchanged);owner.Discard();
      cin::Accepted(owner,held,held_stamp);cin::Same(accepted,held);
      EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,held_stamp));
    } else {
      Snapshot fields(n);ASSERT_EQ(owner.CopyPrepared(token,fields.Buffer(),&p).status,Code::Ok);
      ASSERT_EQ(observer.ObservePrepared(owner,token,{},&out).status,Code::Ok);
      Same(out.motion,Oracle(fixture.x.data(),fields,{},p.proposed_time));
      cin::Complete(owner,token,assembly);
      ASSERT_EQ(owner.Commit(token).status,Code::Ok);
      EXPECT_EQ(owner.accepted().epoch,1);
    }
  }
}
}
