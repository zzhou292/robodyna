#include "VehicleOwnerFixture.h"
#include <array>
#include <cmath>
#include <limits>

namespace tl::fea::vehicle_test {
namespace {
// Frozen node/axis order from FENodalState.cu at 01a9a39. The test drives the
// real owner; it does not invoke or share the parallel validation helper.
NodalReport Serial(const std::vector<double>& values, std::size_t n, bool rotations) {
  for (std::uint32_t node=0; node<n; ++node)
    for (unsigned axis=0; axis<6; ++axis) {
      const auto value=values[axis*n+node];
      if (!std::isfinite(value)) return {NodalStatus::InvalidOutput,"serial",node};
      if (!rotations && axis>=3 && value!=0)
        return {NodalStatus::UnsupportedRotation,"serial",node};
    }
  return {NodalStatus::Ok,"serial"};
}
__global__ void PriorFailure(NodalAssemblyView view, bool stale, bool contributor) {
  if (stale) ++view.result->base_epoch;
  if (contributor) {
    view.result->status=tlfea::contact::Status::kInvalidArgument;
    view.result->node=491;
  }
}
void Upload(const NodalAssemblyView& view, const std::vector<double>& values) {
  const auto n=view.accepted.node_count;
  const std::array<double*,6> arrays{view.forces.force_x,view.forces.force_y,view.forces.force_z,
      view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  for (unsigned axis=0; axis<6; ++axis)
    ASSERT_EQ(cudaMemcpyAsync(arrays[axis],values.data()+axis*n,n*sizeof(double),
                            cudaMemcpyHostToDevice,view.stream),cudaSuccess);
}
}
TEST_F(VehicleOwnerCuda, ParallelSealMatchesFrozenNodeAxisErrorsAndRetry) {
  constexpr std::size_t n=513;
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  const auto inf=std::numeric_limits<double>::infinity();
  Initial in(n);
  for (const bool rotations:{false,true}) {
    SCOPED_TRACE(rotations);
    FENodalState owner;
    auto config=in.config();
    std::vector<std::uint8_t> fixed(n); fixed[n-2]=1;
    if (rotations) ASSERT_EQ(in.Initialize(owner,config).status,NodalStatus::Ok);
    else {
      config.temporal_scheme=NodalTemporalScheme::VelocityFirst;
      ASSERT_EQ(owner.Initialize(config,{in.x.data(),in.v.data(),nullptr,n},
                                 in.inverse.data(),fixed.data()).status,NodalStatus::Ok);
    }
    const auto allocation=owner.allocations();
    std::vector<std::vector<double>> cases(7,std::vector<double>(6*n));
    cases[0][5*n+512]=-0.;
    cases[1][2*n+512]=inf;
    cases[2][5*n+257]=nan; cases[2][3*n+3]=1;
    cases[3][3*n+260]=1; cases[3][4*n+260]=nan;
    cases[4][3*n+260]=nan; cases[4][4*n+260]=1;
    cases[5][5*n+512]=nan; cases[5][1]=inf; cases[5][n]=nan;
    cases[6][5*n+512]=1;
    for (std::size_t index=0; index<cases.size(); ++index) {
      SCOPED_TRACE(index);
      NodalTrialToken token; NodalAssemblyView view;
      ASSERT_EQ(owner.BeginTrial(&token,&view).status,NodalStatus::Ok);
      Upload(view,cases[index]);
      const auto actual=owner.SealAssembly(token), expected=Serial(cases[index],n,rotations);
      EXPECT_EQ(actual.status,expected.status);
      if (expected.status!=NodalStatus::Ok) EXPECT_EQ(actual.node,expected.node);
      EXPECT_EQ(owner.accepted().epoch,0u);
      owner.Discard();
      NodalTrialToken retry;
      ASSERT_EQ(owner.BeginTrial(&retry,&view).status,NodalStatus::Ok);
      ASSERT_EQ(owner.SealAssembly(retry).status,NodalStatus::Ok);
      owner.Discard();
    }
    EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
    Fields snapshot(n); NodalStamp stamp;
    const auto buffer=rotations?snapshot.buffer():NodalSnapshotBuffer{snapshot.x.data(),snapshot.v.data(),n};
    ASSERT_EQ(owner.CopyAccepted(buffer,&stamp).status,NodalStatus::Ok);
    EXPECT_EQ(snapshot.x,in.x); EXPECT_EQ(snapshot.v,in.v);
    if (rotations) { EXPECT_EQ(snapshot.q,in.q); EXPECT_EQ(snapshot.w,in.w); }
  }
}
TEST_F(VehicleOwnerCuda, SealRetainsIdentityAndContributorPrecedenceBeforeScratchErrors) {
  Initial in(513); FENodalState owner;
  ASSERT_EQ(in.Initialize(owner,in.config()).status,NodalStatus::Ok);
  std::vector<double> values(6*in.n); values[0]=std::numeric_limits<double>::quiet_NaN();
  for (const bool stale:{false,true}) {
    NodalTrialToken token; NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,NodalStatus::Ok);
    Upload(view,values); PriorFailure<<<1,1,0,view.stream>>>(view,stale,true);
    const auto report=owner.SealAssembly(token);
    EXPECT_EQ(report.status,stale?NodalStatus::StaleTrial:NodalStatus::ContributorFailure);
    EXPECT_EQ(report.node,stale?UINT32_MAX:491u);
    EXPECT_EQ(owner.accepted().epoch,0u);
    owner.Discard();
  }
  NodalTrialToken retry; ASSERT_EQ(Prepare(owner,retry).status,NodalStatus::Ok);
  ASSERT_EQ(owner.Commit(retry).status,NodalStatus::Ok);
  EXPECT_EQ(owner.accepted().epoch,1u);
}
} // namespace tl::fea::vehicle_test
