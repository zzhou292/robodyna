#include "PreparedSnapshotFixture.h"
#include <vector>
namespace prepared_snapshot_test {
namespace {
struct ActiveFields {
  std::size_t n;std::vector<double> x,v,w,q,force,couple;
  explicit ActiveFields(std::size_t count):n(count),x(3*n),v(3*n),w(3*n),q(4*n),force(3*n),couple(3*n) {}
  fe::NodalSnapshotBuffer buffer() {return {x.data(),v.data(),n,q.data(),w.data(),force.data(),couple.data()};}
};
__global__ void ActiveLoads(fe::NodalAssemblyView view) {
  for(std::size_t n=0;n<view.accepted.node_count;++n) {
    view.forces.force_x[n]=1.+n;view.forces.force_y[n]=2.+n;view.forces.force_z[n]=3.+n;
    view.forces.couple_x[n]=.125;view.forces.couple_y[n]=.25;view.forces.couple_z[n]=.5;
  }
}
}
TEST_F(Cuda,PreparedReadbackCoversMaximumOwnerCountIncludingFinalConstrainedReaction) {
  const auto n=fe::MaxNodalStateNodes;ActiveFields initial(n),candidate(n),accepted(n);
  std::vector<double> inverse(n,1),inertia(n,1);std::vector<std::uint8_t> fixed(n),rotation_fixed(n);
  for(std::size_t i=0;i<n;++i) initial.q[4*i]=1;
  fixed.back()=7;rotation_fixed.back()=1;
  fe::FENodalState owner;fe::NodalStateConfig config;config.node_count=n;config.fixed_dt=1./1024;
  config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  ASSERT_EQ(owner.Initialize(config,{initial.x.data(),initial.v.data(),initial.w.data(),n,initial.q.data()},
    inverse.data(),{fixed.data(),rotation_fixed.data(),inertia.data()}).status,Code::Ok);
  const auto allocations=owner.allocations();EXPECT_LE(allocations.device_bytes,fe::MaxTranslationDeviceBytes);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Code::Ok);ActiveLoads<<<1,1,0,assembly.stream>>>(assembly);
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  ASSERT_EQ(fe::AdvanceStaggeredPrescribed(owner,token,nt::Admission(owner,assembly)).status,Code::Ok);
  auto short_output=candidate.buffer();short_output.capacity_nodes=n-1;
  EXPECT_EQ(owner.CopyPrepared(token,short_output,&prepared).status,Code::ResourceLimit);
  ASSERT_EQ(owner.CopyPrepared(token,candidate.buffer(),&prepared).status,Code::Ok);
  EXPECT_EQ(prepared.kinematics.node_count,n);
  const double kick=config.fixed_dt/2;
  for(std::size_t i=0;i<n;++i) for(unsigned axis=0;axis<3;++axis) {
    const auto j=3*i+axis;const double load=1.+i+axis;
    EXPECT_EQ(candidate.v[j],i+1==n?0:kick*load);
    EXPECT_EQ(candidate.x[j],config.fixed_dt*candidate.v[j]);
    EXPECT_EQ(candidate.force[j],i+1==n?-load:0);
    const double moment[]{.125,.25,.5};EXPECT_EQ(candidate.couple[j],i+1==n?-moment[axis]:0);
  }
  ASSERT_EQ(owner.Commit(token).status,Code::Ok);fe::NodalStamp stamp;
  ASSERT_EQ(owner.CopyAccepted(accepted.buffer(),&stamp).status,Code::Ok);
  EXPECT_EQ(candidate.x,accepted.x);EXPECT_EQ(candidate.v,accepted.v);EXPECT_EQ(candidate.w,accepted.w);
  EXPECT_EQ(candidate.q,accepted.q);EXPECT_EQ(candidate.force,accepted.force);EXPECT_EQ(candidate.couple,accepted.couple);
  EXPECT_EQ(owner.allocations().device_bytes,allocations.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocations.device_allocations);
}
} // namespace prepared_snapshot_test
