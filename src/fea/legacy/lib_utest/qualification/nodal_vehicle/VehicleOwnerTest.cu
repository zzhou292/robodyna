#include "VehicleOwnerFixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cstring>
#include <limits>
#include <numeric>

namespace tl::fea::vehicle_test {
TEST_F(VehicleOwnerCuda, FullCapacitySparseLoadsLastNodeRollbackReadbackAndExactRetry) {
  Initial in(MaxActiveNodalStateNodes);FENodalState owner;
  auto c=in.config();ASSERT_EQ(in.Initialize(owner,c).status,NodalStatus::Ok);
  const auto allocation=owner.allocations();EXPECT_EQ(allocation.device_allocations,6u);
  EXPECT_LE(allocation.device_bytes,c.max_device_bytes);
  EXPECT_GT(allocation.device_bytes,MaxTranslationDeviceBytes);
  Initial tiny(1);FENodalState tiny_owner;
  auto tc=tiny.config();tc.max_nodes=MaxNodalStateNodes;tc.max_device_bytes=MaxTranslationDeviceBytes;
  ASSERT_EQ(tiny.Initialize(tiny_owner,tc).status,NodalStatus::Ok);
  // Full capacity reserves 256 row summaries (8192 B); one node reserves one
  // summary (32 B). The per-node state/scratch allocation remains 411 B.
  EXPECT_EQ(allocation.device_bytes-tiny_owner.allocations().device_bytes,411*(in.n-1)+8192-32);
  const auto initial_stamp=owner.accepted();
  std::vector<std::size_t> indices(in.n);std::iota(indices.begin(),indices.end(),0);
  EXPECT_EQ(owner.ValidateNonRigidNodes(indices.data(),indices.size()).status,NodalStatus::Ok);
  indices.back()=in.n;
  EXPECT_EQ(owner.ValidateNonRigidNodes(indices.data(),indices.size()).status,NodalStatus::InvalidInput);

  Fields expected(in.n),actual(in.n);NodalPreparedView prepared;
  NodalTrialToken clean;ASSERT_EQ(Prepare(owner,clean).status,NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPrepared(clean,expected.buffer(),&prepared).status,NodalStatus::Ok);
  Analytic(in,expected,1);owner.Discard();

  NodalTrialToken failed;const auto bad=Prepare(owner,failed,true);
  EXPECT_EQ(bad.status,NodalStatus::InvalidOutput);EXPECT_EQ(bad.node,in.n-1);
  EXPECT_EQ(owner.Commit(failed).status,NodalStatus::WrongPhase);
  NodalStamp stamp;ASSERT_EQ(owner.CopyAccepted(actual.buffer(),&stamp).status,NodalStatus::Ok);
  EXPECT_TRUE(trial_identity::SameStamp(stamp,initial_stamp));InitialFields(in,actual);
  NodalTrialToken retry;ASSERT_EQ(Prepare(owner,retry).status,NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPrepared(retry,actual.buffer(),&prepared).status,NodalStatus::Ok);
  SameFields(expected,actual);owner.Discard();

  // Capacity rejection and a nonfinite LAST quaternion cannot partially publish
  // any output. Staging must include the tail, beyond every legacy node bound.
  NodalTrialToken corrupted;ASSERT_EQ(Prepare(owner,corrupted).status,NodalStatus::Ok);
  NodalPreparedView source;ASSERT_EQ(owner.BorrowPrepared(corrupted,&source).status,NodalStatus::Ok);
  actual.Fill(-91.);const auto before_hash=FieldBitsHash(actual);
  NodalPreparedView unchanged;unchanged.owner_id=777;unchanged.proposed_time=-91.;
  auto short_buffer=actual.buffer();short_buffer.capacity_nodes--;
  EXPECT_EQ(owner.CopyPrepared(corrupted,short_buffer,&unchanged).status,NodalStatus::ResourceLimit);
  EXPECT_EQ(FieldBitsHash(actual),before_hash);EXPECT_EQ(unchanged.owner_id,777u);EXPECT_EQ(unchanged.proposed_time,-91.);
  unsigned char token_bytes[sizeof(corrupted)];std::memcpy(token_bytes,&corrupted,sizeof(corrupted));
  CorruptLastQuaternion(source);
  EXPECT_EQ(owner.CopyPrepared(corrupted,actual.buffer(),&unchanged).status,NodalStatus::InvalidOutput);
  EXPECT_EQ(FieldBitsHash(actual),before_hash);EXPECT_EQ(unchanged.owner_id,777u);EXPECT_EQ(unchanged.proposed_time,-91.);
  EXPECT_EQ(std::memcmp(token_bytes,&corrupted,sizeof(corrupted)),0);
  ASSERT_EQ(owner.CopyAccepted(actual.buffer(),&stamp).status,NodalStatus::Ok);
  EXPECT_TRUE(trial_identity::SameStamp(stamp,initial_stamp));InitialFields(in,actual);

  NodalTrialToken final_retry;ASSERT_EQ(Prepare(owner,final_retry).status,NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPrepared(final_retry,actual.buffer(),&prepared).status,NodalStatus::Ok);
  SameFields(expected,actual);ASSERT_EQ(owner.Commit(final_retry).status,NodalStatus::Ok);
  ASSERT_EQ(owner.CopyAccepted(actual.buffer(),&stamp).status,NodalStatus::Ok);SameFields(expected,actual);
  EXPECT_EQ(stamp.epoch,1u);EXPECT_EQ(stamp.velocity_time,H/2);
  NodalTrialToken later;ASSERT_EQ(Prepare(owner,later).status,NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPrepared(later,expected.buffer(),&prepared).status,NodalStatus::Ok);Analytic(in,expected,2);
  EXPECT_EQ(prepared.kick_dt,H);ASSERT_EQ(owner.Commit(later).status,NodalStatus::Ok);
  ASSERT_EQ(owner.CopyAccepted(actual.buffer(),&stamp).status,NodalStatus::Ok);SameFields(expected,actual);
  EXPECT_EQ(stamp.epoch,2u);EXPECT_EQ(stamp.time,2*H);EXPECT_EQ(stamp.velocity_time,1.5*H);
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
}

TEST_F(VehicleOwnerCuda, ExplicitCountsAndByteBudgetRejectBeforePublicationAndPermitRetry) {
  Initial in(393165);FENodalState owner;auto c=in.config();
  // All original positional aggregate fields retain their meaning; the new
  // trailing maximum defaults to the previously qualified 2048-node scope.
  const NodalStateConfig old{1,MaxTranslationDeviceBytes,H,1e-12,.8,NodalTemporalScheme::VelocityFirst,false};
  EXPECT_EQ(old.max_nodes,MaxNodalStateNodes);
  const auto reject=[&](const NodalStateConfig& config) {
    EXPECT_EQ(in.Initialize(owner,config).status,NodalStatus::ResourceLimit);
    EXPECT_EQ(owner.allocations().device_bytes,0u);EXPECT_EQ(owner.accepted().owner_id,0u);
  };
  auto invalid=c;invalid.max_nodes=MaxNodalStateNodes;reject(invalid);
  invalid=c;invalid.max_nodes=MaxActiveNodalStateNodes+1;reject(invalid);
  invalid=c;invalid.max_nodes=std::numeric_limits<std::size_t>::max();reject(invalid);
  invalid=c;invalid.node_count=MaxActiveNodalStateNodes+1;reject(invalid);
  invalid=c;invalid.max_device_bytes=MaxActiveNodalStateDeviceBytes+1;reject(invalid);
  invalid=c;invalid.max_device_bytes=MaxTranslationDeviceBytes;reject(invalid);
  // Even the known nodal payload alone exceeds this cap, without control bytes.
  invalid=c;invalid.max_device_bytes=411*in.n-1;reject(invalid);
  const auto last=in.q.back();in.q.back()=std::numeric_limits<double>::quiet_NaN();
  const auto bad=in.Initialize(owner,c);EXPECT_EQ(bad.status,NodalStatus::InvalidInput);EXPECT_EQ(bad.node,in.n-1);
  EXPECT_EQ(owner.allocations().device_bytes,0u);EXPECT_EQ(owner.accepted().owner_id,0u);
  in.q.back()=last;ASSERT_EQ(in.Initialize(owner,c).status,NodalStatus::Ok);
  EXPECT_EQ(owner.accepted().node_count,in.n);EXPECT_EQ(owner.allocations().device_allocations,6u);
  EXPECT_EQ(owner.accepted().epoch,0u);
}
} // namespace tl::fea::vehicle_test
