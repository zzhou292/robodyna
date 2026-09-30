#include "VehicleCudaFixture.h"
#include "lib_utest/qualification/nodal_rigid_group/PreparedSnapshotCudaProbe.h"

namespace type25_batch_test {
using Type25VehicleCopyCuda=vehicle::VehicleOwnerCuda;
TEST_F(Type25VehicleCopyCuda,CompletedPrivateCopyFailurePreservesPublicOutputsAndOwnerState) {
  VehicleRig rig(2052,1025);ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;fe::NodalPreparedView prepared;ASSERT_TRUE(rig.Prepare(token,prepared));
  spring::BatchDiagnostics candidate;ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate).status,spring::BatchStatus::Success);
  std::vector<spring::Evaluation> out(1025);for(auto& e:out)e.critical_dt_s=717;
  const auto before=out;const auto accepted=rig.owner.accepted();const auto identity=candidate;
  prepared_snapshot_probe::FailAfterNextDeviceRead();
  EXPECT_EQ(rig.batch.CopyPreparedResults(candidate,out.data(),out.size()).status,spring::BatchStatus::DeviceFailure);
  EXPECT_TRUE(prepared_snapshot_probe::CompletedReadBeforeFailure());
  for(std::size_t e=0;e<out.size();++e)ExactVehicle(before[e],out[e]);
  EXPECT_TRUE(spring::batch_detail::SameDiagnostics(identity,candidate));
  EXPECT_TRUE(fe::trial_identity::SameStamp(accepted,rig.owner.accepted()));
  rig.Discard();vehicle::Fields fields(rig.initial.n);fe::NodalStamp stamp;
  ASSERT_EQ(rig.owner.CopyAccepted(fields.buffer(),&stamp).status,fe::NodalStatus::Ok);
  vehicle::InitialFields(rig.initial,fields);EXPECT_TRUE(fe::trial_identity::SameStamp(accepted,stamp));
  fe::NodalAssemblyView view;ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(rig.batch.AssembleAccepted(rig.owner,view).status,spring::BatchStatus::Unusable);rig.Discard();
}
} // namespace type25_batch_test
