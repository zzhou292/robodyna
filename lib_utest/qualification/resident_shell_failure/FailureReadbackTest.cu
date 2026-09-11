#include "FailureResidentFixture.h"
namespace resident_failure_test {
TEST_F(Cuda,ReadbackPreflightsIdentityRangesAndLateValuesWithoutPublishingPartialActivity) {
  Rig rig;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(Initialize(rig,catalog,failure));
  ASSERT_TRUE(rig.Bind());
  Frame initial;
  ASSERT_TRUE(Read(rig,initial));
  auto output=initial.qfailure;
  qe::BatchDiagnostics diagnostic;
  auto stamp=rig.owner.accepted();
  EXPECT_EQ(rig.qeph.CopyAcceptedFailureHistory(stamp,reinterpret_cast<fe::ShellBatchFailureState*>(1),Parents-1,&diagnostic).status,qe::BatchStatus::ResourceLimit);
  ++stamp.epoch;
  EXPECT_EQ(rig.qeph.CopyAcceptedFailureHistory(stamp,output.data(),Parents,&diagnostic).status,qe::BatchStatus::StaleTrial);
  stamp=rig.owner.accepted();
  EXPECT_EQ(rig.qeph.CopyAcceptedFailureHistory(stamp,output.data(),Parents,reinterpret_cast<qe::BatchDiagnostics*>(output.data())).status,qe::BatchStatus::InvalidInput);
  for(auto fault:{Fault::Nonfinite,Fault::InvalidFlag}) {
    Prepared prepared;
    ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),prepared));
    Frame next;
    ASSERT_TRUE(Evaluate(rig,prepared,next));
    const auto before=plasticity_binding_test::Bytes(output);
    Arm(fault);
    const auto report=rig.qeph.CopyPreparedFailureHistory(next.material.diagnostics.qeph,output.data(),Parents);
    EXPECT_EQ(report.status,qe::BatchStatus::NonfiniteResult);
    EXPECT_EQ(Copies(),3u);
    EXPECT_EQ(plasticity_binding_test::Bytes(output),before);
    EXPECT_EQ(rig.qeph.CopyPreparedFailureHistory(next.material.diagnostics.qeph,output.data(),Parents).status,qe::BatchStatus::StaleTrial);
    rig.Discard();
    Frame held;
    ASSERT_TRUE(Read(rig,held));
    Same(held,initial);
    Prepared retry;
    ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),retry));
    Frame actual;
    ASSERT_TRUE(Evaluate(rig,retry,actual));
    Same(actual,next);
    rig.Discard();
  }
  const auto before=plasticity_binding_test::Bytes(output);
  const auto diagnostic_before = plasticity_binding_test::Bytes(diagnostic);
  Arm(Fault::DeviceError);
  EXPECT_EQ(rig.qeph.CopyAcceptedFailureHistory(stamp,output.data(),Parents,&diagnostic).status,qe::BatchStatus::DeviceFailure);
  EXPECT_EQ(plasticity_binding_test::Bytes(output),before);
  EXPECT_EQ(plasticity_binding_test::Bytes(diagnostic),diagnostic_before);
  EXPECT_EQ(rig.qeph.CopyAcceptedFailureHistory(stamp,output.data(),Parents,&diagnostic).status,qe::BatchStatus::DeviceFailure);
}
TEST_F(Cuda,CommonCommitRejectsLateReceiptAndCannotPublishOneFailureFamilyAlone) {
  Rig rig;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(Initialize(rig,catalog,failure));
  ASSERT_TRUE(rig.Bind());
  Frame old;
  ASSERT_TRUE(Read(rig,old));
  Prepared prepared;
  ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),prepared));
  Frame next;
  ASSERT_TRUE(Evaluate(rig,prepared,next));
  const auto& d=next.material.diagnostics.qeph;
  EXPECT_EQ(rig.publication.Commit(rig.owner,prepared.token,next.material.diagnostics,
    {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,false}).status,fe::ShellPublicationStatus::StaleTrial);
  Frame held;
  ASSERT_TRUE(Read(rig,held));
  Same(held,old);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
  Prepared retry;
  ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),retry));
  Frame actual;
  ASSERT_TRUE(Evaluate(rig,retry,actual));
  Same(actual,next);
  const auto qd=actual.material.diagnostics.qeph;
  EXPECT_NE(qe::CommitQephTrial(rig.owner,retry.token,rig.qeph,qd,
    {qd.owner_id,qd.base_epoch,qd.attempt,qd.qualification_id,true}).status,qe::BatchStatus::Success);
  rig.Discard();
  Frame final;
  ASSERT_TRUE(Read(rig,final));
  Same(final,old);
}
TEST_F(Cuda,SidecarByteCapsRejectWithoutDeviceAllocationAndPermitCleanInitialization) {
  Rig fixture;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(Initialize(fixture,catalog,failure,false));
  qe::QephBatch qbatch;
  tr::T3Batch tbatch;
  qe::QephBatchConfig qc;
  qc.owner = fixture.owner.accepted();
  qc.element_count = Parents;
  qc.configuration_id = mixed::Configuration;
  qc.qualification_id = mixed::Qualification;
  qc.usage = qe::BatchUsage::PrescribedFields;
  tr::T3BatchConfig tc;
  tc.owner = qc.owner;
  tc.element_count = Parents;
  tc.configuration_id = qc.configuration_id;
  tc.qualification_id = qc.qualification_id;
  tc.usage = tr::BatchUsage::PrescribedFields;
  for (bool device : {false,true}) {
    fe::ShellBatchFailureLimits limits;
    if (device) limits.max_device_bytes = 1;
    else limits.max_host_bytes = 1;
    EXPECT_EQ(qbatch.InitializeJoined(qc,fixture.binding,catalog,failure,limits).status,
              qe::BatchStatus::ResourceLimit);
    EXPECT_EQ(tbatch.InitializeJoined(tc,fixture.binding,catalog,failure,limits).status,
              tr::BatchStatus::ResourceLimit);
    EXPECT_EQ(qbatch.allocations().device_allocations,0u);
    EXPECT_EQ(tbatch.allocations().device_allocations,0u);
  }
  EXPECT_EQ(qbatch.InitializeJoined(qc,fixture.binding,catalog,failure,{}).status,qe::BatchStatus::Success);
  EXPECT_EQ(tbatch.InitializeJoined(tc,fixture.binding,catalog,failure,{}).status,tr::BatchStatus::Success);
}
} // namespace resident_failure_test
