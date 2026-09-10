#include "MixedResidentFixture.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/ShellMixedSectionArenaLayout.h"

namespace mixed_layered_test {
TEST_F(MixedShellCuda, MixedLayeredReadbackRejectsUnavailableLegacyFieldsAndAliasedOrStaleReceipts) {
  Rig r;fe::ShellBatchSectionBinding catalog;ASSERT_TRUE(Initialize(r,catalog));ASSERT_TRUE(r.Bind());
  Frame old;ASSERT_TRUE(ReadFrame(r,old));
  Prepared p;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),p));Frame candidate;ASSERT_TRUE(Evaluate(r,p,candidate));
  auto output=old.qsection;const auto before=Bytes(output);auto receipt=candidate.diagnostics.qeph;
  const auto saved_receipt=Bytes(receipt);
  EXPECT_EQ(r.qeph.CopyPreparedLayeredSectionHistory(receipt,output.data(),1).status,q::BatchStatus::ResourceLimit);
  EXPECT_EQ(Bytes(output),before);
  EXPECT_EQ(r.qeph.CopyPreparedLayeredSectionHistory(receipt,
    reinterpret_cast<fe::ShellBatchLayeredSection*>(&receipt),Parents).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(receipt),saved_receipt);
  auto stamp=r.owner.accepted();const auto saved_stamp=Bytes(stamp);
  EXPECT_EQ(r.qeph.CopyAcceptedLayeredSectionHistory(stamp,output.data(),Parents,
    reinterpret_cast<q::BatchDiagnostics*>(output.data())).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(output),before);EXPECT_EQ(Bytes(stamp),saved_stamp);
  ++receipt.attempt;
  EXPECT_EQ(r.qeph.CopyPreparedLayeredSectionHistory(receipt,output.data(),Parents).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(output),before);
  std::array<fe::ShellBatchSectionState,Parents> legacy{};const auto legacy_before=Bytes(legacy);
  EXPECT_EQ(r.qeph.CopyPreparedSectionHistory(candidate.diagnostics.qeph,legacy.data(),Parents).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(legacy),legacy_before);
  r.Discard();
  EXPECT_EQ(r.qeph.CopyPreparedLayeredSectionHistory(candidate.diagnostics.qeph,output.data(),Parents).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(output),before);
  Prepared retry;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),retry));Frame next;ASSERT_TRUE(Evaluate(r,retry,next));SameFrame(next,candidate);
  EXPECT_EQ(r.qeph.CopyPreparedLayeredSectionHistory(candidate.diagnostics.qeph,output.data(),Parents).status,q::BatchStatus::StaleTrial);
  ASSERT_TRUE(Commit(r,retry,next));
  EXPECT_EQ(r.qeph.CopyPreparedLayeredSectionHistory(next.diagnostics.qeph,output.data(),Parents).status,q::BatchStatus::StaleTrial);
  q::QephBatchConfig capped;capped.owner=stamp;capped.element_count=Parents;
  capped.configuration_id=Configuration;capped.qualification_id=Qualification;capped.usage=q::BatchUsage::PrescribedFields;
  q::batch_detail::Layout plain;ASSERT_TRUE(plain.Initialize(Parents,Nodes,1<<20));
  fe::shell_batch_plasticity_detail::MixedLayout section;ASSERT_TRUE(section.Initialize(Parents,3,1<<20));
  capped.max_device_bytes=plain.bytes+section.bytes-1;
  q::QephBatch denied;EXPECT_EQ(denied.InitializeJoined(capped,r.binding,catalog).status,q::BatchStatus::ResourceLimit);
  EXPECT_EQ(denied.allocations().device_allocations,0u);
}
TEST_F(MixedShellCuda, MixedLayeredLateActiveReadbackNonfinitePreservesOutputsAndAllowsExactRetry) {
  Rig r;fe::ShellBatchSectionBinding catalog;ASSERT_TRUE(Initialize(r,catalog));ASSERT_TRUE(r.Bind());
  Snapshot initial;ASSERT_TRUE(Read(r.owner,initial));Frame old;ASSERT_TRUE(ReadFrame(r,old));
  Prepared p;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),p));Frame candidate;ASSERT_TRUE(Evaluate(r,p,candidate));
  auto output=old.tsection;const auto before=Bytes(output);const auto receipt=candidate.diagnostics.t3;
  const auto receipt_before=Bytes(receipt);
  ArmReadFault(ReadFault::Nonfinite);
  EXPECT_EQ(r.t3.CopyPreparedLayeredSectionHistory(receipt,output.data(),Parents).status,t::BatchStatus::NonfiniteResult);
  EXPECT_EQ(ReadFaultCopies(),2u);EXPECT_EQ(Bytes(output),before);EXPECT_EQ(Bytes(receipt),receipt_before);
  EXPECT_EQ(r.t3.CopyPreparedLayeredSectionHistory(receipt,output.data(),Parents).status,t::BatchStatus::StaleTrial);
  r.Discard();Frame held;ASSERT_TRUE(ReadFrame(r,held));SameFrame(held,old);
  Snapshot state;ASSERT_TRUE(Read(r.owner,state));SameState(state,initial);
  Prepared retry;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),retry));Frame next;
  ASSERT_TRUE(Evaluate(r,retry,next));SameFrame(next,candidate);ASSERT_TRUE(Commit(r,retry,next));
}
TEST_F(MixedShellCuda, MixedLayeredCompletedFinalCopyFailurePublishesNothingAndPoisonsParticipant) {
  Rig r;fe::ShellBatchSectionBinding catalog;ASSERT_TRUE(Initialize(r,catalog));ASSERT_TRUE(r.Bind());
  Snapshot initial;ASSERT_TRUE(Read(r.owner,initial));Frame old;ASSERT_TRUE(ReadFrame(r,old));
  Prepared p;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),p));Frame candidate;ASSERT_TRUE(Evaluate(r,p,candidate));
  auto output=old.tsection;const auto before=Bytes(output);const auto receipt=candidate.diagnostics.t3;
  const auto receipt_before=Bytes(receipt);
  ArmReadFault(ReadFault::DeviceError);
  EXPECT_EQ(r.t3.CopyPreparedLayeredSectionHistory(receipt,output.data(),Parents).status,t::BatchStatus::DeviceFailure);
  EXPECT_EQ(ReadFaultCopies(),2u);EXPECT_EQ(Bytes(output),before);EXPECT_EQ(Bytes(receipt),receipt_before);
  t::BatchDiagnostics diagnostic;
  EXPECT_EQ(r.t3.CopyAcceptedLayeredSectionHistory(r.owner.accepted(),output.data(),Parents,&diagnostic).status,t::BatchStatus::DeviceFailure);
  EXPECT_EQ(Bytes(output),before);r.Discard();Snapshot state;ASSERT_TRUE(Read(r.owner,state));SameState(state,initial);
}
TEST_F(MixedShellCuda, MixedLayeredPublicationRejectsDifferentCompleteSourceCatalog) {
  Rig r;fe::ShellBatchSectionBinding catalog;ASSERT_TRUE(Initialize(r,catalog,true));
  ASSERT_TRUE(AssembleCollectionForBinding(r));
  EXPECT_EQ(r.publication.Initialize(r.owner,r.qeph,r.t3).status,fe::ShellPublicationStatus::InvalidInput);
  EXPECT_EQ(r.owner.accepted().epoch,0u);
}
} // namespace mixed_layered_test
