#include "CudaFixture.h"
#include "CopyFault.h"
#include <type_traits>
static_assert(!std::is_aggregate_v<tlfea::contact::radioss_type25::search::ReferenceToken>);
#include <limits>
namespace type25_search_test {
TEST_F(Type25SearchCuda, NativeAndSiSparseDenseReferenceAndCurrentGapMatchIndependentOracle) {
  for(bool compact:{false,true})for(bool si:{false,true})for(bool gaps:{false,true}) {
    DeviceFixture d(compact,gaps,si);d.Initialize();d.Reference();
    const auto saved=d.fixture.positions,old_gaps=d.fixture.gaps;
    auto reference=d.fixture.Current();reference.positions.data=saved.data();
    if(gaps)reference.main_gaps=old_gaps.data();
    d.fixture.positions[0]+=.125*(si?.001:1.);
    d.fixture.velocities[3]=2*(si?.001:1.);
    d.fixture.gaps[0]+=.25*(si?.001:1.);d.fixture.stamp.epoch=1;++d.fixture.stamp.attempt;d.Upload();
    d.Compare(reference);d.Compare(reference,0,true);
  }
}
TEST_F(Type25SearchCuda, RefreshPublishDiscardForeignTokensAndFailedStagesPreserveReference) {
  DeviceFixture d;d.Initialize();s::Report prior;prior.budget.distance=123;
  EXPECT_EQ(d.owner.Evaluate(d.current,1,false,prior),s::Status::NoReference);EXPECT_EQ(prior.budget.distance,123);
  const auto first=d.Stage();d.owner.DiscardReference();
  EXPECT_EQ(d.owner.PublishReference(first),s::Status::StaleReference);
  d.Reference();EXPECT_EQ(d.owner.reference_generation(),1u);
  const auto saved=d.fixture.positions,old_gaps=d.fixture.gaps;
  auto reference=d.fixture.Current();reference.positions.data=saved.data();reference.main_gaps=old_gaps.data();
  d.fixture.positions[0]+=.5;d.fixture.stamp.epoch=2;++d.fixture.stamp.attempt;d.Upload();
  auto staged=d.Stage();d.Compare(reference);EXPECT_EQ(d.owner.reference_generation(),1u);
  d.owner.DiscardReference();EXPECT_EQ(d.owner.PublishReference(staged),s::Status::StaleReference);
  staged=d.Stage();DeviceFixture other;other.Initialize();other.Reference();
  EXPECT_EQ(other.owner.PublishReference(staged),s::Status::StaleReference);
  EXPECT_EQ(d.owner.PublishReference(staged),s::Status::Ok);EXPECT_EQ(d.owner.reference_generation(),2u);
  EXPECT_EQ(d.owner.PublishReference(staged),s::Status::StaleReference);
  d.Compare(d.fixture.Current());
}
TEST_F(Type25SearchCuda, InvalidGapStageCannotReplaceAcceptedSnapshotOrReviveOldToken) {
  DeviceFixture d;d.Initialize();d.Reference();auto token=d.Stage();
  const auto saved=d.fixture.positions,old_gaps=d.fixture.gaps;
  auto reference=d.fixture.Current();reference.positions.data=saved.data();reference.main_gaps=old_gaps.data();
  d.fixture.gaps[1]=-1;d.Upload();
  EXPECT_EQ(d.owner.StageReference(d.current,token),s::Status::NonfiniteResult);
  EXPECT_EQ(d.owner.PublishReference(token),s::Status::StaleReference);EXPECT_EQ(d.owner.reference_generation(),1u);
  d.fixture.gaps=old_gaps;d.Upload();d.Compare(reference);
}
TEST_F(Type25SearchCuda, ActivityGenerationsAndHiddenMaskChangesFailBeforePublication) {
  DeviceFixture d;d.Initialize();d.Reference();s::ReferenceToken token;s::Report result;result.budget.distance=97;
  ++d.fixture.stamp.source.activity;d.Upload();
  EXPECT_EQ(d.owner.Evaluate(d.current,1,false,result),s::Status::UnsupportedLifecycle);EXPECT_EQ(result.budget.distance,97);
  EXPECT_EQ(d.owner.StageReference(d.current,token),s::Status::UnsupportedLifecycle);
  --d.fixture.stamp.source.activity;d.fixture.stiffness[0]=0;d.Upload();
  EXPECT_EQ(d.owner.Evaluate(d.current,1,false,result),s::Status::UnsupportedLifecycle);
  EXPECT_EQ(d.owner.StageReference(d.current,token),s::Status::UnsupportedLifecycle);
  EXPECT_EQ(d.owner.reference_generation(),1u);
}
TEST_F(Type25SearchCuda, EmptyActiveSideIsUnsupportedAndInactiveNonfiniteNodeIsNotConsumed) {
  DeviceFixture d;d.fixture.one_d={UINT32_MAX};d.fixture.Bind();d.fixture.stiffness[0]=0;
  d.fixture.positions[0]=std::numeric_limits<double>::quiet_NaN();d.Upload();d.Initialize();d.Reference();
  d.Compare(d.fixture.Current());
  DeviceFixture empty;empty.fixture.one_d={UINT32_MAX};empty.fixture.Bind();
  for(auto& k:empty.fixture.stiffness)k=0;empty.Upload();empty.Initialize();s::ReferenceToken token;
  EXPECT_EQ(empty.owner.StageReference(empty.current,token),s::Status::UnsupportedLifecycle);
  EXPECT_EQ(empty.owner.reference_generation(),0u);
}
TEST_F(Type25SearchCuda, MultipleBlocksAndInputLayoutsKeepCompleteExtremaAndWarningStatus) {
  DeviceFixture d(true,true,false,4097);
  d.fixture.secondary.resize(2048);d.fixture.main.resize(2048);d.fixture.stiffness.resize(2048,1);
  for(unsigned i=0;i<2048;++i){d.fixture.secondary[i]=i;d.fixture.main[i]=i+2048;}
  d.fixture.Bind();
  // Replace the external test buffer only; owner is not initialized yet.
  Cuda(cudaFree(d.data));d.data=nullptr;Cuda(cudaMalloc(reinterpret_cast<void**>(&d.data),d.Bytes()));
  d.Upload();d.Initialize();d.Reference();
  const auto saved=d.fixture.positions,old_gaps=d.fixture.gaps;
  auto reference=d.fixture.Current();reference.positions.data=saved.data();reference.main_gaps=old_gaps.data();
  d.fixture.velocities[0]=30;d.fixture.positions[3*2047]=1e3;d.Upload();d.Compare(reference,1);
  s::Report report;ASSERT_EQ(d.owner.Evaluate(d.current,1,false,report),s::Status::Ok);
  EXPECT_EQ(report.budget.velocity,s::VelocityStatus::Error);EXPECT_EQ(report.budget.stored_motion,d.fixture.source.margin);
}
TEST_F(Type25SearchCuda, StartupCopiedMapsAndBorrowedInputGuardsAreExplicit) {
  DeviceFixture d;d.Initialize();d.Reference();const auto saved=d.fixture.positions,old_gaps=d.fixture.gaps;
  auto reference=d.fixture.Current();reference.positions.data=saved.data();reference.main_gaps=old_gaps.data();
  const auto old=d.fixture.secondary[0];d.fixture.secondary[0]=UINT32_MAX;
  s::Report report;ASSERT_EQ(d.owner.Evaluate(d.current,1,false,report),s::Status::Ok);
  d.fixture.secondary[0]=old;d.Compare(reference,1);
  auto wrong=d.current;wrong.positions.node_count--;
  report.budget.distance=99;EXPECT_EQ(d.owner.Evaluate(wrong,1,false,report),s::Status::InvalidInput);EXPECT_EQ(report.budget.distance,99);
  wrong=d.current;wrong.stamp.epoch=0;wrong.stamp.attempt=0;
  EXPECT_EQ(d.owner.Evaluate(wrong,1,false,report),s::Status::StaleReference);
}
TEST_F(Type25SearchCuda, SameSequenceForeignTokensCannotPublishAndDiscardedTokenCannotRevive) {
  DeviceFixture a,b;a.Initialize();b.Initialize();
  const auto token_a=a.Stage(),token_b=b.Stage();
  EXPECT_EQ(a.owner.PublishReference(token_b),s::Status::StaleReference);
  EXPECT_EQ(b.owner.PublishReference(token_a),s::Status::StaleReference);
  EXPECT_EQ(a.owner.PublishReference(token_a),s::Status::Ok);
  EXPECT_EQ(b.owner.PublishReference(token_b),s::Status::Ok);
  auto old=a.Stage();a.owner.DiscardReference();const auto current=a.Stage();
  EXPECT_EQ(a.owner.PublishReference(old),s::Status::StaleReference);
  EXPECT_EQ(a.owner.PublishReference(current),s::Status::Ok);
}
TEST_F(Type25SearchCuda, SoAPositionsAndAoSVelocitiesUseExistingBorrowedViewWithoutConversionArrays) {
  DeviceFixture d;d.Initialize();d.Reference();
  const auto saved=d.fixture.positions,old_gaps=d.fixture.gaps;
  auto reference=d.fixture.Current();reference.positions.data=saved.data();reference.main_gaps=old_gaps.data();
  const auto count=d.fixture.source.physical_nodes;
  for(std::size_t i=0;i<count;++i)for(unsigned c=0;c<3;++c)
    d.fixture.positions[c*count+i]=saved[3*i+c]+(i%3==0 ? .25 : 0.);
  d.fixture.velocities[2]=7;d.Upload();d.current.positions.node_stride=1;d.current.positions.component_stride=count;
  auto host=d.fixture.Current();host.positions.node_stride=1;host.positions.component_stride=count;
  s::Report report;ASSERT_EQ(d.owner.Evaluate(d.current,.01,false,report),s::Status::Ok);
  const auto expected=NativeExtrema(d.fixture.source,host,reference);
  Same(report.extrema,expected);Same(report.budget,NativeBudget(expected,d.fixture.source.margin,.01,false));
  auto token=d.Stage();ASSERT_EQ(d.owner.PublishReference(token),s::Status::Ok);
  ASSERT_EQ(d.owner.Evaluate(d.current,.01,false,report),s::Status::Ok);
  Same(report.extrema,NativeExtrema(d.fixture.source,host,host));
}
TEST_F(Type25SearchCuda, CudaCopyFailurePoisonsWithoutReferencePublicationOrCpuRetry) {
  DeviceFixture d;d.Initialize();d.Reference();auto token=d.Stage();
  copy_fault::Arm();
  EXPECT_EQ(d.owner.StageReference(d.current,token),s::Status::DeviceFailure);
  EXPECT_EQ(d.owner.reference_generation(),1u);
  EXPECT_EQ(d.owner.PublishReference(token),s::Status::Unusable);
  s::Report result;result.budget.distance=123;
  EXPECT_EQ(d.owner.Evaluate(d.current,1,false,result),s::Status::Unusable);
  EXPECT_EQ(result.budget.distance,123);
}
} // namespace type25_search_test

#include "AdmissionCudaCases.h"
