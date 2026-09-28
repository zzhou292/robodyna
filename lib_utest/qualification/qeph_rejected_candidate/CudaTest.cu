// SPDX-License-Identifier: MIT
#include "../qt_mapped/OwnerFixture.h"
#include "lib_src/elements/qeph/rejected_candidate/Types.h"
#include "lib_src/elements/qeph/rejected_candidate/Read.h"
#include <cstring>
#include <limits>
namespace rejected_capture_test {
namespace fe=tl::fea;namespace q=fe::qeph;
using Rig=qt_mapped_test::Rig<qt_mapped_test::Quad>;
struct Transfers {bool enabled=false;std::size_t calls=0,fail=0,forces=0,bytes=0;} transfers;
void BeginWatch(std::size_t fail=0) {transfers={true,0,fail,0,0};}
template<class T> auto Bytes(const T& value) {return qt_mapped_test::Bytes(value);}
std::uint64_t Bits(double value) {std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));return bits;}
struct Attempt {
  Rig rig;fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView view;
  q::BatchDiagnostics diagnostics;
  bool Prepare() {return rig.Initialize()&&rig.Prepare(token,assembly,view);}
  q::BatchReport Fail(bool aggregate=false) {
    const auto& ref=qt_mapped_test::Quad::Reference(rig.fixture,rig.config.element_count-1);
    const auto node=rig.fixture.mechanics.domain.Find(ref.input.node_ids[3]);
    if(aggregate) {
      const double invalid=0;
      EXPECT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.orientation_wxyz)+4*node,
          &invalid,sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
    } else {
      const double invalid=std::numeric_limits<double>::quiet_NaN();
      EXPECT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.position_xyz)+3*node,
          &invalid,sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
    }
    EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
    return rig.batch.EvaluateCandidate(rig.owner,token,view,&diagnostics);
  }
};
TEST(RejectedCandidateCuda,ActualOwnerFailureCapturesExactInputsAndReplaysWithoutPublishing) {
  Attempt a;ASSERT_TRUE(a.Prepare());
  const auto accepted=qt_mapped_test::Values(a.rig.Accepted());
  const auto state=a.rig.AcceptedState();const auto stamp=a.rig.owner.accepted();
  const auto failed=a.Fail();ASSERT_EQ(failed.status,q::BatchStatus::ElementFailure);
  q::RejectedCandidateInput captured;BeginWatch();
  const auto copied=a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,failed,&captured);
  transfers.enabled=false;
  ASSERT_EQ(copied.status,q::RejectedCaptureStatus::Captured)<<copied.message;
  EXPECT_TRUE(copied.metadata_available);EXPECT_FALSE(copied.metadata.candidate.valid);
  EXPECT_EQ(transfers.forces,1u);EXPECT_LT(transfers.bytes,65536u);
  EXPECT_TRUE(captured.has_mixed);EXPECT_TRUE(captured.has_failure);
  EXPECT_EQ(captured.failure_policy,fe::ShellFailurePolicy::Tab1AnyPoint);
  EXPECT_TRUE(captured.source.available);
  EXPECT_EQ(captured.source.parent,a.rig.fixture.Physical().shells()->qeph_source_id(failed.element));
  EXPECT_EQ(captured.metadata.original.element,failed.element);
  bool observed_nan=false;
  for(unsigned k=0;k<4;++k) {
    const auto n=captured.element.nodes[k];double fields[9];
    ASSERT_EQ(cudaMemcpy(fields,a.view.kinematics.position_xyz+3*n,24,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(fields+3,a.view.kinematics.velocity_xyz+3*n,24,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(fields+6,a.view.kinematics.angular_velocity_xyz+3*n,24,cudaMemcpyDeviceToHost),cudaSuccess);
    const auto& i=captured.interval;const double actual[]{i.position_endpoint[k].x,i.position_endpoint[k].y,i.position_endpoint[k].z,
      i.velocity_midpoint[k].x,i.velocity_midpoint[k].y,i.velocity_midpoint[k].z,
      i.omega_midpoint[k].x,i.omega_midpoint[k].y,i.omega_midpoint[k].z};
    for(unsigned j=0;j<9;++j)EXPECT_EQ(Bits(fields[j]),Bits(actual[j]));
    observed_nan|=std::isnan(fields[0]);
  }
  EXPECT_TRUE(observed_nan);
  q::RejectedReplayResult replay;ASSERT_TRUE(q::ReplayRejectedCandidate(captured,&replay));
  EXPECT_EQ(replay.operator_status,failed.element_status);EXPECT_FALSE(replay.force_available);
  EXPECT_FALSE(replay.mapped_result_checked);
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,a.rig.owner.accepted()));
  EXPECT_EQ(qt_mapped_test::Values(a.rig.Accepted()),accepted);EXPECT_EQ(a.rig.AcceptedState(),state);
  std::vector<q::ForceTrial> outputs(a.rig.config.element_count);
  EXPECT_NE(a.rig.batch.CopyPreparedResults(copied.metadata.candidate,outputs.data(),outputs.size()).status,q::BatchStatus::Success);
}
TEST(RejectedCandidateCuda,WrongReportViewOwnerAndAliasingLeaveCallerUntouched) {
  Attempt a;ASSERT_TRUE(a.Prepare());const auto failed=a.Fail();ASSERT_EQ(failed.status,q::BatchStatus::ElementFailure);
  q::RejectedCandidateInput output;output.metadata.owner.time=-99;const auto before=Bytes(output);
  auto wrong=failed;wrong.element=(failed.element+1)%a.rig.config.element_count;
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,wrong,&output).status,q::RejectedCaptureStatus::StaleTrial);
  auto view=a.view;++view.attempt;
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,view,failed,&output).status,q::RejectedCaptureStatus::StaleTrial);
  Rig other;ASSERT_TRUE(other.Initialize());
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(other.owner,a.token,a.view,failed,&output).status,q::RejectedCaptureStatus::StaleTrial);
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,failed,
      reinterpret_cast<q::RejectedCandidateInput*>(&a.view)).status,q::RejectedCaptureStatus::InvalidInput);
  EXPECT_EQ(Bytes(output),before);
  a.rig.batch.DiscardTrial();
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,failed,&output).status,q::RejectedCaptureStatus::NoRejectedCandidate);
  EXPECT_EQ(Bytes(output),before);
}
TEST(RejectedCandidateCuda,SuccessfulAndDiscardedOwnerAttemptsCannotBeCapturedAndRetryWorks) {
  Attempt a;ASSERT_TRUE(a.Prepare());q::RejectedCandidateInput output;const auto before=Bytes(output);
  auto result=a.rig.batch.EvaluateCandidate(a.rig.owner,a.token,a.view,&a.diagnostics);ASSERT_EQ(result.status,q::BatchStatus::Success);
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,result,&output).status,q::RejectedCaptureStatus::NoRejectedCandidate);
  EXPECT_EQ(Bytes(output),before);
  a.rig.owner.Discard();a.rig.batch.DiscardTrial();ASSERT_TRUE(a.rig.Prepare(a.token,a.assembly,a.view));
  result=a.Fail();ASSERT_EQ(result.status,q::BatchStatus::ElementFailure);
  a.rig.owner.Discard();
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,result,&output).status,q::RejectedCaptureStatus::StaleTrial);
  EXPECT_EQ(Bytes(output),before);
  a.rig.batch.DiscardTrial();ASSERT_TRUE(a.rig.Prepare(a.token,a.assembly,a.view));
  EXPECT_EQ(a.rig.batch.EvaluateCandidate(a.rig.owner,a.token,a.view,&a.diagnostics).status,q::BatchStatus::Success);
}
TEST(RejectedCandidateCuda,AggregateFailureExposesOnlyTypedMetadataWithoutChoosingAParent) {
  Attempt a;ASSERT_TRUE(a.Prepare());const auto failed=a.Fail(true);
  ASSERT_EQ(failed.status,q::BatchStatus::NonfiniteResult);ASSERT_EQ(failed.element,UINT32_MAX);
  q::RejectedCandidateInput output;const auto before=Bytes(output);BeginWatch();
  const auto report=a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,failed,&output);transfers.enabled=false;
  EXPECT_EQ(report.status,q::RejectedCaptureStatus::UnsupportedFailure);EXPECT_TRUE(report.metadata_available);
  EXPECT_EQ(report.metadata.original.element,UINT32_MAX);EXPECT_FALSE(report.metadata.candidate.valid);
  EXPECT_EQ(transfers.forces,0u);EXPECT_EQ(Bytes(output),before);
}
TEST(RejectedCandidateCuda,ReadFailureAtEveryTransferIsAtomicAndDoesNotMaskOriginalStatus) {
  Attempt a;ASSERT_TRUE(a.Prepare());const auto failed=a.Fail();ASSERT_EQ(failed.status,q::BatchStatus::ElementFailure);
  q::RejectedCandidateInput output;BeginWatch();
  ASSERT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,failed,&output).status,q::RejectedCaptureStatus::Captured);
  const auto calls=transfers.calls;transfers.enabled=false;const auto before=Bytes(output);
  for(std::size_t k=1;k<=calls;++k) {
    SCOPED_TRACE(k);BeginWatch(k);const auto report=a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,failed,&output);transfers.enabled=false;
    EXPECT_EQ(report.status,q::RejectedCaptureStatus::DeviceFailure);EXPECT_EQ(report.cuda_status,cudaErrorInvalidValue);
    EXPECT_EQ(Bytes(output),before);EXPECT_EQ(failed.status,q::BatchStatus::ElementFailure);
  }
  EXPECT_EQ(a.rig.batch.CopyRejectedCandidate(a.rig.owner,a.token,a.view,failed,&output).status,q::RejectedCaptureStatus::Captured);
}
TEST(RejectedCandidateCuda,DeviceCurveSubrangeBeyond1024UsesExactArenaBounds) {
  double* allocation=nullptr;ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&allocation),2048*sizeof(double)),cudaSuccess);
  EXPECT_TRUE(q::rejected_detail::CurveRange(allocation+1500,allocation,2048,32));
  EXPECT_TRUE(q::rejected_detail::CurveRange(allocation+2046,allocation,2048,2));
  EXPECT_FALSE(q::rejected_detail::CurveRange(allocation+2047,allocation,2048,2));
  EXPECT_FALSE(q::rejected_detail::CurveRange(allocation,allocation,2048,1025));
  ASSERT_EQ(cudaFree(allocation),cudaSuccess);
}
} // namespace rejected_capture_test
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* destination,const void* source,std::size_t bytes,cudaMemcpyKind kind,cudaStream_t stream) {
  auto& t=rejected_capture_test::transfers;
  if(t.enabled&&kind==cudaMemcpyDeviceToHost) {
    ++t.calls;t.bytes+=bytes;if(bytes==sizeof(tl::fea::qeph::ForceTrial))++t.forces;
    if(t.fail==t.calls)return cudaErrorInvalidValue;
  }
  return __real_cudaMemcpyAsync(destination,source,bytes,kind,stream);
}
