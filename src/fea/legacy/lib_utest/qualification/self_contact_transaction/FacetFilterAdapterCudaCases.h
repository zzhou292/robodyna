// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FacetFilterCudaProbe.h"
#include "TransactionReportAssertions.h"
#include "../self_contact_current_regularity/Fixture.h"
#include "lib_src/collision/self_contact_transaction/FacetFilters.h"
#include <cfenv>

namespace self_contact_transaction_cuda_test {
namespace facet_filter_adapter_test {
namespace filters=c::self_contact_filters;
struct RestoreEnvironment {
  fenv_t saved;
  RestoreEnvironment(){std::fegetenv(&saved);}
  ~RestoreEnvironment(){std::fesetenv(&saved);}
};
struct NumericFixture {
  current_regularity_test::Fixture source{0,true};
  std::vector<c::FixedContactFacet> descriptors;
  std::vector<c::CurrentFixedTriangle> base,next;
  std::vector<sct::MotionSupport> motion;
  std::vector<c::SelfContactSweptParentBounds> bounds;
  std::vector<c::FixedTrianglePair> pairs{{0,1},{0,2}};
  cudaStream_t stream=nullptr;
  sct::FacetFilters adapter;
  ~NumericFixture(){if(stream)cudaStreamDestroy(stream);}
  bool Geometry() {
    const auto count=source.uses.facet_uses().size();
    descriptors.resize(count);base.resize(count);motion.resize(count);bounds.resize(count);
    for(std::size_t i=0;i<count;++i) {
      const auto& use=source.uses.facet_uses()[i];const auto& parent=source.uses.parents()[use.parent];
      if(source.source.facets.Describe(parent.surface_parent,use.local_facet,&descriptors[i]).status!=c::FixedContactFacetStatus::Ok)return false;
      motion[i].parent=use.parent;motion[i].certified_affine=true;
      bounds[i]={{-10,-10,-10},{10,10,10}};
    }
    if(sct::EvaluateCompleteTriangles(descriptors.data(),count,source.Positions(),base.data()).status!=c::SelfContactTransactionStatus::Ok)return false;
    next=base;
    return cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking)==cudaSuccess;
  }
  bool Initialize(){return adapter.Initialize(source.uses,16,1u<<20,1u<<20,stream).status==filters::Status::Ok;}
  bool Candidate(){return adapter.CandidateScene(base.data(),next.data(),motion.data(),bounds.data()).status==filters::Status::Ok;}
  void Compare(std::size_t index,const sct::FacetPrismReply& reply) {
    ASSERT_TRUE(reply.supplied);ASSERT_EQ(reply.report.status,filters::Status::Ok);
    const auto pair=pairs[index];const auto parents=source.uses.parents();
    bool valid=false;c::SelfContactFacetPrismSeparationAxis axis{};
    const bool separated=c::CertifiedLinearFacetPrismSeparation(base[pair.first],next[pair.first],
        parents[motion[pair.first].parent].reference_half_thickness_m,
        base[pair.second],next[pair.second],parents[motion[pair.second].parent].reference_half_thickness_m,
        c::SelfContactFacetPrismAxisLimit::VertexVertex,&axis,&valid);
    EXPECT_EQ(reply.value.status,valid?c::SelfContactFacetFilterStatus::Ok:c::SelfContactFacetFilterStatus::InvalidInput);
    EXPECT_EQ(reply.value.separated,separated);EXPECT_EQ(reply.value.axis,axis);
  }
};
}
TEST(SelfContactFacetFilterAdapterCuda, NullBackendPreservesScalarDispatch) {
  using namespace facet_filter_adapter_test;
  // This runtime template references PrismAt even for a null pointer in an
  // unoptimized build, so qualify it in the adapter's owning executable.
  bool valid = false;
  c::SelfContactFacetPrismSeparationAxis axis{};
  filters::Report report;
  unsigned calls = 0;
  const bool separated = sct::OptionalFacetPrism(nullptr, 0, [&] {
    ++calls;
    valid = true;
    axis = c::SelfContactFacetPrismSeparationAxis::VertexEdge;
    return true;
  }, &axis, &valid, &report);
  EXPECT_TRUE(separated && valid);
  EXPECT_EQ(calls, 1u);
  EXPECT_EQ(axis, c::SelfContactFacetPrismSeparationAxis::VertexEdge);
  EXPECT_EQ(report.status, filters::Status::Ok);
}
TEST(SelfContactFacetFilterAdapterCuda, AcceptedNumericErrorPrecedesLaterInvalidOrdinalAndRetry) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_TRUE(f.Initialize());
  const auto saved=f.base;f.base[0].vertices[0].x=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(f.adapter.AcceptedScene(f.base.data(),f.motion.data()).status,filters::Status::Ok);
  std::vector<c::FixedTrianglePair> original{{0,1},{0,UINT32_MAX}},actual=original;
  std::size_t expected_count=original.size(),actual_count=actual.size();
  const auto expected=sct::FilterAcceptedFacetPairs(f.source.uses,f.base.data(),f.motion.data(),f.base.size(),original.data(),&expected_count);
  const auto report=f.adapter.AcceptedPairs(actual.data(),&actual_count);
  sct::test::ExpectTransactionReport(report,expected);
  EXPECT_EQ(report.pair,0u);EXPECT_EQ(actual_count,expected_count);
  f.base=saved;ASSERT_EQ(f.adapter.AcceptedScene(f.base.data(),f.motion.data()).status,filters::Status::Ok);
  actual=f.pairs;original=f.pairs;actual_count=actual.size();expected_count=original.size();
  sct::test::ExpectTransactionReport(f.adapter.AcceptedPairs(actual.data(),&actual_count),
      sct::FilterAcceptedFacetPairs(f.source.uses,f.base.data(),f.motion.data(),f.base.size(),original.data(),&expected_count));
  ASSERT_EQ(actual_count,expected_count);
  for(std::size_t i=0;i<actual_count;++i){EXPECT_EQ(actual[i].first,original[i].first);EXPECT_EQ(actual[i].second,original[i].second);}
}
TEST(SelfContactFacetFilterAdapterCuda, CachedChunkEnvironmentChangeAndDiscardRequireCorrectReentry) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  f.Compare(0,f.adapter.PrismAt(0));
  const auto copies=facet_filter_cuda_probe::Copies();
  {RestoreEnvironment restore;ASSERT_EQ(std::fesetround(FE_UPWARD),0);
   const auto reply=f.adapter.PrismAt(1);EXPECT_FALSE(reply.supplied);EXPECT_EQ(reply.report.status,filters::Status::Ok);
   EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies);}
  EXPECT_FALSE(f.adapter.PrismAt(1).supplied);
  EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies);
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  f.Compare(1,f.adapter.PrismAt(1));EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies+2);
  f.adapter.Discard();EXPECT_EQ(f.adapter.PrismAt(0).report.status,filters::Status::NoScene);
  ASSERT_TRUE(f.Candidate());ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);f.Compare(0,f.adapter.PrismAt(0));
}
TEST(SelfContactFacetFilterAdapterCuda, UnsupportedStartupMathUsesCpuWithoutFilterCommandsOrAllocations) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());const auto allocations=facet_filter_cuda_probe::Allocations();
  const auto copies=facet_filter_cuda_probe::Copies();
  {RestoreEnvironment restore;ASSERT_EQ(std::fesetround(FE_UPWARD),0);ASSERT_TRUE(f.Initialize());}
  EXPECT_EQ(facet_filter_cuda_probe::Allocations(),allocations);
  ASSERT_TRUE(f.Candidate());ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  EXPECT_FALSE(f.adapter.PrismAt(0).supplied);EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies);
  EXPECT_GT(f.adapter.UnallocatedDevice().device_bytes,0u);
  EXPECT_EQ(f.adapter.initialization_mode(),c::SelfContactFacetFilterInitialization::UnsupportedHostArithmetic);
}
TEST(SelfContactFacetFilterAdapterCuda, QueryCudaFailureIsExplicitPoisonsStorageAndNeverFallsBack) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  facet_filter_cuda_probe::FailNextHostToDeviceCopy();
  const auto failed=f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size());
  EXPECT_EQ(failed.status,filters::Status::DeviceFailure);
  EXPECT_EQ(failed.pair,SIZE_MAX); // The failed chunk query names no physical pair.
  EXPECT_EQ(f.adapter.PrismAt(1).report.status,filters::Status::DeviceFailure);
  f.adapter.Discard();
  EXPECT_EQ(f.adapter.initialization_mode(),c::SelfContactFacetFilterInitialization::Cuda);
  {RestoreEnvironment restore;ASSERT_EQ(std::fesetround(FE_UPWARD),0);
   EXPECT_EQ(f.adapter.CandidateScene(f.base.data(),f.next.data(),f.motion.data(),f.bounds.data()).status,filters::Status::DeviceFailure);
   EXPECT_EQ(f.adapter.AcceptedScene(f.base.data(),f.motion.data()).status,filters::Status::DeviceFailure);
   const auto still_failed=f.adapter.PrismAt(0);
   EXPECT_TRUE(still_failed.supplied);EXPECT_EQ(still_failed.report.status,filters::Status::DeviceFailure);}
  EXPECT_EQ(f.adapter.CandidateScene(f.base.data(),f.next.data(),f.motion.data(),f.bounds.data()).status,
      filters::Status::DeviceFailure);
}
TEST(SelfContactFacetFilterAdapterCuda, MissingLifecycleOrChunkCannotBecomeEnvironmentFallback) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());
  EXPECT_EQ(f.adapter.initialization_mode(),c::SelfContactFacetFilterInitialization::NotInitialized);
  {RestoreEnvironment restore;ASSERT_EQ(std::fesetround(FE_UPWARD),0);
   EXPECT_EQ(f.adapter.PrismAt(0).report.status,filters::Status::NotInitialized);}
  ASSERT_TRUE(f.Initialize());
  {RestoreEnvironment restore;ASSERT_EQ(std::fesetround(FE_UPWARD),0);
   EXPECT_EQ(f.adapter.PrismAt(0).report.status,filters::Status::NoScene);}
  ASSERT_TRUE(f.Candidate());
  EXPECT_EQ(f.adapter.BeginCandidateChunk(nullptr,1).status,filters::Status::InvalidInput);
  {RestoreEnvironment restore;ASSERT_EQ(std::fesetround(FE_UPWARD),0);
   const auto missing=f.adapter.PrismAt(0);
   EXPECT_TRUE(missing.supplied);EXPECT_EQ(missing.report.status,filters::Status::InvalidInput);}
}
TEST(SelfContactFacetFilterAdapterCuda, CompactChunkUsesOneQueryAcrossNonlinearAndExcludedGaps) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_GE(f.motion.size(),4u);
  f.motion[3].certified_affine=false;
  f.motion[2].motion=f.motion[3].motion=c::SelfContactFacetMotion::CompleteRigidGroup;
  f.motion[2].complete_rigid_group=f.motion[3].complete_rigid_group=9;
  f.pairs={{0,1},{0,3},{2,3},{0,2},{0,1},{1,0}};
  ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  const auto copies=facet_filter_cuda_probe::Copies();
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies+2);
  for (unsigned i:{0u,3u,4u,5u}) f.Compare(i,f.adapter.PrismAt(i));
  EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies+2);
}
TEST(SelfContactFacetFilterAdapterCuda, FutureMalformedMetadataAndGeometryAreNotEagerQueryFailures) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_GE(f.motion.size(),5u);
  f.motion[3].certified_affine=false;
  f.motion[4].parent=UINT32_MAX;
  f.base[2].vertices[0].x=std::numeric_limits<double>::quiet_NaN();
  f.pairs={{0,3},{0,1},{0,4},{0,UINT32_MAX},{0,2}};
  ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  // Preparation cannot preempt whatever the original nonlinear row0 will
  // report. Its untouched motion/bounds remain owned by that original fold.
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(f.motion[0],f.bounds[0],f.motion[3],f.bounds[3]),
      sct::PairMotionAction::UnsupportedRigidArc);
  f.Compare(1,f.adapter.PrismAt(1));
  const auto numeric=f.adapter.PrismAt(4);
  ASSERT_EQ(numeric.report.status,filters::Status::Ok);
  EXPECT_EQ(numeric.value.status,c::SelfContactFacetFilterStatus::InvalidInput);
  EXPECT_EQ(f.adapter.PrismAt(2).report.status,filters::Status::InvalidInput);
  ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  EXPECT_EQ(f.adapter.PrismAt(3).report.status,filters::Status::InvalidInput);
}
TEST(SelfContactFacetFilterAdapterCuda, EmptyAndNoLinearChunksIssueNoQueryAndDiscardRevokesMap) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());
  for(auto& motion:f.motion)motion.certified_affine=false;
  ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  const auto copies=facet_filter_cuda_probe::Copies();
  EXPECT_EQ(f.adapter.BeginCandidateChunk(nullptr,0).status,filters::Status::Ok);
  EXPECT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies);
  f.adapter.Discard();
  EXPECT_EQ(f.adapter.PrismAt(0).report.status,filters::Status::NoScene);
  for(auto& motion:f.motion)motion.certified_affine=true;
  ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  f.Compare(0,f.adapter.PrismAt(0));
}
TEST(SelfContactFacetFilterAdapterCuda, ChangedOriginalPairCannotConsumeAStalePackedSlotAndRetryIsValid) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  f.pairs[0]={0,2};
  EXPECT_EQ(f.adapter.PrismAt(0).report.status,filters::Status::InvalidInput);
  EXPECT_EQ(f.adapter.PrismAt(1).report.status,filters::Status::NoScene);
  ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  f.Compare(0,f.adapter.PrismAt(0));
}
TEST(SelfContactFacetFilterAdapterCuda, RejectedReplacementRevokesTheOldBorrowBeforeItCanExpire) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  f.Compare(0,f.adapter.PrismAt(0));
  EXPECT_EQ(f.adapter.BeginCandidateChunk(nullptr,1).status,filters::Status::InvalidInput);
  f.pairs.clear();f.pairs.shrink_to_fit();
  EXPECT_EQ(f.adapter.PrismAt(0).report.status,filters::Status::InvalidInput);
  // PrismAt rejection discards its scene; normal reentry can publish again.
  f.pairs={{0,1},{0,2}};
  ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  f.Compare(0,f.adapter.PrismAt(0));
}
TEST(SelfContactFacetFilterAdapterCuda, AliasedReplacementRevokesMappingBeforeAnyPackingOrTransfer) {
  using namespace facet_filter_adapter_test;
  NumericFixture f;ASSERT_TRUE(f.Geometry());ASSERT_TRUE(f.Initialize());ASSERT_TRUE(f.Candidate());
  ASSERT_EQ(f.adapter.BeginCandidateChunk(f.pairs.data(),f.pairs.size()).status,filters::Status::Ok);
  const auto copies=facet_filter_cuda_probe::Copies();
  const auto* aliased=reinterpret_cast<const c::FixedTrianglePair*>(&f.adapter);
  EXPECT_EQ(f.adapter.BeginCandidateChunk(aliased,1).status,filters::Status::InvalidInput);
  EXPECT_EQ(facet_filter_cuda_probe::Copies(),copies);
  EXPECT_EQ(f.adapter.PrismAt(0).report.status,filters::Status::InvalidInput);
  EXPECT_EQ(f.adapter.initialization_mode(),c::SelfContactFacetFilterInitialization::Cuda);
}
}  // namespace self_contact_transaction_cuda_test
