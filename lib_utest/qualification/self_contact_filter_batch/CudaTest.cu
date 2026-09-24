// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Corpus.h"
#include "lib_src/collision/self_contact_filters/Batch.h"
#include <gtest/gtest.h>

namespace {
namespace c = tlfea::contact;
namespace f = c::self_contact_filters;

class SelfContactFilterBatchCuda : public ::testing::Test {
 protected:
  filter_batch_test::Corpus corpus = filter_batch_test::MakeCorpus();
  cudaStream_t stream = nullptr, other = nullptr;
  f::Batch batch;
  void SetUp() override {
    ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
    ASSERT_EQ(cudaStreamCreateWithFlags(&other, cudaStreamNonBlocking), cudaSuccess);
    f::Limits limits;
    limits.max_facets = corpus.accepted.size(); limits.max_pairs = corpus.pairs.size();
    ASSERT_EQ(batch.Initialize(limits, stream).status, f::Status::Ok);
    ASSERT_EQ(batch.Upload(corpus.scene(), stream).status, f::Status::Ok);
  }
  void TearDown() override {
    if (other) EXPECT_EQ(cudaStreamDestroy(other), cudaSuccess);
    if (stream) EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
  }
  void Check(f::PairView input, bool accepted, c::SelfContactFacetPrismAxisLimit axes) {
    const auto report = accepted ? batch.Accepted(input, stream) : batch.Linear(input, axes, stream);
    ASSERT_EQ(report.status, f::Status::Ok) << report.message;
    const auto values = batch.results();
    ASSERT_TRUE(values.complete); ASSERT_EQ(values.count, input.count);
    for (std::size_t i = 0; i < input.count; ++i) {
      SCOPED_TRACE(i);
      const auto expected = filter_batch_test::Reference(corpus, input.data[i], accepted, axes);
      EXPECT_EQ(values.data[i].status, expected.status);
      EXPECT_EQ(values.data[i].category, expected.category);
      EXPECT_EQ(values.data[i].separated, expected.separated);
      EXPECT_EQ(values.data[i].axis, expected.axis);
    }
  }
};

TEST_F(SelfContactFilterBatchCuda, CompleteCorpusMatchesCpuForAllAxesOrientationsAndReuse) {
  const f::PairResult* storage = nullptr;
  for (unsigned repetition = 0; repetition < 4; ++repetition) {
    for (unsigned limit = 0; limit < 4; ++limit) {
      const auto axes = static_cast<c::SelfContactFacetPrismAxisLimit>(limit);
      Check(corpus.input(), true, axes);
      Check(corpus.input(), false, axes);
      if (storage) EXPECT_EQ(storage, batch.results().data);
      storage = batch.results().data;
    }
    std::reverse(corpus.pairs.begin(), corpus.pairs.end());
    for (auto& pair : corpus.pairs) std::swap(pair.first, pair.second);
  }
}

TEST_F(SelfContactFilterBatchCuda, PartialEmptyAndRepeatedPairsRetainExactlyInputOrder) {
  for (std::size_t chunk : {std::size_t{1}, std::size_t{13}, corpus.pairs.size()})
    for (std::size_t offset = 0; offset < corpus.pairs.size(); offset += chunk)
      Check({corpus.pairs.data() + offset, std::min(chunk, corpus.pairs.size() - offset)},
            false, c::SelfContactFacetPrismAxisLimit::VertexVertex);
  const c::FixedTrianglePair duplicates[]{corpus.pairs[3], corpus.pairs[0], corpus.pairs[3]};
  Check({duplicates, 3}, true, c::SelfContactFacetPrismAxisLimit::VertexVertex);
  Check({}, false, c::SelfContactFacetPrismAxisLimit::FaceNormal);
}

TEST_F(SelfContactFilterBatchCuda, InvalidPairAndBorrowedAliasRevokePublicationBeforeDeviceRead) {
  Check(corpus.input(), true, c::SelfContactFacetPrismAxisLimit::VertexVertex);
  const auto prior = batch.results();
  const c::FixedTrianglePair malformed[]{corpus.pairs[0], {0, UINT32_MAX}, {0, 0}};
  const auto report = batch.Accepted({malformed, 3}, stream);
  EXPECT_EQ(report.status, f::Status::InvalidInput); EXPECT_EQ(report.pair, 1u);
  EXPECT_FALSE(batch.results().complete);
  EXPECT_EQ(batch.Accepted({nullptr, 1}, stream).status, f::Status::InvalidInput);
  EXPECT_EQ(batch.Accepted({corpus.pairs.data(), SIZE_MAX}, stream).status, f::Status::InvalidInput);
  EXPECT_EQ(batch.Accepted({reinterpret_cast<const c::FixedTrianglePair*>(prior.data), 1}, stream).status,
            f::Status::InvalidInput);
  EXPECT_EQ(batch.Linear(corpus.input(), static_cast<c::SelfContactFacetPrismAxisLimit>(255), stream).status,
            f::Status::InvalidInput);
  EXPECT_EQ(batch.Accepted(corpus.input(), other).status, f::Status::InvalidInput);
  Check(corpus.input(), true, c::SelfContactFacetPrismAxisLimit::VertexVertex);
  EXPECT_EQ(prior.data, batch.results().data);
  EXPECT_EQ(prior.scene_generation, batch.results().scene_generation);
}

TEST_F(SelfContactFilterBatchCuda, SceneReplacementAndInvalidUploadCannotReuseAnOldScene) {
  Check(corpus.input(), false, c::SelfContactFacetPrismAxisLimit::VertexVertex);
  const auto old = batch.results();
  EXPECT_EQ(batch.Upload({}, stream).status, f::Status::InvalidInput);
  EXPECT_FALSE(batch.results().complete);
  EXPECT_EQ(batch.Accepted(corpus.input(), stream).status, f::Status::NoScene);
  auto alias = corpus.scene();
  alias.accepted = reinterpret_cast<const f::TriangleGeometry*>(old.data);
  EXPECT_EQ(batch.Upload(alias, stream).status, f::Status::InvalidInput);
  EXPECT_EQ(batch.Upload(corpus.scene(), other).status, f::Status::InvalidInput);
  for (auto& point : corpus.prepared[0].vertices) point.z += .25;
  ASSERT_EQ(batch.Upload(corpus.scene(), stream).status, f::Status::Ok);
  Check(corpus.input(), false, c::SelfContactFacetPrismAxisLimit::VertexVertex);
  EXPECT_EQ(batch.results().scene_generation, old.scene_generation + 1);
  EXPECT_EQ(batch.results().data, old.data);
}

TEST(SelfContactFilterBatchAdmission, ForecastCapsAndExplicitStreamPrecedeAllocation) {
  f::Batch batch;
  f::Limits limits;
  const auto forecast = f::Batch::PreflightLimits(limits);
  ASSERT_EQ(forecast.report.status, f::Status::Ok);
  EXPECT_EQ(batch.Initialize(limits, nullptr).status, f::Status::InvalidInput);
  EXPECT_EQ(batch.Initialize(limits, cudaStreamLegacy).status, f::Status::InvalidInput);
  EXPECT_EQ(batch.Initialize(limits, cudaStreamPerThread).status, f::Status::InvalidInput);
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
  limits.max_device_bytes = forecast.forecast.device_bytes - 1;
  EXPECT_EQ(batch.Initialize(limits, stream).status, f::Status::ResourceLimit);
  limits.max_device_bytes++;
  limits.max_host_bytes = forecast.forecast.startup_host_bytes - 1;
  EXPECT_EQ(batch.Initialize(limits, stream).status, f::Status::ResourceLimit);
  limits.max_host_bytes++;
  ASSERT_EQ(batch.Initialize(limits, stream).status, f::Status::Ok);
  EXPECT_EQ(batch.forecast().device_bytes, forecast.forecast.device_bytes);
  EXPECT_EQ(batch.Initialize(limits, stream).status, f::Status::AlreadyInitialized);
  EXPECT_EQ(batch.Accepted({}, stream).status, f::Status::NoScene);
  EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}
}  // namespace

#include "lib_src/collision/self_contact_filters/Environment.h"
namespace {
TEST_F(SelfContactFilterBatchCuda, NoncanonicalHostEnvironmentRejectsWithoutChangingIt) {
  fenv_t saved;
  ASSERT_EQ(std::fegetenv(&saved), 0);
#if defined(__x86_64__) || defined(__i386__)
  const auto csr = _mm_getcsr();
#endif
  // No ASSERT after changing the environment: restoration must always happen.
  const int changed = std::fesetround(FE_UPWARD);
  EXPECT_EQ(changed, 0);
  if (changed == 0) {
    EXPECT_EQ(batch.Linear(corpus.input(), c::SelfContactFacetPrismAxisLimit::VertexVertex, stream).status,
              f::Status::UnsupportedEnvironment);
    EXPECT_EQ(std::fegetround(), FE_UPWARD);
    EXPECT_FALSE(batch.results().complete);
  }
  EXPECT_EQ(std::fesetenv(&saved), 0);
#if defined(__x86_64__) || defined(__i386__)
  for (unsigned bit : {1u << 15, 1u << 6}) {
    _mm_setcsr(csr | bit);
    EXPECT_EQ(batch.Accepted(corpus.input(), stream).status, f::Status::UnsupportedEnvironment);
    EXPECT_EQ(_mm_getcsr(), csr | bit);
    _mm_setcsr(csr);
  }
  // Do not invoke the test framework while a hardware exception is unmasked.
  _mm_setcsr((csr & ~0x3fu) & ~(1u << 7));
  const auto trapped = batch.Accepted(corpus.input(), stream).status;
  const auto observed_csr = _mm_getcsr();
  _mm_setcsr(csr);
  EXPECT_EQ(trapped, f::Status::UnsupportedEnvironment);
  EXPECT_EQ(observed_csr, (csr & ~0x3fu) & ~(1u << 7));
#endif
  Check(corpus.input(), true, c::SelfContactFacetPrismAxisLimit::VertexVertex);
}
}  // namespace
