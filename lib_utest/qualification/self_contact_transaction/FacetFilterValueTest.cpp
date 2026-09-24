// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include <gtest/gtest.h>
#include <array>
namespace {
namespace c = tlfea::contact;
namespace sct = c::self_contact_transaction;
namespace f = c::self_contact_filters;
TEST(FacetFilterIntegrationValues, OptionalBackendIsDisabledByDefault) {
  EXPECT_FALSE(c::SelfContactTransactionConfig{}.enable_cuda_facet_filters);
}
TEST(FacetFilterIntegrationValues, ForecastIncludesOneCompactSceneAndOneBoundedBatch) {
  constexpr std::size_t facets=17,pairs=5,cap=1u<<20;
  static_assert(sizeof(f::TriangleGeometry)==72 && sizeof(f::FacetProperties)==16);
  static_assert(sizeof(f::PairResult)==4 && sizeof(c::FixedTrianglePair)==8);
  const auto forecast=sct::FacetFilters::Preflight(facets,pairs,cap,cap);
  ASSERT_EQ(forecast.report.status,f::Status::Ok);
  EXPECT_EQ(forecast.scene.bytes,160*facets);
  EXPECT_EQ(forecast.device_bytes,160*facets+12*pairs);
  EXPECT_EQ(forecast.device_allocations,1u);
  EXPECT_EQ(forecast.owned_host_bytes,sizeof(sct::FacetFilters)+forecast.scene.bytes+
      forecast.batch.owned_host_bytes-sizeof(f::Batch));
  EXPECT_GT(forecast.startup_host_bytes,forecast.owned_host_bytes);
  EXPECT_EQ(sct::FacetFilters::Preflight(facets,pairs,forecast.startup_host_bytes,
      forecast.device_bytes).report.status,f::Status::Ok);
  EXPECT_EQ(sct::FacetFilters::Preflight(facets,pairs,forecast.startup_host_bytes-1,
      forecast.device_bytes).report.status,f::Status::ResourceLimit);
  EXPECT_EQ(sct::FacetFilters::Preflight(facets,pairs,cap,forecast.device_bytes-1).report.status,
      f::Status::ResourceLimit);
}
TEST(FacetFilterIntegrationValues, LinearSpanStopsBeforeNonlinearAndInvalidParentRows) {
  std::array<sct::MotionSupport,5> motion;
  std::array<c::SelfContactSweptParentBounds,5> bounds;
  for(unsigned i=0;i<motion.size();++i) {
    motion[i].parent=i;motion[i].certified_affine=true;
    bounds[i]={{-1,-1,-1},{1,1,1}};
  }
  motion[3].certified_affine=false;
  const c::FixedTrianglePair pairs[]{{0,1},{0,2},{0,3},{1,2},{0,4},{0,1}};
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,6,0,motion.data(),bounds.data(),5,5),2u);
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,6,2,motion.data(),bounds.data(),5,5),2u);
  motion[4].parent=99;
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,6,3,motion.data(),bounds.data(),5,5),4u);
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,6,4,motion.data(),bounds.data(),5,5),4u);
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,6,5,motion.data(),bounds.data(),5,5),6u);
}
TEST(FacetFilterIntegrationValues, LinearSpanLeavesExclusionsAndMalformedOrdinalsAtTheirOriginalPosition) {
  std::array<sct::MotionSupport,4> motion;
  std::array<c::SelfContactSweptParentBounds,4> bounds;
  for(unsigned i=0;i<motion.size();++i) {
    motion[i].parent=i;motion[i].certified_affine=true;bounds[i]={{-1,-1,-1},{1,1,1}};
  }
  const c::FixedTrianglePair pairs[]{{0,1},{2,3},{0,UINT32_MAX},{0,1}};
  motion[2].motion=motion[3].motion=c::SelfContactFacetMotion::CompleteRigidGroup;
  motion[2].complete_rigid_group=motion[3].complete_rigid_group=7;
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,4,0,motion.data(),bounds.data(),4,4),1u);
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,4,1,motion.data(),bounds.data(),4,4),1u);
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,4,2,motion.data(),bounds.data(),4,4),2u);
  EXPECT_EQ(sct::LinearFacetSpanEnd(pairs,4,3,motion.data(),bounds.data(),4,4),4u);
  EXPECT_EQ(pairs[2].second,UINT32_MAX); // Lookahead does not mutate or compact.
}
}  // namespace
