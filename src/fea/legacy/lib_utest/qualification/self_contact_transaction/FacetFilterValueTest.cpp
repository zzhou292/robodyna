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
  EXPECT_EQ(forecast.storage.bytes,160*facets+12*pairs);
  EXPECT_EQ(forecast.device_bytes,160*facets+12*pairs);
  EXPECT_EQ(forecast.device_allocations,1u);
  EXPECT_EQ(forecast.owned_host_bytes,sizeof(sct::FacetFilters)+forecast.storage.bytes+
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
TEST(FacetFilterIntegrationValues, CompactPackingLeavesNonlinearExclusionsAndMalformedRowsAtTheirOrdinals) {
  std::array<sct::MotionSupport,5> motion;
  std::array<c::SelfContactSweptParentBounds,5> bounds;
  for (unsigned i=0;i<motion.size();++i) {
    motion[i].parent=i;motion[i].certified_affine=true;
    bounds[i]={{-1,-1,-1},{1,1,1}};
  }
  motion[3].certified_affine=false;
  motion[2].motion=motion[3].motion=c::SelfContactFacetMotion::CompleteRigidGroup;
  motion[2].complete_rigid_group=motion[3].complete_rigid_group=7;
  motion[4].parent=99;
  const c::FixedTrianglePair pairs[]{{0,1},{0,3},{2,3},{0,2},{0,4},{0,UINT32_MAX},{0,1},{1,0}};
  std::array<c::FixedTrianglePair,8> packed{};
  std::array<std::uint32_t,8> slots{};
  ASSERT_EQ(sct::PackCandidateLinearPairs(pairs,8,motion.data(),bounds.data(),5,5,
      packed.data(),slots.data()),4u);
  const std::array<std::uint32_t,8> expected{{0,UINT32_MAX,UINT32_MAX,1,UINT32_MAX,UINT32_MAX,2,3}};
  EXPECT_EQ(slots,expected);
  for (unsigned i:{0u,3u,6u,7u}) {
    EXPECT_EQ(packed[slots[i]].first,pairs[i].first);
    EXPECT_EQ(packed[slots[i]].second,pairs[i].second);
  }
  EXPECT_EQ(pairs[5].second,UINT32_MAX);
  EXPECT_EQ(motion[4].parent,99u);
  EXPECT_FALSE(motion[3].certified_affine);
}
TEST(FacetFilterIntegrationValues, EmptyAndInvalidPackingAdmissionDoNotTouchCallerStorage) {
  c::FixedTrianglePair packed{11,13};
  std::uint32_t slot=17;
  EXPECT_EQ(sct::PackCandidateLinearPairs(nullptr,0,nullptr,nullptr,0,0,&packed,&slot),0u);
  EXPECT_EQ(sct::PackCandidateLinearPairs(nullptr,1,nullptr,nullptr,0,0,&packed,&slot),SIZE_MAX);
  EXPECT_EQ(packed.first,11u);EXPECT_EQ(packed.second,13u);EXPECT_EQ(slot,17u);
}
}  // namespace
