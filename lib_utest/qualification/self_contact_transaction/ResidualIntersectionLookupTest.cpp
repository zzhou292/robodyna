// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_transaction/ResidualIntersectionLookup.h"
#include "SortedIntersectionsTestAccess.h"
#include "../fixed_triangle_features/Fixture.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>
#include <vector>

namespace {
namespace c = tlfea::contact;
namespace sct = c::self_contact_transaction;
namespace geometry = fixed_triangle_test;
using Status = sct::LinearResidualSeparationStatus;
using Result = sct::LinearResidualSeparationResult;

std::uint64_t Bits(double value) {
  std::uint64_t result;
  std::memcpy(&result,&value,sizeof(result));
  return result;
}
void Same(const Result& actual,const Result& expected) {
  EXPECT_EQ(actual.status,expected.status);
  EXPECT_EQ(actual.exact_common_translation,expected.exact_common_translation);
  const std::array<double,7> a{{actual.reference_translation.x,actual.reference_translation.y,
      actual.reference_translation.z,actual.first_residual_upper_m,
      actual.second_residual_upper_m,actual.prepared_distance_lower_m,actual.strict_gap_lower_m}};
  const std::array<double,7> b{{expected.reference_translation.x,expected.reference_translation.y,
      expected.reference_translation.z,expected.first_residual_upper_m,
      expected.second_residual_upper_m,expected.prepared_distance_lower_m,expected.strict_gap_lower_m}};
  for(std::size_t i=0;i<a.size();++i) EXPECT_EQ(Bits(a[i]),Bits(b[i]))<<i;
}
struct Input {
  c::CurrentFixedTriangle first_base,first_prepared,second_base,second_prepared;
  double first_half=.125,second_half=.125,duration=1;
  sct::FacetQuadraticCoefficients first_curvature,second_curvature;
  c::FixedTriangleFeatureView features;
  c::FixedTriangleIntersectionView intersections;
};
std::array<Result,2> Compare(const Input& in,const sct::SortedIntersections& index) {
  const auto linear=sct::CertifyLinearResidualSeparation(
      in.first_base,in.first_prepared,in.first_half,in.second_base,in.second_prepared,
      in.second_half,in.features,in.intersections);
  const auto indexed=sct::CertifyLinearResidualSeparation(
      in.first_base,in.first_prepared,in.first_half,in.second_base,in.second_prepared,
      in.second_half,in.features,in.intersections,index);
  Same(indexed,linear);
  const auto quadratic=sct::CertifyQuadraticResidualSeparation(
      in.first_base,in.first_prepared,in.first_curvature,in.first_half,
      in.second_base,in.second_prepared,in.second_curvature,in.second_half,
      in.duration,in.features,in.intersections);
  const auto indexed_quadratic=sct::CertifyQuadraticResidualSeparation(
      in.first_base,in.first_prepared,in.first_curvature,in.first_half,
      in.second_base,in.second_prepared,in.second_curvature,in.second_half,
      in.duration,in.features,in.intersections,index);
  Same(indexed_quadratic,quadratic);
  return {indexed,indexed_quadratic};
}
struct Scene {
  c::FixedTriangleFeatureDiscovery discovery;
  Input input;
  explicit Scene(double gap=1,std::uint64_t eid=10) {
    const c::Vec3 a[]{{0,0,0},{2,0,0},{0,2,0}};
    const c::Vec3 b[]{{0,0,gap},{2,0,gap},{0,2,gap}};
    const std::uint64_t first_ids[]{1,2,3},second_ids[]{4,5,6};
    input.first_base=geometry::Triangle(eid,0,a,first_ids,17);
    input.second_base=geometry::Triangle(eid+10,0,b,second_ids,17);
    input.first_prepared=input.first_base;input.second_prepared=input.second_base;
    input.first_curvature.complete=input.second_curvature.complete=true;
    geometry::Initialize(&discovery,geometry::Limits(4));
    Discover();
  }
  void Discover() {
    const c::CurrentFixedTriangle triangles[]{input.first_prepared,input.second_prepared};
    const c::FixedTrianglePair pair{0,1};
    const auto report=discovery.Discover(triangles,2,&pair,1);
    EXPECT_EQ(report.status,c::FixedTriangleDiscoveryStatus::Ok);
    input.features=discovery.features();input.intersections=discovery.intersections();
  }
  c::FixedTriangleIntersection Target() const {
    c::FixedTriangleIntersection row;
    row.triangles[0]=input.first_prepared.key;row.triangles[1]=input.second_prepared.key;
    return row;
  }
};
c::FixedTriangleIntersection Other(std::uint64_t first) {
  c::FixedTriangleIntersection row;
  row.triangles[0]={17,first,0,0};row.triangles[1]={17,first+1,0,0};
  return row;
}
void Translate(c::CurrentFixedTriangle& triangle,c::Vec3 value) {
  for(auto& point:triangle.vertices) {
    point.x+=value.x;point.y+=value.y;point.z+=value.z;
  }
}
Input Reverse(Input in) {
  std::swap(in.first_base,in.second_base);std::swap(in.first_prepared,in.second_prepared);
  std::swap(in.first_half,in.second_half);std::swap(in.first_curvature,in.second_curvature);
  return in;
}

TEST(SelfContactResidualIntersectionLookup, OrderedFirstLastAndMissingKeysKeepAllLinearAndQuadraticBits) {
  Scene scene(1,128);
  ASSERT_EQ(scene.input.features.count,15u);
  for(unsigned placement=0;placement<3;++placement) {
    SCOPED_TRACE(placement);
    std::vector<c::FixedTriangleIntersection> rows;
    if(placement==0)rows.push_back(scene.Target());
    for(unsigned i=0;i<63;++i)rows.push_back(Other(placement==0?150+2*i:1+2*i));
    if(placement==1)rows.push_back(scene.Target());
    if(placement==2)rows.push_back(Other(256));
    ASSERT_EQ(rows.size(),64u);
    auto input=scene.input;input.intersections={rows.data(),rows.size(),true};
    for(auto& vertex:input.second_curvature.q)vertex[2]={.0625,.0625};
    const auto index=sct::SortedIntersectionsTestAccess::FromRaw(input.intersections);
    ASSERT_TRUE(index.ordered());ASSERT_TRUE(index.matches(input.intersections));
    c::RepresentedIntervalPairKey key;
    const auto target=scene.Target();
    for(unsigned side=0;side<2;++side) {
      const auto& source=target.triangles[side];
      key.paths[side]={source.source_instance_id,source.parent_eid,source.level,source.local_facet};
    }
    const auto lookup=sct::CompareIntersectionLookup(input.intersections,key,index);
    const std::size_t expected=placement==0?0:placement==1?63:SIZE_MAX;
    EXPECT_EQ(lookup.original,expected);EXPECT_EQ(lookup.indexed,expected);
    EXPECT_EQ(lookup.original_counts.rows,placement==0?1u:64u);
    EXPECT_LE(lookup.indexed_counts.rows,8u);
    EXPECT_FALSE(lookup.original_counts.saturated || lookup.indexed_counts.saturated);
    if(placement!=0)EXPECT_LT(lookup.indexed_counts.rows,lookup.original_counts.rows);
    for(bool reversed:{false,true}) {
      const auto results=Compare(reversed?Reverse(input):input,index);
      for(const auto& result:results)
        EXPECT_EQ(result.status,placement<2?Status::PotentialContact:Status::CertifiedSeparated);
    }
  }
}

TEST(SelfContactResidualIntersectionLookup, ForeignUnsortedDuplicateAndReversedViewsRetainRawFallback) {
  Scene scene;
  const std::array<c::FixedTriangleIntersection,3> original{{Other(1),scene.Target(),Other(30)}};
  const c::FixedTriangleIntersectionView retained{original.data(),original.size(),true};
  const auto retained_index=sct::SortedIntersectionsTestAccess::FromRaw(retained);
  for(unsigned mutation=0;mutation<5;++mutation) {
    SCOPED_TRACE(mutation);auto rows=original;
    if(mutation==0)std::swap(rows[0],rows[2]);
    if(mutation==1)rows[2]=rows[1];
    if(mutation==2)std::swap(rows[1].triangles[0],rows[1].triangles[1]);
    if(mutation==3)rows[1]=Other(5);
    if(mutation==4)++rows[1].triangles[1].source_instance_id;
    auto input=scene.input;input.intersections={rows.data(),rows.size(),true};
    const auto local=sct::SortedIntersectionsTestAccess::FromRaw(input.intersections);
    if(mutation<3)EXPECT_FALSE(local.ordered());
    EXPECT_FALSE(retained_index.matches(input.intersections));
    const auto a=Compare(input,local),b=Compare(input,retained_index);
    for(unsigned i=0;i<2;++i) {
      Same(a[i],b[i]);
      EXPECT_EQ(a[i].status,mutation<3?Status::PotentialContact:Status::CertifiedSeparated);
    }
  }
}

TEST(SelfContactResidualIntersectionLookup, MalformedCommonInputsRejectBeforeARealMatchingIntersection) {
  Scene scene;
  const auto match=scene.Target();
  const c::FixedTriangleIntersectionView view{&match,1,true};
  const auto index=sct::SortedIntersectionsTestAccess::FromRaw(view);
  for(unsigned mutation=0;mutation<9;++mutation) {
    SCOPED_TRACE(mutation);auto input=scene.input;input.intersections=view;
    if(mutation==0)++input.first_base.key.parent_eid;
    if(mutation==1)++input.second_base.key.source_instance_id;
    if(mutation==2)input.features.complete=false;
    if(mutation==3)input.features={nullptr,1,true};
    if(mutation==4)input.intersections.complete=false;
    if(mutation==5)input.intersections={nullptr,1,true};
    if(mutation==6)input.first_half=0;
    if(mutation==7)input.second_half=std::numeric_limits<double>::quiet_NaN();
    if(mutation==8)input.first_prepared.vertices[0].x=std::numeric_limits<double>::infinity();
    for(const auto& result:Compare(input,index))EXPECT_EQ(result.status,Status::InvalidInput);
  }
  // Presence still precedes the old feature-roster completeness/geometry walk.
  // The optimization must not eagerly move that later validation earlier.
  auto input=scene.input;input.intersections=view;input.features={nullptr,0,true};
  for(const auto& result:Compare(input,index))EXPECT_EQ(result.status,Status::PotentialContact);

  // Quadratic metadata is deliberately checked only after a separated linear
  // result. Presence must preserve this first-result/error ordering.
  for(unsigned mutation=0;mutation<4;++mutation) {
    SCOPED_TRACE(mutation);auto malformed=scene.input;
    if(mutation==0)malformed.first_curvature.complete=false;
    if(mutation==1)malformed.second_curvature.q[1][2]={0,std::numeric_limits<double>::quiet_NaN()};
    if(mutation==2)malformed.duration=-1;
    if(mutation==3)malformed.duration=std::numeric_limits<double>::infinity();
    malformed.intersections=view;
    for(const auto& result:Compare(malformed,index))
      EXPECT_EQ(result.status,Status::PotentialContact);
    malformed.intersections=scene.input.intersections;
    const auto absent_index=sct::SortedIntersectionsTestAccess::FromRaw(malformed.intersections);
    const auto absent=Compare(malformed,absent_index);
    EXPECT_EQ(absent[0].status,Status::CertifiedSeparated);
    EXPECT_EQ(absent[1].status,Status::InvalidInput);
  }
}

TEST(SelfContactResidualIntersectionLookup, NativeDiscoveryContactAndSeparatedCohortsReusePublisherOnlyAsOrderingProof) {
  for(double gap:{0.,1.}) {
    Scene scene(gap);
    ASSERT_TRUE(scene.input.features.complete && scene.input.intersections.complete);
    ASSERT_EQ(scene.input.intersections.count,gap==0?1u:0u);
    const auto index=sct::SortedIntersectionsTestAccess::FromPublisher(scene.discovery);
    ASSERT_TRUE(index.ordered());ASSERT_TRUE(index.matches(scene.input.intersections));
    for(const auto& result:Compare(scene.input,index))
      EXPECT_EQ(result.status,gap==0?Status::PotentialContact:Status::CertifiedSeparated);
  }
}

TEST(SelfContactResidualIntersectionLookup, SignedZeroHighKeysRelativeMotionAndCurvaturePreserveEveryBit) {
  Scene scene(1,(std::uint64_t{1}<<63)+10);
  scene.input.first_prepared.vertices[0].x=-0.;
  scene.input.first_prepared.vertices[0].y=-0.;
  scene.Discover();
  {
    const auto index=sct::SortedIntersectionsTestAccess::FromPublisher(scene.discovery);
    const auto results=Compare(scene.input,index);
    EXPECT_EQ(Bits(results[0].reference_translation.x),Bits(-0.));
    EXPECT_EQ(results[0].status,Status::CertifiedSeparated);
  }
  scene.input.first_prepared=scene.input.first_base;
  scene.input.second_prepared=scene.input.second_base;
  Translate(scene.input.first_prepared,{.125,-.25,.5});
  Translate(scene.input.second_prepared,{.15625,-.25,.5});
  scene.Discover();
  const auto index=sct::SortedIntersectionsTestAccess::FromPublisher(scene.discovery);
  for(double curvature:{0.,.0625,16.}) {
    auto input=scene.input;
    for(auto& vertex:input.second_curvature.q)vertex[2]={curvature,curvature};
    const auto results=Compare(input,index);
    EXPECT_EQ(results[0].status,Status::CertifiedSeparated);
    EXPECT_EQ(results[1].status,curvature==16?Status::PotentialContact:Status::CertifiedSeparated);
    EXPECT_FALSE(results[0].exact_common_translation);
  }
}

TEST(SelfContactResidualIntersectionLookup, MissingDuplicateAndNonfiniteFeatureErrorsStayAfterNegativeLookup) {
  Scene scene;
  ASSERT_EQ(scene.input.features.count,15u);
  const auto index=sct::SortedIntersectionsTestAccess::FromPublisher(scene.discovery);
  const std::vector<c::FixedTriangleFeatureCandidate> original(
      scene.input.features.data,scene.input.features.data+scene.input.features.count);
  for(unsigned mutation=0;mutation<4;++mutation) {
    auto features=original;auto input=scene.input;
    if(mutation==0)features.pop_back();
    if(mutation==1)features[1]=features[0];
    if(mutation==2)features[0].distance_m=std::numeric_limits<double>::quiet_NaN();
    if(mutation==3)features[0].representation_error_m=-1;
    input.features={features.data(),features.size(),true};
    for(const auto& result:Compare(input,index))EXPECT_EQ(result.status,Status::IncompleteFeatureRoster);
  }
}
}  // namespace
