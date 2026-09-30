#include "lib_src/collision/PreparedPlanarWallQuery.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {
namespace sc=tlfea::contact;
using Face=sc::planar_detail::WallFace;
using Status=sc::Status;
constexpr double Tolerance=0x1p-40;
Face Triangle(double y,double z,double scale,std::uint64_t id) {
  return {{{{.5,y,z},{.5,y,z+scale},{.5,y+scale,z}}, {id*3,id*3+1,id*3+2},id},id+1000,id+2000};
}
std::vector<Face> Mesh(bool hole=false) {
  std::vector<Face> out;
  for (unsigned y=0;y<3;++y) for (unsigned z=0;z<3;++z) {
    if (hole && y==1 && z==1) continue;
    const auto id=100-2*(3*y+z);
    out.push_back(Triangle(y,z,1,id));
    Face second=Triangle(y,z,1,id-1);
    second.geometry.vertices[0]={.5,double(y+1),double(z+1)};
    second.geometry.vertices[1]={.5,double(y+1),double(z)};
    second.geometry.vertices[2]={.5,double(y),double(z+1)};
    out.push_back(second);
  }
  return out;
}
sc::TrianglePointGeometry Seed() {
  sc::TrianglePointGeometry point;
  point.point={11,12,13}; point.distance=17; point.weights[0]=.25; point.weights[1]=.5; point.weights[2]=.25;
  point.feature={sc::FeatureKind::kFace,123,456}; point.face_normal={1,2,3}; point.signed_plane_distance=-7;
  return point;
}
void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void Same(const sc::TrianglePointGeometry& a,const sc::TrianglePointGeometry& b) {
  Same(a.point,b.point); Same(a.face_normal,b.face_normal); EXPECT_EQ(a.distance,b.distance);
  EXPECT_EQ(a.signed_plane_distance,b.signed_plane_distance); EXPECT_EQ(a.degenerate,b.degenerate);
  for (unsigned i=0;i<3;++i) EXPECT_EQ(a.weights[i],b.weights[i]);
  EXPECT_EQ(a.feature.kind,b.feature.kind); EXPECT_EQ(a.feature.first,b.feature.first); EXPECT_EQ(a.feature.second,b.feature.second);
}
struct CheckResult { Status status; std::uint32_t owner; sc::PreparedWallQueryDiagnostics diagnostics; };
CheckResult Compare(const std::vector<Face>& faces,const sc::PreparedPlanarWallQuery& prepared,
                    sc::Vec3 query,double tolerance=Tolerance) {
  unsigned old_owner=73,new_owner=73;
  auto old_point=Seed(),new_point=old_point;
  sc::PreparedWallQueryDiagnostics diagnostics;
  const auto expected=sc::planar_detail::FindOwner(query,faces.data(),static_cast<unsigned>(faces.size()),
                                                 tolerance,&old_owner,&old_point);
  const auto actual=prepared.FindOwner(query,&new_owner,&new_point,&diagnostics);
  EXPECT_EQ(actual,expected); EXPECT_EQ(new_owner,old_owner); Same(new_point,old_point);
  return {actual,new_owner,diagnostics};
}
template<class T> auto Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&value,sizeof(T)); return bytes;
}

TEST(PreparedPlanarWallQuery, ExactInteriorSeamVertexAndOutsideParityWithRealPruning) {
  for (unsigned order=0;order<3;++order) {
    auto faces=Mesh();
    if (order==1) std::reverse(faces.begin(),faces.end());
    if (order==2) std::rotate(faces.begin(),faces.begin()+5,faces.end());
    sc::PreparedPlanarWallQuery prepared;
    ASSERT_EQ(prepared.Initialize(faces.data(),faces.size(),Tolerance),Status::kOk);
    ASSERT_TRUE(prepared.filter_eligible());
    for (int y=-1;y<=13;++y) for (int z=-1;z<=13;++z) {
      const auto result=Compare(faces,prepared,{.5,.25*y,.25*z});
      EXPECT_EQ(result.status,Status::kOk); EXPECT_TRUE(result.diagnostics.used_filter);
      EXPECT_GT(result.diagnostics.skipped_faces,0u);
    }
    const auto seam=Compare(faces,prepared,{.5,.5,.5});
    ASSERT_LT(seam.owner,faces.size()); EXPECT_EQ(faces[seam.owner].geometry.face_id,99u);
    const auto outside=Compare(faces,prepared,{.5,4,4});
    EXPECT_EQ(outside.owner,UINT32_MAX); EXPECT_EQ(outside.diagnostics.skipped_faces,faces.size());
  }
}

TEST(PreparedPlanarWallQuery, ClosedToleranceAndOneUlpNeighborhoodsNeverLoseCandidates) {
  const std::vector<Face> faces{Triangle(0,0,1,8),Triangle(4,4,1,3)};
  for (double tolerance:{0.,0x1p-40,0x1p-20}) {
    sc::PreparedPlanarWallQuery prepared;
    ASSERT_EQ(prepared.Initialize(faces.data(),faces.size(),tolerance),Status::kOk);
    ASSERT_TRUE(prepared.filter_eligible());
    // Exact edge/vertex, original tolerance boundary, and both neighboring
    // represented values. No acceptance is inferred from the enlarged box.
    for (double boundary:{0.,-tolerance,1.+tolerance})
      for (double y:{std::nextafter(boundary,-HUGE_VAL),boundary,std::nextafter(boundary,HUGE_VAL)})
        for (double z:{0.,.25}) {
          const auto result=Compare(faces,prepared,{.5,y,z},tolerance);
          EXPECT_TRUE(result.diagnostics.used_filter);
        }
    const auto far=Compare(faces,prepared,{.5,2.,2.},tolerance);
    EXPECT_EQ(far.owner,UINT32_MAX); EXPECT_EQ(far.diagnostics.skipped_faces,2u);
  }
}

TEST(PreparedPlanarWallQuery, TranslationsScalesAndHoleRetainOriginalPointAndFeature) {
  for (const auto scale_offset:std::array<std::array<double,2>,4>{{
      {{1,0}},{{0x1p-80,0}},{{0x1p8,0x1p48}},{{0x1p70,-0x1p75}}}}) {
    auto faces=Mesh(true); const double scale=scale_offset[0],offset=scale_offset[1];
    for (auto& face:faces) for (auto& p:face.geometry.vertices) { p.y=offset+scale*p.y; p.z=-offset+scale*p.z; }
    const double tolerance=scale*Tolerance;
    sc::PreparedPlanarWallQuery prepared;
    ASSERT_EQ(prepared.Initialize(faces.data(),faces.size(),tolerance),Status::kOk);
    ASSERT_TRUE(prepared.filter_eligible());
    for (double y:{0.,.25,1.5,2.5,3.5}) for (double z:{0.,.25,1.5,2.5,3.5})
      Compare(faces,prepared,{.5,offset+scale*y,-offset+scale*z},tolerance);
    const auto hole=Compare(faces,prepared,{.5,offset+scale*1.5,-offset+scale*1.5},tolerance);
    EXPECT_EQ(hole.owner,UINT32_MAX);
  }
}

TEST(PreparedPlanarWallQuery, IneligibleGeometryAndQueriesUseEntireOriginalScan) {
  for (unsigned variant=0;variant<7;++variant) {
    std::vector<Face> faces{Triangle(0,0,1,8)}; double tolerance=Tolerance;
    if (variant==0) faces[0]=Triangle(0,0,0x1p-110,8);
    if (variant==1) faces[0].geometry.vertices[1].z=2048*DBL_EPSILON; // Valid old skinny triangle.
    if (variant==2) faces[0].geometry.vertices[1].x=std::nextafter(.5,1.);
    if (variant==3) faces[0].geometry.vertices[2]=faces[0].geometry.vertices[1];
    if (variant==4) tolerance=-1;
    if (variant==5) tolerance=std::numeric_limits<double>::infinity();
    if (variant==6) tolerance=std::numeric_limits<double>::quiet_NaN();
    sc::PreparedPlanarWallQuery prepared;
    ASSERT_EQ(prepared.Initialize(faces.data(),faces.size(),tolerance),Status::kOk);
    EXPECT_FALSE(prepared.filter_eligible());
    const auto result=Compare(faces,prepared,{.5,.25,.25},tolerance);
    EXPECT_FALSE(result.diagnostics.used_filter);
  }
  const auto faces=Mesh(); sc::PreparedPlanarWallQuery prepared;
  ASSERT_EQ(prepared.Initialize(faces.data(),faces.size(),Tolerance),Status::kOk);
  for (const auto query:std::array<sc::Vec3,4>{{
      {std::nextafter(.5,1.),.25,.25},{.5,0x1p101,.25},
      {.5,DBL_MAX,DBL_MAX},{.5,std::numeric_limits<double>::quiet_NaN(),0}}}) {
    const auto result=Compare(faces,prepared,query);
    EXPECT_FALSE(result.diagnostics.used_filter);
  }
}

TEST(PreparedPlanarWallQuery, InvalidFarFacesAndFarArithmeticFailurePreserveEarlierCandidate) {
  for (unsigned variant=0;variant<3;++variant) {
    std::vector<Face> faces{Triangle(0,0,1,8),Triangle(10,10,1,3)};
    if (variant==0) faces[1].geometry.vertices[1].z=std::numeric_limits<double>::quiet_NaN();
    if (variant==1) faces[1].geometry.vertices[2]=faces[1].geometry.vertices[0];
    if (variant==2) {
      faces[1].geometry.vertices[0]={.5,-1e308,0};
      faces[1].geometry.vertices[1]={.5,1e308,0};
      faces[1].geometry.vertices[2]={.5,1e308,1};
    }
    sc::PreparedPlanarWallQuery prepared;
    ASSERT_EQ(prepared.Initialize(faces.data(),faces.size(),Tolerance),Status::kOk);
    EXPECT_FALSE(prepared.filter_eligible());
    const auto matched=Compare(faces,prepared,{.5,.25,.25});
    EXPECT_NE(matched.status,Status::kOk); EXPECT_EQ(matched.owner,0u);
    EXPECT_FALSE(matched.diagnostics.used_filter); EXPECT_EQ(matched.diagnostics.skipped_faces,0u);
    const auto unmatched=Compare(faces,prepared,{.5,-2,-2});
    EXPECT_NE(unmatched.status,Status::kOk); EXPECT_EQ(unmatched.owner,UINT32_MAX);
  }
}

TEST(PreparedPlanarWallQuery, CopiesCompleteSourceAndPreservesPacketOnArgumentFailureThenRetry) {
  auto faces=Mesh(); const auto original=faces; sc::PreparedPlanarWallQuery prepared;
  ASSERT_EQ(prepared.Initialize(faces.data(),faces.size(),Tolerance),Status::kOk);
  faces[0].geometry.vertices[0]={DBL_MAX,DBL_MAX,DBL_MAX};
  EXPECT_EQ(Compare(original,prepared,{.5,.25,.25}).status,Status::kOk);
  const auto before=Bytes(prepared);
  EXPECT_EQ(prepared.Initialize(nullptr,1,Tolerance),Status::kInvalidArgument);
  EXPECT_EQ(Bytes(prepared),before);
  EXPECT_EQ(prepared.Initialize(original.data(),0,Tolerance),Status::kInvalidArgument);
  EXPECT_EQ(Bytes(prepared),before);
  EXPECT_EQ(prepared.Initialize(original.data(),sc::MaxPlanarWallTriangles+1,Tolerance),Status::kInvalidArgument);
  EXPECT_EQ(Bytes(prepared),before);
  ASSERT_EQ(prepared.Initialize(original.data(),original.size(),Tolerance),Status::kOk);
  EXPECT_EQ(Compare(original,prepared,{.5,.5,.5}).status,Status::kOk);
  sc::PreparedPlanarWallQuery empty; unsigned owner=17; auto point=Seed(); const auto seed=point;
  EXPECT_EQ(empty.FindOwner({.5,0,0},&owner,&point),Status::kInvalidArgument);
  EXPECT_EQ(owner,UINT32_MAX); Same(point,seed);
  EXPECT_EQ(prepared.FindOwner({.5,0,0},nullptr,&point),Status::kInvalidArgument); Same(point,seed);
  EXPECT_EQ(prepared.FindOwner({.5,0,0},&owner,nullptr),Status::kInvalidArgument);
}
} // namespace
