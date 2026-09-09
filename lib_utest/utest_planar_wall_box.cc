#include "lib_src/collision/PlanarWallBox.h"
#include "lib_utest/q4_planar_geometry_fixture.h"

#include <gtest/gtest.h>
#include <array>
#include <cstring>

namespace {
namespace sc=tlfea::contact;
namespace fixture=q4_planar_test;
using Code=sc::PlanarContactStatus;
using Mode=sc::PlanarWallBoxMode;
template<class T> auto Bytes(const T& object) {
  std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&object,sizeof(T)); return bytes;
}
void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void Same(sc::PlanarWallBox a,sc::PlanarWallBox b) { Same(a.minimum,b.minimum); Same(a.maximum,b.maximum); }
sc::PlanarContactReport Check(const sc::PlanarWallGeometry& wall,sc::PlanarWallBox box,
                            Mode mode,sc::PlanarWallBoxCoverage* result,double clearance=fixture::Clearance) {
  return sc::CheckPlanarWallBox(wall,box,clearance,fixture::LargeId+7,mode,result);
}
void ExpansionEncloses(const sc::PlanarWallBoxCoverage& result) {
  EXPECT_EQ(result.physical.minimum.x,result.query.minimum.x);
  EXPECT_EQ(result.physical.maximum.x,result.query.maximum.x);
  EXPECT_EQ(result.lower_expansion_upper.x,0); EXPECT_EQ(result.upper_expansion_upper.x,0);
  const double p0[2]={result.physical.minimum.y,result.physical.minimum.z};
  const double p1[2]={result.physical.maximum.y,result.physical.maximum.z};
  const double q0[2]={result.query.minimum.y,result.query.minimum.z};
  const double q1[2]={result.query.maximum.y,result.query.maximum.z};
  const double e0[2]={result.lower_expansion_upper.y,result.lower_expansion_upper.z};
  const double e1[2]={result.upper_expansion_upper.y,result.upper_expansion_upper.z};
  for (unsigned c=0;c<2;++c) {
    EXPECT_LE(q0[c],p0[c]); EXPECT_GE(q1[c],p1[c]); EXPECT_LT(q0[c],q1[c]);
    EXPECT_GE(static_cast<long double>(e0[c]),static_cast<long double>(p0[c])-q0[c]);
    EXPECT_GE(static_cast<long double>(e1[c]),static_cast<long double>(q1[c])-p1[c]);
  }
}

TEST(PlanarWallBox, ExactConstructionMatchesLegacyTrianglesAndMeshVariants) {
  const sc::PlanarWallBox physical{{0,-.7,-.25},{0,.375,.8}};
  for (unsigned variant=0;variant<4;++variant) {
    SCOPED_TRACE(variant);
    const auto mesh=fixture::Square(variant); sc::PlanarWallGeometry wall;
    ASSERT_EQ(wall.Initialize(mesh.view()).status,Code::Ok);
    sc::PlanarWallBoxCoverage result;
    ASSERT_EQ(Check(wall,physical,Mode::Exact,&result).status,Code::Ok);
    EXPECT_TRUE(result.covered); EXPECT_EQ(result.mode,Mode::Exact);
    Same(result.physical,physical); Same(result.query,physical);
    Same(result.lower_expansion_upper,{}); Same(result.upper_expansion_upper,{});
    // Direct existing classifier oracle with the frozen C5a corner/triangle
    // order and local geometry keys. These are never source node IDs.
    const auto lo=physical.minimum,hi=physical.maximum;
    const sc::Vec3 corners[4]={{lo.x,hi.y,hi.z},{lo.x,lo.y,hi.z},{lo.x,lo.y,lo.z},{lo.x,hi.y,lo.z}};
    const unsigned split[2][3]={{0,1,2},{0,2,3}};
    for (const auto& indices:split) {
      sc::TriangleGeometry triangle; triangle.face_id=fixture::LargeId+7;
      for (unsigned n=0;n<3;++n) { triangle.vertices[n]=corners[indices[n]]; triangle.vertex_ids[n]=indices[n]; }
      bool covered=false;
      ASSERT_EQ(wall.ClassifyTriangle(triangle,fixture::Clearance,&covered).status,Code::Ok);
      EXPECT_TRUE(covered);
    }
  }
}

TEST(PlanarWallBox, ExactlyEdgeOnLinesAndPointUseReportedQueryExpansion) {
  const sc::PlanarWallBox boxes[3]={{{0,0,-.5},{0,0,.5}},{{0,-.5,0},{0,.5,0}},{{0,.125,-.25},{0,.125,-.25}}};
  for (unsigned variant=0;variant<3;++variant) {
    const auto mesh=fixture::Square(variant); sc::PlanarWallGeometry wall;
    ASSERT_EQ(wall.Initialize(mesh.view()).status,Code::Ok);
    for (const auto& physical:boxes) {
      sc::PlanarWallBoxCoverage result;
      EXPECT_NE(Check(wall,physical,Mode::Exact,&result).status,Code::Ok);
      ASSERT_EQ(Check(wall,physical,Mode::ConservativeExpansion,&result).status,Code::Ok);
      EXPECT_TRUE(result.covered); Same(result.physical,physical); ExpansionEncloses(result);
      EXPECT_LT(result.lower_expansion_upper.y,fixture::Clearance);
      EXPECT_LT(result.lower_expansion_upper.z,fixture::Clearance);
    }
  }
}

TEST(PlanarWallBox, ExpansionLeavesWellConditionedBoundsExactAndEnclosesThinSpans) {
  const auto mesh=fixture::Square(); sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(mesh.view()).status,Code::Ok);
  const sc::PlanarWallBox regular{{0,-.5,-.25},{0,.5,.75}};
  sc::PlanarWallBoxCoverage result;
  ASSERT_EQ(Check(wall,regular,Mode::ConservativeExpansion,&result).status,Code::Ok);
  Same(result.query,regular); Same(result.lower_expansion_upper,{}); Same(result.upper_expansion_upper,{});
  const sc::PlanarWallBox thin{{0,.25,-.5},{0,::nextafter(.25,HUGE_VAL),.5}};
  ASSERT_EQ(Check(wall,thin,Mode::ConservativeExpansion,&result).status,Code::Ok);
  Same(result.physical,thin); ExpansionEncloses(result);
  EXPECT_GT(result.lower_expansion_upper.y,0); EXPECT_EQ(result.lower_expansion_upper.z,0);
}

TEST(PlanarWallBox, HolesAndExposedBoundaryRejectExpandedAndRegularQueries) {
  const auto ring=fixture::Ring(); sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(ring.view()).status,Code::Ok);
  sc::PlanarWallBoxCoverage result;
  const sc::PlanarWallBox safe{{0,.7,-.5},{0,1.2,.5}};
  ASSERT_EQ(Check(wall,safe,Mode::Exact,&result).status,Code::Ok);
  const auto before=Bytes(result);
  const sc::PlanarWallBox failures[5]={
    {{0,-.6,-.6},{0,.6,.6}}, // All corners covered; enclosed hole must still reject.
    {{0,0,-.1},{0,0,.1}},   // Exactly edge-on, wholly in the hole.
    {{0,.4000005,-.2},{0,.4000005,.2}}, // Inside the exposed clearance band.
    {{0,1.9999995,-.2},{0,1.9999995,.2}},
    {{0,3,-.2},{0,3,.2}}};
  for (const auto& physical:failures) {
    EXPECT_NE(Check(wall,physical,Mode::ConservativeExpansion,&result).status,Code::Ok);
    EXPECT_EQ(Bytes(result),before);
  }
  ASSERT_EQ(Check(wall,safe,Mode::Exact,&result).status,Code::Ok);
  EXPECT_EQ(result.query.maximum.y,safe.maximum.y);
}

TEST(PlanarWallBox, InvalidAndUnrepresentableQueriesPreserveCompleteOutputAndRetry) {
  const auto mesh=fixture::Square(); sc::PlanarWallGeometry wall,empty;
  ASSERT_EQ(wall.Initialize(mesh.view()).status,Code::Ok);
  const sc::PlanarWallBox safe{{0,-.5,-.5},{0,.5,.5}};
  sc::PlanarWallBoxCoverage result;
  ASSERT_EQ(Check(wall,safe,Mode::Exact,&result).status,Code::Ok);
  const auto before=Bytes(result);
  for (unsigned fault=0;fault<7;++fault) {
    auto box=safe; auto mode=Mode::ConservativeExpansion; double clearance=fixture::Clearance;
    switch (fault) {
      case 0: box.minimum.x=1; break;
      case 1: box.maximum.z=-1; break;
      case 2: box.maximum.y=HUGE_VAL; break;
      case 3: box.minimum.y=-DBL_MAX; box.maximum.y=DBL_MAX; break;
      case 4: mode=static_cast<Mode>(57); break;
      case 5: clearance=8*wall.tolerance(); break;
      case 6: box.minimum.z=::nan(""); break;
    }
    EXPECT_NE(Check(wall,box,mode,&result,clearance).status,Code::Ok);
    EXPECT_EQ(Bytes(result),before);
  }
  EXPECT_EQ(Check(empty,safe,Mode::Exact,&result).status,Code::NotInitialized);
  EXPECT_EQ(Bytes(result),before);
  EXPECT_EQ(Check(wall,safe,Mode::Exact,nullptr).status,Code::InvalidOutput);
  EXPECT_EQ(sc::CheckPlanarWallBox(wall,safe,fixture::Clearance,0,Mode::Exact,&result).status,Code::InvalidInput);
  EXPECT_EQ(Bytes(result),before);
  ASSERT_EQ(Check(wall,safe,Mode::Exact,&result).status,Code::Ok);
  Same(result.physical,safe);
}

TEST(PlanarWallBox, ShiftedWorldPlaneAndCoordinateScaleAreExplicit) {
  auto mesh=fixture::Square();
  for (auto& vertex:mesh.vertices) { vertex.position.x=3.25; vertex.position.y+=1e8; vertex.position.z-=1e8; }
  sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(mesh.view()).status,Code::Ok);
  const double clearance=64*wall.tolerance()+.001;
  const sc::PlanarWallBox line{{3.25,1e8,-1e8-.5},{3.25,1e8,-1e8+.5}};
  sc::PlanarWallBoxCoverage result;
  ASSERT_EQ(Check(wall,line,Mode::ConservativeExpansion,&result,clearance).status,Code::Ok);
  Same(result.physical,line); ExpansionEncloses(result);
  EXPECT_EQ(result.query.minimum.x,3.25);
  EXPECT_GT(result.lower_expansion_upper.y,1e-5);
  EXPECT_LT(result.lower_expansion_upper.y,.001);
}
} // namespace
