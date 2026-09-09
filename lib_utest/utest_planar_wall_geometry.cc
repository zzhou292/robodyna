#include "lib_src/collision/PlanarWallGeometry.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {
namespace sc = tlfea::contact;
using Status = sc::PlanarContactStatus;
constexpr double kClearance = 1e-6;

struct Wall {
  std::vector<sc::PlanarWallVertex> vertices{
      {{.5,-2,-2},1,11},{{.5,-2,2},2,12},{{.5,2,2},3,13},{{.5,2,-2},4,14}};
  std::vector<sc::PlanarWallTriangle> triangles{{{0,1,2},12,21,31},{{0,2,3},8,21,31}};
  sc::PlanarWallView view() const {
    return {vertices.data(),static_cast<std::uint32_t>(vertices.size()),
            triangles.data(),static_cast<std::uint32_t>(triangles.size())};
  }
};
sc::TriangleGeometry Footprint(double y = 0, double z = 0, double span = .2) {
  return {{{.5,y-span,z-span},{.5,y-span,z+span},{.5,y+span,z}}, {101,102,103},71};
}
Wall Ring() {
  Wall wall; wall.vertices.clear(); wall.triangles.clear();
  const double coordinate[4] = {-2,-.4,.4,2};
  for (unsigned y = 0; y < 4; ++y) for (unsigned z = 0; z < 4; ++z) {
    const unsigned node = 4*y+z;
    wall.vertices.push_back({{.5,coordinate[y],coordinate[z]},node+1,node+101});
  }
  for (unsigned y = 0; y < 3; ++y) for (unsigned z = 0; z < 3; ++z) {
    if (y == 1 && z == 1) continue;
    const unsigned a=4*y+z, b=a+1, c=a+5, d=a+4, parent=3*y+z+1;
    wall.triangles.push_back({{a,b,c},2*parent,parent,parent+100});
    wall.triangles.push_back({{a,c,d},2*parent+1,parent,parent+100});
  }
  return wall;
}

TEST(PlanarWallGeometry, CopiesValidatedGeometryAndPreservesInitializedObject) {
  Wall input; sc::PlanarWallGeometry geometry;
  ASSERT_EQ(geometry.Initialize(input.view()).status,Status::Ok);
  ASSERT_EQ(geometry.faces().size(),2u);
  EXPECT_TRUE(geometry.initialized()); EXPECT_EQ(geometry.wall_x(),.5);
  EXPECT_GT(geometry.tolerance(),0);
  EXPECT_EQ(geometry.faces()[0].source_quad_id,21u);
  EXPECT_EQ(geometry.faces()[0].assembled_source_quad_id,31u);
  input.vertices[0].position.x=99;
  EXPECT_EQ(geometry.faces()[0].geometry.vertices[0].x,.5);
  EXPECT_EQ(geometry.Initialize(input.view()).status,Status::InvalidInput);
  bool covered=false;
  ASSERT_EQ(geometry.ClassifyTriangle(Footprint(),kClearance,&covered).status,Status::Ok);
  EXPECT_TRUE(covered); EXPECT_EQ(geometry.wall_x(),.5);
}

TEST(PlanarWallGeometry, CoverageAndSmallestIdSeamOwnershipIgnoreWallDiagonalAndOrder) {
  for (unsigned variant=0;variant<3;++variant) {
    Wall input;
    if (variant == 1) input.triangles={{{0,1,3},12,21,31},{{1,2,3},8,21,31}};
    if (variant == 2) std::reverse(input.triangles.begin(),input.triangles.end());
    sc::PlanarWallGeometry geometry;
    ASSERT_EQ(geometry.Initialize(input.view()).status,Status::Ok);
    bool covered=false;
    ASSERT_EQ(geometry.ClassifyTriangle(Footprint(),kClearance,&covered).status,Status::Ok);
    EXPECT_TRUE(covered);
    ASSERT_EQ(geometry.ClassifyTriangle(Footprint(3,0),kClearance,&covered).status,Status::Ok);
    EXPECT_FALSE(covered);
    std::uint32_t owner=UINT32_MAX; sc::TrianglePointGeometry point;
    ASSERT_EQ(sc::planar_detail::FindOwner({.5,0,0},geometry.faces().data(),2,
                                          geometry.tolerance(),&owner,&point),sc::Status::kOk);
    ASSERT_LT(owner,2u); EXPECT_EQ(geometry.faces()[owner].geometry.face_id,8u);
    EXPECT_EQ(point.distance,0);
  }
}

TEST(PlanarWallGeometry, RejectsBoundaryCrossingAndEnclosedHoleWithoutChangingCoverage) {
  sc::PlanarWallGeometry geometry; const auto ring=Ring();
  ASSERT_EQ(geometry.Initialize(ring.view()).status,Status::Ok);
  bool covered=true;
  ASSERT_EQ(geometry.ClassifyTriangle(Footprint(0,0,.1),kClearance,&covered).status,Status::Ok);
  EXPECT_FALSE(covered);
  for (const auto triangle : {Footprint(0,0,1.8),Footprint(-.4,0,.2),Footprint(2,0,.2)}) {
    covered=true;
    EXPECT_EQ(geometry.ClassifyTriangle(triangle,kClearance,&covered).status,Status::AmbiguousBoundary);
    EXPECT_TRUE(covered);
  }
  ASSERT_EQ(geometry.ClassifyTriangle(Footprint(-1,0,.1),kClearance,&covered).status,Status::Ok);
  EXPECT_TRUE(covered);
}

TEST(PlanarWallGeometry, InvalidTopologyAndGeometryCanRetryCleanly) {
  for (unsigned variant=0;variant<7;++variant) {
    Wall bad;
    if (variant == 0) bad.vertices[3].source_node_id=bad.vertices[0].source_node_id;
    if (variant == 1) bad.vertices[3].position=bad.vertices[0].position;
    if (variant == 2) bad.vertices[3].position.x=std::nextafter(.5,1.);
    if (variant == 3) std::swap(bad.triangles[1].nodes[1],bad.triangles[1].nodes[2]);
    if (variant == 4) bad.triangles.push_back(bad.triangles[0]);
    if (variant == 5) bad.triangles[1].assembled_source_quad_id=99;
    if (variant == 6) bad.triangles[1].nodes[2]=99;
    sc::PlanarWallGeometry geometry;
    EXPECT_NE(geometry.Initialize(bad.view()).status,Status::Ok) << variant;
    EXPECT_FALSE(geometry.initialized()); EXPECT_TRUE(geometry.faces().empty());
    const Wall clean;
    ASSERT_EQ(geometry.Initialize(clean.view()).status,Status::Ok);
    bool covered=false;
    ASSERT_EQ(geometry.ClassifyTriangle(Footprint(),kClearance,&covered).status,Status::Ok);
    EXPECT_TRUE(covered);
  }
}

TEST(PlanarWallGeometry, InvalidPublicInputsPreserveOutput) {
  sc::PlanarWallGeometry geometry; bool covered=true;
  EXPECT_EQ(geometry.ClassifyTriangle(Footprint(),kClearance,&covered).status,Status::NotInitialized);
  EXPECT_TRUE(covered);
  EXPECT_EQ(geometry.Initialize({}).status,Status::ResourceLimit);
  const Wall input;
  auto missing=input.view(); missing.vertices=nullptr;
  EXPECT_EQ(geometry.Initialize(missing).status,Status::InvalidInput);
  ASSERT_EQ(geometry.Initialize(input.view()).status,Status::Ok);
  for (double clearance : {0.,geometry.tolerance(),std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_EQ(geometry.ClassifyTriangle(Footprint(),clearance,&covered).status,Status::InvalidInput);
    EXPECT_TRUE(covered);
  }
  auto off_plane=Footprint(); off_plane.vertices[2].x=0;
  EXPECT_EQ(geometry.ClassifyTriangle(off_plane,kClearance,&covered).status,Status::UnsupportedGeometry);
  EXPECT_TRUE(covered);
  auto collapsed=Footprint(); collapsed.vertices[2]=collapsed.vertices[0];
  EXPECT_EQ(geometry.ClassifyTriangle(collapsed,kClearance,&covered).status,Status::UnsupportedGeometry);
  EXPECT_TRUE(covered);
  EXPECT_EQ(geometry.ClassifyTriangle(Footprint(),kClearance,nullptr).status,Status::InvalidInput);
}
}  // namespace
