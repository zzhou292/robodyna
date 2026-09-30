#pragma once

#include "lib_src/collision/PlanarWallGeometry.h"
#include "lib_src/collision/Q4PlanarGeometry.h"
#include "lib_utest/q4_contact_integration_fixture.h"

#include <algorithm>
#include <array>
#include <vector>

namespace q4_planar_test {
namespace sc=tlfea::contact;
constexpr double Clearance=1e-6;
constexpr double Depth=1./32;
constexpr std::uint64_t LargeId=std::uint64_t{1}<<54;
struct Wall {
  std::vector<sc::PlanarWallVertex> vertices;
  std::vector<sc::PlanarWallTriangle> triangles;
  sc::PlanarWallView view() const {
    return {vertices.data(),static_cast<unsigned>(vertices.size()),triangles.data(),static_cast<unsigned>(triangles.size())};
  }
};
inline Wall GridWall(const std::vector<double>& coordinates,bool alternate=false,bool hole=false) {
  Wall wall; const auto n=static_cast<unsigned>(coordinates.size());
  for (unsigned y=0;y<n;++y) for (unsigned z=0;z<n;++z) {
    const auto node=n*y+z;
    wall.vertices.push_back({{0,coordinates[y],coordinates[z]},node+1,LargeId+node+1});
  }
  for (unsigned y=0;y+1<n;++y) for (unsigned z=0;z+1<n;++z) {
    if (hole && y == 1 && z == 1) continue;
    const unsigned a=n*y+z,b=a+1,c=a+n+1,d=a+n,parent=(n-1)*y+z+1;
    const unsigned faces[2][3]={{a,b,alternate?d:c},{alternate?b:a,c,d}};
    for (unsigned t=0;t<2;++t)
      wall.triangles.push_back({{faces[t][0],faces[t][1],faces[t][2]},LargeId+2*parent+t,parent,LargeId+parent});
  }
  return wall;
}
inline Wall Square(unsigned variant=0) {
  auto wall=GridWall(variant == 2 ? std::vector<double>{-2,0,2} : std::vector<double>{-2,2},variant == 1);
  if (variant == 3) std::reverse(wall.triangles.begin(),wall.triangles.end());
  return wall;
}
inline Wall Ring() { return GridWall({-2,-.4,.4,2},false,true); }

// C2's prescribed-view fixture permits arbitrary normal velocities. This C3
// fixture additionally honors the fixed-component motion contract.
struct Single : q4_contact_test::Fixture {
  Single() {
    for (unsigned n=0;n<4;++n) if (fixed[n] == 7) velocity[3*n]=0;
  }
};

struct Pair {
  std::array<double,18> x{},v{};
  std::array<double,6> inverse{{0,.5,.25,.125,1,.75}};
  std::array<std::uint8_t,6> fixed{{7,6,6,6,6,6}};
  std::array<sc::SurfaceQ4,2> parents{{{{2,0,1,3},LargeId+73,LargeId+42,2,0},
                                     {{4,2,3,5},LargeId+74,LargeId+43,2,0}}};
  Pair() {
    const double y[6]={-1,-1,0,0,1,1},z[6]={.5,-.5,.5,-.5,.5,-.5};
    const double depth[6]={-Depth,Depth,Depth,-Depth,Depth,-Depth};
    const double velocity[6]={0,.5,-1,2,.75,-.5};
    for (unsigned n=0;n<6;++n) {
      x[n]=depth[n]; x[n+6]=y[n]; x[n+12]=z[n]; v[3*n]=velocity[n];
    }
  }
  sc::Q4SurfaceView surface() const { return {{x.data(),6,1,6},{v.data(),6,3,1},parents.data(),2}; }
  sc::Q4FixedYZMassView mass() const { return {inverse.data(),fixed.data(),6,9}; }
};
struct Scratch {
  std::vector<sc::Q4IntegrationCell> leaves=std::vector<sc::Q4IntegrationCell>(sc::MaxQ4IntegrationLeaves);
  std::vector<std::uint32_t> heap=std::vector<std::uint32_t>(sc::MaxQ4IntegrationLeaves);
  sc::Q4IntegrationScratch view() { return {leaves.data(),heap.data(),sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves}; }
};
}  // namespace q4_planar_test
