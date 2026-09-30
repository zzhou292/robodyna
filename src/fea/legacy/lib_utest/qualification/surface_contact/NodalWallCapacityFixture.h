#pragma once
#include "NodalWallCapacitySource.h"

namespace nodal_wall_capacity_test {
namespace fe=tl::fea;
namespace detail=sc::nodal_wall_device_detail;
using Code=sc::NodalWallDeviceStatus;
constexpr double Step=1./1024;
constexpr std::uint64_t Qualification=0x4e57414354495631;
struct Fixture {
  Source source;
  std::vector<double> x,v,omega,q,mass,inertia,inverse,inverse_j;
  std::vector<std::uint8_t> fixed,rotation_fixed;
  std::vector<long double> area;
  sc::NodalWallWeights weights;
  q4_planar_test::Wall wall=q4_planar_test::Square(2);
  sc::PlanarWallBox motion{{0,-.75,-.75},{0,.75,.75}};
  bool Prepare();
  sc::VectorView Positions() const { return {x.data(),Nodes,3,1}; }
  sc::NodalWallDeviceConfig Config(fe::NodalStamp={}) const;
  bool Owner(fe::FENodalState&) const;
  bool Bind(fe::FENodalState&,sc::NodalWallContactDevice&) const;
};
struct Results {
  sc::NodalWallDiagnostics diagnostics;
  std::vector<sc::NodalWallParentResult> parents=std::vector<sc::NodalWallParentResult>(Parents);
  std::vector<sc::NodalWallPointResult> nodes=std::vector<sc::NodalWallPointResult>(Nodes);
  std::vector<std::uint64_t> faces=std::vector<std::uint64_t>(Nodes);
  sc::NodalWallDeviceResultView View() { return {&diagnostics,parents.data(),nodes.data(),faces.data(),Parents,Nodes}; }
};
} // namespace nodal_wall_capacity_test
