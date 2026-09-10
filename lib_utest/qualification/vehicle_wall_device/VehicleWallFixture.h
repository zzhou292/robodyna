#pragma once
#include "../surface_contact/NodalWallOwnerFixture.h"
#include "../nodal_vehicle/VehicleOwnerFixture.h"
#include "../vehicle_shell_host/VehicleShellFixture.h"
#include <vector>
namespace vehicle_wall_device_test {
namespace sc=tlfea::contact;namespace fe=tl::fea;namespace detail=sc::nodal_wall_device_detail;
using Code=sc::NodalWallDeviceStatus;
using CudaTest=fe::vehicle_test::VehicleOwnerCuda;
constexpr double H=1./1024;
constexpr std::uint64_t Qualification=0x565748445145;
struct Fixture {
  const std::size_t nq,nt,n;
  std::vector<double> x,v,w,q,inverse,inverse_j;
  std::vector<std::uint8_t> fixed;
  std::vector<long double> area;
  sc::NodalWallWeights weights;
  q4_planar_test::Wall wall=q4_planar_test::Square(2);
  sc::PlanarWallBox motion{{0,-.75,-.75},{0,.75,.75}};
  Fixture(std::size_t quads=328344,std::size_t triangles=21301,std::size_t nodes=359785)
    :nq(quads),nt(triangles),n(nodes),x(3*n),v(3*n),w(3*n),q(4*n),inverse(n),inverse_j(n),fixed(n),area(n){}
  bool Prepare();
  sc::VectorView Positions() const {return {x.data(),static_cast<std::uint32_t>(n),3,1};}
  sc::NodalWallDeviceConfig Config(fe::NodalStamp={}) const;
  bool Owner(fe::FENodalState&) const;
  bool Bind(fe::FENodalState&,sc::NodalWallContactDevice&) const;
};
struct Results {
  sc::NodalWallDiagnostics diagnostics;
  std::vector<sc::NodalWallParentResult> parents;
  std::vector<sc::NodalWallPointResult> nodes;
  std::vector<std::uint64_t> faces;
  explicit Results(const Fixture& f):parents(f.nq+f.nt),nodes(f.n),faces(f.n){}
  sc::NodalWallDeviceResultView View(){return {&diagnostics,parents.data(),nodes.data(),faces.data(),parents.size(),nodes.size()};}
};
void CheckIncidence(const Fixture&,const detail::PreparedModel&);
void CheckLoads(const Fixture&,const Results&,const std::vector<double>&);
void SameResults(const Results&,const Results&);
std::vector<double> Forces(const fe::NodalAssemblyView&);
} // namespace vehicle_wall_device_test
