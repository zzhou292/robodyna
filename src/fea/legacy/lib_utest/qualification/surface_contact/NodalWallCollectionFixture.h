#pragma once
#include "NodalWallOwnerFixture.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"
#include <algorithm>

namespace nodal_wall_collection_test {
namespace sc=tlfea::contact;
namespace fe=tl::fea;
namespace detail=sc::nodal_wall_device_detail;
using Code=sc::NodalWallDeviceStatus;
using nodal_wall_owner_test::Same;
using nodal_wall_owner_test::Bytes;
using nodal_wall_owner_test::Unchanged;
using nodal_wall_owner_test::Near;
constexpr unsigned Nodes=128,Parents=128;
constexpr double Step=1./1024,ForceBudget=5e-7,EnergyBudget=1.2500000000000005e-12;
constexpr std::uint64_t Qualification=0x4e57434f4c4c5431ULL,WideId=std::uint64_t{1}<<54;
using Vectors=std::array<double,3*Nodes>;
using Forces=std::array<double,6*Nodes>;
// Capacity fixture only: an 8x16 rectangular node grid, actual QEPH/T3
// startup contributions, then prescribed normal positions. Not source Yaris.
// Every Q4 reference remains its existing one-parent immutable object.
struct Fixture {
  unsigned parent_count=0,q4_count=0,t3_count=0;
  Vectors reference{},x{},v{},omega{};
  std::array<double,4*Nodes> q{};
  std::array<double,Nodes> mass{},inertia{},inverse{},inverse_j{};
  std::array<std::uint8_t,Nodes> fixed{},rotation_fixed{};
  std::array<long double,Nodes> area{};
  std::array<sc::SurfaceQ4,Parents> quads{};
  std::array<sc::SurfaceTriangle,Parents> triangles{};
  std::array<sc::Q4ParametricReference,Parents> references;
  std::array<sc::T3MaterialMeasure,Parents> measures;
  std::array<sc::NodalWallParentInput,Parents> input{};
  sc::NodalWallWeights weights;
  q4_planar_test::Wall wall=q4_planar_test::Square(2);
  sc::PlanarWallBox motion{{0,-1,-1},{0,1,1}};
  bool started=false;
  bool Prepare(unsigned count=128,bool reverse=false);
  sc::VectorView View(const Vectors& a) const { return {a.data(),Nodes,3,1}; }
  sc::NodalWallDeviceConfig Config(fe::NodalStamp stamp={}) const;
  bool Owner(fe::FENodalState&) const;
  bool Bind(fe::FENodalState&,sc::NodalWallContactDevice&,sc::NodalWallDeviceConfig) const;
  sc::NodalWallResult Host(const Vectors&,const Vectors&,std::uint64_t epoch,std::uint64_t attempt,
                          sc::NodalWallConfig) const;
};
struct Snapshot {
  Vectors x{},v{},omega{},reaction{},couple{};
  std::array<double,4*Nodes> q{};
  fe::NodalStamp stamp;
};
Snapshot Read(fe::FENodalState&);
void Same(const Snapshot&,const Snapshot&);
Forces ReadForces(const fe::NodalAssemblyView&);
Snapshot ReadPrepared(const fe::NodalPreparedView&);
void CheckLoads(const Fixture&,const sc::NodalWallDeviceResults&,const Vectors&,const Vectors&,double stiffness);
} // namespace nodal_wall_collection_test
