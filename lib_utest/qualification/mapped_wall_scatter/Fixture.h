#pragma once
#include "lib_src/collision/nodal_wall_mapped/ScatterNode.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <gtest/gtest.h>
namespace wall_scatter_test {
namespace c=tlfea::contact;
namespace d=c::nodal_wall_device_detail;
namespace m=c::nodal_wall_mapped;
namespace fe=tl::fea;
constexpr unsigned Nodes=137,Globals=257;
struct Packet {
  d::Storage storage;
  c::NodalWallNodeWeight nodes[Nodes];
  c::NodalWallPointResult result[Nodes],base[Nodes];
  d::Control status[Nodes];
  double staged[6*Globals],error[Nodes],private_stiffness[Nodes];
  double forces[6*Globals],stiffness[Globals],rotation[Globals];
  fe::NodalAssemblyResult assembly_result;
  fe::stability::RowBounds bounds;
  void Reset();
  fe::NodalAssemblyView View();
  fe::NodalCinAssemblyView Cin();
  m::Sidecar Side();
};
inline std::uint64_t Bits(double value) {
  std::uint64_t result;
  std::memcpy(&result,&value,sizeof(result));
  return result;
}
inline void Same(const Packet& a,const Packet& b) {
  EXPECT_EQ(a.storage.control.status,b.storage.control.status);
  EXPECT_EQ(a.storage.control.node,b.storage.control.node);
  EXPECT_EQ(a.storage.control.parent,b.storage.control.parent);
  EXPECT_EQ(a.storage.control.point.status,b.storage.control.point.status);
  EXPECT_EQ(a.storage.control.point.cause,b.storage.control.point.cause);
  for(unsigned i=0;i<6*Globals;++i) EXPECT_EQ(Bits(a.forces[i]),Bits(b.forces[i]))<<i;
  for(unsigned i=0;i<Globals;++i) {
    EXPECT_EQ(Bits(a.stiffness[i]),Bits(b.stiffness[i]))<<i;
    EXPECT_EQ(Bits(a.rotation[i]),Bits(b.rotation[i]))<<i;
  }
  if(a.storage.control.status==c::NodalWallDeviceStatus::Ok)
    for(unsigned i=0;i<Nodes;++i) EXPECT_EQ(Bits(a.error[i]),Bits(b.error[i]))<<i;
}
struct Destinations {
  double forces[6*Globals],stiffness[Globals],rotation[Globals];
  explicit Destinations(const Packet& p) {
    std::copy_n(p.forces,6*Globals,forces);
    std::copy_n(p.stiffness,Globals,stiffness);
    std::copy_n(p.rotation,Globals,rotation);
  }
  void Unchanged(const Packet& p) const {
    EXPECT_EQ(std::memcmp(forces,p.forces,sizeof(forces)),0);
    EXPECT_EQ(std::memcmp(stiffness,p.stiffness,sizeof(stiffness)),0);
    EXPECT_EQ(std::memcmp(rotation,p.rotation,sizeof(rotation)),0);
  }
};
void Fault(Packet&,unsigned);
} // namespace wall_scatter_test
