// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/collision/NodalWallMappedContact.h"
#include "GeometryFixture.h"
namespace physical_wall_test {
namespace c=tlfea::contact;
namespace p=physical_publication_test;
namespace fe=tl::fea;
inline c::NodalWallMappedSource Source(p::Rig& rig) {
  return {&rig.fixture.physical,&rig.fixture.rigid,rig.fixture.WitnessSource(),&rig.publication,
      rig.Participants(),rig.fixture.Identity()};
}
inline c::NodalWallDeviceConfig Config(p::Rig& rig) {
  return Settings(rig.owner.accepted(),p::Configuration,p::Qualification);
}
inline bool Good(c::NodalWallDeviceReport report) {
  EXPECT_EQ(report.status,c::NodalWallDeviceStatus::Ok)<<report.message<<" node="<<report.node<<" parent="<<report.parent;
  return report.status==c::NodalWallDeviceStatus::Ok;
}
struct Results {
  c::NodalWallDiagnostics diagnostics;
  std::vector<c::NodalWallParentResult> parents;
  std::vector<c::NodalWallPointResult> nodes;
  std::vector<std::uint64_t> faces;
  explicit Results(const Geometry& geometry) : parents(geometry.weights.parent_count()),
      nodes(geometry.weights.node_count()),faces(nodes.size()) {}
  c::NodalWallDeviceResultView View() {
    return {&diagnostics,parents.data(),nodes.data(),faces.data(),parents.size(),nodes.size()};
  }
};
} // namespace physical_wall_test
