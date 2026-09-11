// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/collision/NodalWallMappedContact.h"
#include "lib_src/collision/Q4ParametricContact.h"
namespace physical_wall_test {
namespace c=tlfea::contact;
namespace p=physical_publication_test;
namespace fe=tl::fea;
struct Geometry {
  std::array<c::Q4ParametricReference,3> quads;
  c::T3MaterialMeasure triangle;
  c::NodalWallWeights weights;
  std::array<c::PlanarWallTriangle,2> faces;
  std::array<c::PlanarWallVertex,4> vertices{{{{.039,-1,-1},1},{{.039,-1,1},2},
      {{.039,1,1},3},{{.039,1,-1},4}}};
  explicit Geometry(const p::Fixture& source,bool alter_reference=false) {
    auto xyz=source.x;
    if(alter_reference) xyz[3*source.domain.Find(14)+1]+=.001;
    const c::VectorView coordinates{xyz.data(),static_cast<std::uint32_t>(source.domain.node_count()),3,1};
    const auto& binding=source.source.shells;
    std::array<c::NodalWallParentInput,4> input;
    for(unsigned i=0;i<3;++i) {
      c::SurfaceQ4 parent;
      parent.feature_id=parent.parent_element_id=i<2?binding.qeph_source_id(i):binding.qbat_source_id(0);
      const auto& local=i<2?binding.qeph_nodes(i):binding.qbat_nodes(0);
      for(unsigned n=0;n<4;++n) parent.nodes[n]=source.shells.owner_index(local[n]);
      EXPECT_EQ(quads[i].Initialize(coordinates,&parent,1).status,c::Q4ParametricStatus::Ok);
      input[i]={&quads[i],0,nullptr};
    }
    c::SurfaceTriangle parent;
    parent.feature_id=parent.parent_element_id=binding.t3_source_id(0);
    parent.interpolation=c::SurfaceInterpolation::kLinearTriangle;
    for(unsigned n=0;n<3;++n) parent.nodes[n]=source.shells.owner_index(binding.t3_nodes(0)[n]);
    EXPECT_EQ(c::PrepareT3MaterialMeasure(coordinates,parent,&triangle),c::SurfaceMeasureStatus::Ok);
    input[3]={nullptr,0,&triangle};
    EXPECT_EQ(weights.Initialize(coordinates.node_count,input.data(),input.size()).status,c::NodalWallStatus::Ok);
    for(unsigned i=0;i<2;++i) {
      faces[i].triangle_id=700+i;
    }
    faces[0].nodes[0]=0; faces[0].nodes[1]=1; faces[0].nodes[2]=2;
    faces[1].nodes[0]=0; faces[1].nodes[1]=2; faces[1].nodes[2]=3;
  }
  c::PlanarWallView Wall() const { return {vertices.data(),4,faces.data(),2}; }
  c::PlanarWallBox Motion() const { return {{-1,-.2,-.2},{1,.2,.2}}; }
};
inline c::NodalWallMappedSource Source(p::Rig& rig) {
  return {&rig.fixture.physical,&rig.fixture.rigid,rig.fixture.WitnessSource(),&rig.publication,
      rig.Participants(),rig.fixture.Identity()};
}
inline c::NodalWallDeviceConfig Config(p::Rig& rig) {
  c::NodalWallDeviceConfig config;
  config.owner=rig.owner.accepted();
  config.configuration_id=p::Configuration;
  config.qualification_id=p::Qualification;
  config.wall_binding_id=723;
  config.law={.039,1e7,.1,1e-6,1e-6};
  config.max_host_bytes=8u<<20;
  return config;
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
