// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/assembly/ShellNodeMap.h"
#include "lib_src/collision/NodalWallContactDevice.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include <gtest/gtest.h>
#include <vector>
namespace physical_wall_test {
namespace c=tlfea::contact;
namespace fe=tl::fea;
struct Geometry {
  std::array<c::Q4ParametricReference,3> quads;
  c::T3MaterialMeasure triangle;
  c::NodalWallWeights weights;
  std::array<c::PlanarWallTriangle,2> faces;
  std::array<c::PlanarWallVertex,4> vertices{{{{.039,-1,-1},1},{{.039,-1,1},2},
      {{.039,1,1},3},{{.039,1,-1},4}}};
  template<class Source>
  explicit Geometry(const Source& source,bool alter_reference=false)
      : Geometry(source.source.shells,source.shells,source.x,alter_reference) {}
  Geometry(const fe::ShellBatchBinding& binding,const fe::ShellNodeMap& mapping,
      const std::vector<double>& initial,bool alter_reference=false) {
    auto xyz=initial;
    if(alter_reference) xyz[3*mapping.domain()->Find(14)]+=.001;
    const c::VectorView coordinates{xyz.data(),static_cast<std::uint32_t>(mapping.owner_node_count()),3,1};
    std::array<c::NodalWallParentInput,4> input;
    for(unsigned i=0;i<3;++i) {
      c::SurfaceQ4 parent;
      parent.feature_id=parent.parent_element_id=i<2?binding.qeph_source_id(i):binding.qbat_source_id(0);
      const auto& local=i<2?binding.qeph_nodes(i):binding.qbat_nodes(0);
      for(unsigned n=0;n<4;++n) parent.nodes[n]=mapping.owner_index(local[n]);
      EXPECT_EQ(quads[i].Initialize(coordinates,&parent,1).status,c::Q4ParametricStatus::Ok);
      input[i]={&quads[i],0,nullptr};
    }
    c::SurfaceTriangle parent;
    parent.feature_id=parent.parent_element_id=binding.t3_source_id(0);
    parent.interpolation=c::SurfaceInterpolation::kLinearTriangle;
    for(unsigned n=0;n<3;++n) parent.nodes[n]=mapping.owner_index(binding.t3_nodes(0)[n]);
    EXPECT_EQ(c::PrepareT3MaterialMeasure(coordinates,parent,&triangle),c::SurfaceMeasureStatus::Ok);
    input[3]={nullptr,0,&triangle};
    EXPECT_EQ(weights.Initialize(coordinates.node_count,input.data(),input.size()).status,c::NodalWallStatus::Ok);
    for(unsigned i=0;i<4;++i) vertices[i].assembled_source_node_id=1001+i;
    for(unsigned i=0;i<2;++i) {
      faces[i].triangle_id=700+i;
      faces[i].source_quad_id=600;
      faces[i].assembled_source_quad_id=1600;
    }
    faces[0].nodes[0]=0; faces[0].nodes[1]=1; faces[0].nodes[2]=2;
    faces[1].nodes[0]=0; faces[1].nodes[1]=2; faces[1].nodes[2]=3;
  }
  c::PlanarWallView Wall() const { return {vertices.data(),4,faces.data(),2}; }
  c::PlanarWallBox Motion() const { return {{.039,-.2,-.2},{.039,.2,.2}}; }
};
inline c::NodalWallDeviceConfig Settings(const fe::NodalStamp& stamp,
    std::uint64_t configuration,std::uint64_t qualification) {
  c::NodalWallDeviceConfig config;
  config.owner=stamp;
  config.configuration_id=configuration;
  config.qualification_id=qualification;
  config.wall_binding_id=723;
  config.law={.039,1e7,.1,1e-6,1e-6};
  config.max_host_bytes=8u<<20;
  return config;
}
} // namespace physical_wall_test
