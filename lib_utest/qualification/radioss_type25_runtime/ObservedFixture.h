// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ObservedScene.h"
#include "../nodal_empty_cin/Fixture.h"
#include "lib_src/collision/RadiossType25Transaction.h"
#include "modelio/native_contact_scene/MaterialBridge.h"
#include <array>
namespace native_runtime_test {
namespace n=tlfea::contact::radioss_type25;namespace l=n::lifecycle;namespace fe=tl::fea;
namespace observed=native_scene_fixture;
// Imports literal exported source arrays; does not reconstruct a second mesh or
// use captured physical trajectories as initial conditions/restart state.
inline nodal_empty_test::Source ModelSource() {
  nodal_empty_test::Source source;source.nodes=18;source.quads.resize(4);source.triangles.resize(8);
  const auto point=[](unsigned node){return tl::math::Vec3{observed::PositionMm[3*node]*.001,
    observed::PositionMm[3*node+1]*.001,observed::PositionMm[3*node+2]*.001};};
  for(unsigned i=0;i<4;++i) {
    auto& q=source.quads[i];q.source_parent_id=observed::QuadsIds[i];
    q.reference.density=observed::Material[0]*1e12;q.reference.young_modulus=observed::Material[1]*1e6;
    q.reference.poisson_ratio=observed::Material[2];q.reference.thickness=observed::TimeAndThickness[2]*.001;
    for(unsigned j=0;j<4;++j){const auto node=observed::QuadsNodes[4*i+j];q.nodes[j]=node;
      q.reference.position[j]=point(node);q.reference.node_ids[j]=std::uint32_t(observed::NodeIds[node]);}
    source.parents.push_back({fe::ShellBindingFamily::Qeph,i,q.source_parent_id,2,1,1});
  }
  for(unsigned i=0;i<8;++i) {
    auto& t=source.triangles[i];t.source_parent_id=observed::TrianglesIds[i];
    t.reference.density=observed::Material[0]*1e12;t.reference.young_modulus=observed::Material[1]*1e6;
    t.reference.poisson_ratio=observed::Material[2];t.reference.thickness=observed::TimeAndThickness[2]*.001;
    for(unsigned j=0;j<3;++j){const auto node=observed::TrianglesNodes[3*i+j];t.nodes[j]=node;
      t.reference.position[j]=point(node);t.reference.node_ids[j]=observed::NodeIds[node];}
    source.parents.push_back({fe::ShellBindingFamily::T3,i,t.source_parent_id,1,1,1});
  }
  fe::ShellPlasticityMaterialInput material;material.material_id=1;
  material.young_pa=observed::Material[1]*1e6;material.poisson_ratio=observed::Material[2];material.density_kg_m3=observed::Material[0]*1e12;
  material.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
  material.rate={true,observed::Material[5],observed::Material[6],observed::Material[7]};
  const auto bridge=crash::modelio::native_scene::PrepareNativeHardening(material.young_pa,material.poisson_ratio,
      material.density_kg_m3,observed::Material[3]*1e6,observed::Material[4]*1e6,material.rate);
  material.linear={observed::Material[3]*1e6,bridge.derived_etan_pa};source.materials.push_back(material);
  source.sections.push_back({1,observed::TimeAndThickness[2]*.001,3,fe::ShellSectionFormulation::LayeredNip3});return source;
}
struct ObservedSource {
  std::array<l::Node,18> nodes;
  std::array<l::Main,16> mains;
  std::array<l::Secondary,18> secondary;
  std::array<l::NormalReference,18> normals;
  std::array<std::uint32_t,19> removal_offsets{};
  ObservedSource() {
    for(unsigned i=0;i<18;++i){nodes[i]={observed::NodeIds[i],observed::ConstraintCodes[i],observed::SkewCodes[i]};
      secondary[i]={observed::SecondaryNodes[i],observed::SecondaryK[i],observed::SecondaryGaps[i],observed::InitialContact[i]};
      normals[i].boundary=observed::Boundaries[i];
      // Boundary-zero bisectors are source-unused; canonical API zeros carry no
      // claim that arbitrary unconsumed native scratch was a physical value.
      if(normals[i].boundary)for(unsigned j=0;j<2;++j){const auto offset=6*i+3*j;
        normals[i].bisector[j]={float(observed::Bisectors[offset]),float(observed::Bisectors[offset+1]),float(observed::Bisectors[offset+2])};}
    }
    for(unsigned i=0;i<16;++i){auto& main=mains[i];main.global_id=observed::MainGlobalIds[i];main.segment_type=observed::MainRoles[i];
      main.coefficient=observed::MainK[i];main.maximum_gap=observed::MainMaximumGaps[i];
      for(unsigned j=0;j<4;++j){main.nodes[j]=observed::MainNodes[4*i+j];main.normal_reference[j]=observed::MainNormalReferences[4*i+j];
        main.neighbors[j]=observed::Neighbors[4*i+j];main.gap[j]=observed::MainGaps[4*i+j];const auto offset=12*i+3*j;
        main.normal_slot[j]={float(observed::MainNormals[offset]),float(observed::MainNormals[offset+1]),float(observed::MainNormals[offset+2])};}
    }
  }
  n::FixedMainSource View() const {
    n::FixedMainSource out;out.source_id=1;out.topology_generation=1;out.primary_main_count=8;
    out.primary_parent_ids=observed::TrianglesIds;out.primary_curvature=observed::Curvature;
    out.margin=observed::NativeControls[0];out.drad=observed::NativeControls[1];out.gap_load=observed::NativeControls[2];
    out.force_packet_size=observed::ForcePacketSize;out.native_workers=1;
    out.selection={nodes.data(),nodes.size(),mains.data(),mains.size(),secondary.data(),secondary.size(),normals.data(),normals.size(),
      {observed::NormalOffsets,19,observed::NormalEntries,sizeof(observed::NormalEntries)/sizeof(observed::NormalEntries[0])},
      {removal_offsets.data(),removal_offsets.size(),nullptr,0},1};return out;
  }
  static n::TransactionConfig Config() {
    n::TransactionConfig c;c.units={.001,1000,1};c.lifecycle.selection={1,5,1,false,false,false};
    c.lifecycle.geometry={1,1,5,1,false,false,false};c.lifecycle.coefficient={4,0};
    c.lifecycle.minimum_coefficient=observed::NativeControls[3];c.lifecycle.maximum_coefficient=observed::NativeControls[4];
    c.lifecycle.neighbor_removal=2;c.lifecycle.optcd_response_precision=observed::ResponsePrecision;
    c.normal.engine={0,0,0};c.normal.damping_factor=.05;c.friction={2,10,0,1,0,0,1};
    c.friction_coefficients={.1,{0,0,0,0,.1,-.001}};
    c.assembly={observed::AssemblyControls[0],observed::AssemblyControls[1],observed::AssemblyControls[2],
      observed::AssemblyControls[3],observed::AssemblyControls[4],{0,0,0}};return c;
  }
  static n::TransactionLimits Limits() {
    n::TransactionLimits l;l.inventory.max_nodes=18;l.inventory.max_secondaries=18;l.inventory.max_mains=8;
    l.inventory.max_tasks=8;l.inventory.max_pairs=144;
    l.inventory.max_removals=1; // Positive API capacity; actual genuine removal count remains zero.
    l.optimized_candidates=1024;l.sliding_entries=1024;l.max_device_bytes=16u<<20;l.max_host_bytes=16u<<20;return l;
  }
};
} // namespace native_runtime_test
