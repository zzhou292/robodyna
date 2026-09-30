// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../nodal_empty_cin/Fixture.h"
#include "../../shell_tab1_glass/Tab1Fixture.h"
namespace glass_removal_test {
namespace fe=tl::fea;
inline constexpr std::uint64_t GlassMaterial=2000452;
inline constexpr double Density=2499.9999999999995,Thickness=.0033900000000000002;
// The original part/material/section blocks and importer policy are retained
// in source-2000452.json. Source NUMINT1 means TAB1 AnyPoint with three section
// points; it does not mean a one-point QEPH material integration.
inline nodal_empty_test::Source Source() {
 nodal_empty_test::Source s;s.nodes=11;s.quads.resize(2);s.triangles.resize(1);s.source_instance_id=910077;
 fe::ShellPlasticityMaterialInput glass;
 glass.material_id=GlassMaterial;glass.young_pa=70e9;glass.poisson_ratio=.22;glass.density_kg_m3=Density;
 glass.law=fe::ShellSectionLaw::LayeredLaw44Nip3;glass.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
 glass.linear={30e6,1e9};glass.rate={true,0,1,10000,tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
 auto elastic=glass;elastic.material_id=910001;elastic.law=fe::ShellSectionLaw::LayeredLaw1Nip3;
 elastic.hardening=tl::material::ShellPlasticityHardeningKind::Tabulated;elastic.linear={};elastic.rate={};
 s.materials={elastic,glass};s.sections={{910001,.002,3,fe::ShellSectionFormulation::LayeredNip3},
   {GlassMaterial,Thickness,3,fe::ShellSectionFormulation::LayeredNip3}};
 const tl::math::Vec3 quad[]{{0,0,0},{.04,0,0},{.04,.02,0},{0,.02,0}};
 for(unsigned parent=0;parent<2;++parent) {
  auto& q=s.quads[parent];q.source_parent_id=910100+parent;q.reference.density=Density;
  q.reference.young_modulus=70e9;q.reference.poisson_ratio=.22;
  q.reference.thickness=parent?Thickness:.002;q.reference.projection_working_length_m=.001;
  for(unsigned k=0;k<4;++k){q.nodes[k]=4*parent+k;q.reference.node_ids[k]=100+q.nodes[k];
   q.reference.position[k]=quad[k];q.reference.position[k].z=parent?0:-.00025;}
  const auto mid=parent?GlassMaterial:910001;
  s.parents.push_back({fe::ShellBindingFamily::Qeph,parent,q.source_parent_id,mid,mid,mid});
 }
 auto& t=s.triangles[0];t.source_parent_id=910102;t.reference.density=Density;
 t.reference.young_modulus=70e9;t.reference.poisson_ratio=.22;t.reference.thickness=.002;
 const tl::math::Vec3 tri[]{{.01,.005,.00025},{.03,.005,.00025},{.02,.015,.00025}};
 for(unsigned k=0;k<3;++k){t.nodes[k]=8+k;t.reference.node_ids[k]=108+k;t.reference.position[k]=tri[k];}
 s.parents.push_back({fe::ShellBindingFamily::T3,0,t.source_parent_id,910001,910001,910001});return s;
}
inline std::vector<fe::ShellFailureParentInput> Failures(const nodal_empty_test::Source& source) {
 std::vector<fe::ShellFailureParentInput> out;
 for(const auto& parent:source.parents){fe::ShellFailureParentInput row;row.source=parent;
  if(parent.material_id==GlassMaterial){row.policy=fe::ShellFailurePolicy::Tab1AnyPoint;row.tab1={tab1_test::Table()};}
  out.push_back(row);}
 return out;
}
}
