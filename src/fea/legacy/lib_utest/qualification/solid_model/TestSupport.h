// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solids/Model.h"
#include "lib_src/elements/solid18/Solid18Reference.h"
#include "lib_src/elements/solid6z/Solid6zReference.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace solid_model_test {
namespace fe=tl::fea;
namespace s=fe::solids;
inline void Require(bool success) {if(!success)throw std::runtime_error("solid model test fixture");}
struct Fixture {
  double strain[3]{0,.2,.4},stress[3]{1e6,2e6,3e6};
  std::vector<s::Input18> input18;
  std::vector<s::Input24> input24;
  std::vector<s::Input6z> input6z;
  std::vector<fe::NodalDomainNode> nodes;
  Fixture() {
    const fe::solid18::Vec3 x[8]{{0,0,0},{.02,0,0},{.02,.02,0},{0,.02,0},
      {0,0,.02},{.02,0,.02},{.02,.02,.02},{0,.02,.02}};
    fe::solid18::ReferenceInput a;
    a.source_element_id=101;a.source_part_id=10;a.source_section_id=11;a.source_material_id=20;
    a.density_kg_m3=1070;
    fe::solid24::ReferenceInput b;
    b.source_element_id=102;b.source_part_id=12;b.source_section_id=13;b.source_material_id=30;
    b.density_kg_m3=1980;b.profile.reference_strain=fe::solid24::ReferenceStrain::TotalLagrangian10;
    for(unsigned n=0;n<8;++n) {
      a.position_m[n]=b.position_m[n]=x[n];
      a.source_node_id[n]=b.source_node_id[n]=n+1;
      nodes.push_back({n+1,x[n]});
    }
    input18.resize(1);input24.resize(1);input6z.resize(1);
    Require(fe::solid18::InitializeReference(a,input18[0].reference)==fe::solid18::Status::Success);
    Require(fe::solid24::InitializeReference(b,input24[0].reference)==fe::solid24::Status::Success);
    Require(tl::material::law36::Prepare(100e6,.3,1070,{strain,stress,3},input18[0].material)==tl::material::law36::Status::Ok);
    Require(tl::material::law42::Prepare(24e6,.463,1980,1e26,input24[0].material)==tl::material::law42::Status::Ok);
    fe::solid6z::ReferenceInput c;
    c.source_element_id=103;c.source_part_id=14;c.source_section_id=15;c.source_material_id=30;
    c.density_kg_m3=1980;
    constexpr unsigned slot[]{0,1,3,4,5,7};
    for(unsigned n=0;n<6;++n) {c.position_m[n]=x[slot[n]];c.source_node_id[n]=slot[n]+1;}
    Require(fe::solid6z::InitializeReference(c,input6z[0].reference)==fe::solid6z::Status::Success);
    input6z[0].material=input24[0].material;
    nodes.insert(nodes.begin()+2,{99,{1,2,3}}); // Uncovered domain nodes remain legal.
    std::reverse(nodes.begin(),nodes.end());
  }
  s::ModelInput Input() const {
    return {777,{input18.empty()?nullptr:input18.data(),input18.size()},
      {input24.empty()?nullptr:input24.data(),input24.size()},
      {input6z.empty()?nullptr:input6z.data(),input6z.size()}};
  }
  fe::NodalNodeDomain Domain() const {
    fe::NodalNodeDomain domain;
    Require(bool(domain.Initialize({777,nodes.data(),nodes.size()})));
    return domain;
  }
};
inline bool Bits(double a,double b) {return std::memcmp(&a,&b,sizeof(double))==0;}
} // namespace solid_model_test
