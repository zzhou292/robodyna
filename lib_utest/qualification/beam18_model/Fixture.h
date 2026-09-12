// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../beam18_force/TestSupport.h"
#include "../nodal_coefficients/Fixture.h"
#include "lib_src/assembly/Beam18NodeContributions.h"
#include <vector>
namespace beam18_model_test {
namespace fe=tl::fea;
namespace b=fe::beam18;
using beam18_test::Bytes;
using qbat_binding_test::Bits;
struct Fixture {
  coefficient_test::Fixture structural;
  std::vector<double> x{0,.01,.1},y{200e6,250e6,350e6};
  std::vector<b::ParentInput> rows;
  Fixture() { Add(9901,0,1); Add(9902,1,4); }
  void Add(std::uint64_t eid,std::size_t a,std::size_t z) {
    auto raw=beam18_force_test::Reference().input(); raw.source_element_id=eid;
    const auto& n=structural.nodes;
    raw.source_node_id[0]=n[a].source_id; raw.source_node_id[1]=n[z].source_id;
    raw.source_node_id[2]=99999;
    raw.position[0]=n[a].position; raw.position[1]=n[z].position;
    raw.position[2]={.1,.2,.3};
    b::ParentInput row;
    if(b::InitializeReference(raw,row.reference)!=b::Status::Success) throw std::runtime_error("beam reference");
    auto material=beam18_force_test::Material(row.reference).material;
    if(b::point::Prepare(material,{x.data(),y.data(),static_cast<std::uint32_t>(x.size())},row.material)!=b::point::Status::Ok)
      throw std::runtime_error("beam material");
    rows.push_back(row);
  }
  b::ModelInput Input() const {return {1,{rows.data(),rows.size()},b::ModelProfile::CircularFourPointLaw44V1};}
  b::Model Model(const fe::NodalNodeDomain& domain) const {
    b::Model result; const auto r=result.Initialize(domain,Input());
    EXPECT_TRUE(r)<<r.message<<" parent "<<r.parent; return result;
  }
  fe::Beam18NodeContributions Contributions(const fe::NodalNodeDomain& d) const {
    const auto m=Model(d); fe::Beam18NodeContributions result;
    EXPECT_TRUE(result.Initialize(m)); return result;
  }
};
} // namespace beam18_model_test
