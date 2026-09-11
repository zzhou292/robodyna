// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../nodal_coefficients/SolidFixture.h"
#include "lib_src/constraints/NodalRigidPartAssemblyModel.h"

namespace rigid_part_solid_test {
namespace fe=tl::fea;
namespace r=fe::rigid;
struct Fixture {
  coefficient_test::SolidFixture source;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap map;
  fe::SolidNodeContributions solid;
  fe::NodalCoefficientLedger ledger;
  r::NodalRigidPartTopology topology;
  std::vector<std::uint64_t> ids;
  std::vector<r::AssemblyMassPoint> points;
  r::AssemblyPrimary primary;
  Fixture(unsigned family):domain(source.Domain()),map(source.Map(domain)) {
    fe::solid18::Reference a; fe::solid24::Reference b; fe::solid6z::Reference c;
    EXPECT_EQ(fe::solid18::InitializeReference(source.a,a),fe::solid18::Status::Success);
    EXPECT_EQ(fe::solid24::InitializeReference(source.b,b),fe::solid24::Status::Success);
    EXPECT_EQ(fe::solid6z::InitializeReference(source.c,c),fe::solid6z::Status::Success);
    fe::SolidCoefficientInput input; input.source_instance_id=1;
    if(family==0) {input.solid18=&a;input.solid18_count=1;}
    else if(family==1) {input.solid24=&b;input.solid24_count=1;}
    else {input.solid6z=&c;input.solid6z_count=1;}
    EXPECT_TRUE(solid.Initialize(domain,input));
    EXPECT_TRUE(ledger.InitializeWithSolids({{&map},nullptr,&solid}));
    const auto& parent=solid.parents()[0];
    // Only solid-supported nodes: no shell/beam/point-mass fallback coverage.
    for(unsigned k=0;k<parent.node_count;++k) {
      const auto n=parent.domain_node[k];
      if(!ledger.nodes()[n].occurrences.qeph&&!ledger.nodes()[n].occurrences.t3&&
          !ledger.nodes()[n].occurrences.qbat) ids.push_back(parent.source_node_id[k]);
    }
    std::sort(ids.begin(),ids.end());
    for(const auto id:ids) {
      const auto n=domain.Find(id);
      points.push_back({domain.nodes()[n].position,ledger.nodes()[n].coefficients.mass,0});
      const auto p=points.back().position;
      primary.position.x+=p.x;primary.position.y+=p.y;primary.position.z+=p.z;
    }
    primary.position.x/=ids.size();primary.position.y/=ids.size();primary.position.z/=ids.size();
    primary.mass=1e-20*1000.;primary.inertia=primary.mass*.001*.001;
    const r::PartTopologyPartInput part{200,ids.data(),ids.size()};
    r::PartTopologyInput top;
    top.source_instance_id=1;top.parts=&part;top.part_count=1;
    top.expected_members=ids.data();top.expected_member_count=ids.size();
    EXPECT_TRUE(topology.Initialize(top));
  }
  r::AssemblyBodyInput RawInput() const {
    return {primary,points.data(),nullptr,points.size(),0,{1000,.001}};
  }
};
} // namespace rigid_part_solid_test
