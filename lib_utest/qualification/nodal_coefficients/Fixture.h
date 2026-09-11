// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/assembly/NodalCoefficientLedger.h"
#include "lib_src/assembly/NodalMassBinding.h"
#include "../qbat_binding/Fixture.h"
#include "../type13_model/Fixture.h"
#include <cmath>
#include <cstring>

namespace coefficient_test {
namespace fe=tl::fea;
namespace beam=fe::type13;
namespace spring=fe::type25;
using S=fe::CoefficientStatus;
using qbat_binding_test::Bits;
using qbat_binding_test::Bytes;
struct Fixture {
  qbat_binding_test::Fixture shell_input;
  fe::ShellBatchBinding shells;
  std::vector<fe::NodalDomainNode> nodes;
  std::array<std::size_t,5> map{1,3,4,0,5};
  type13_model_test::Fixture beam_input;
  spring::PropertyInput spring_property;
  std::array<spring::ConnectionInput,2> spring_input;
  Fixture() {
    EXPECT_EQ(shells.InitializeFormulations(shell_input.Input()).status,fe::ShellBindingStatus::Success);
    nodes.resize(7);
    for(std::size_t n=0;n<5;++n) nodes[map[n]]={shells.nodes()[n].source_id,shells.nodes()[n].position};
    nodes[2]={777,{0,0,0}}; // Deliberately no admitted coefficient producer.
    nodes[6]={55,{.06,0,0}};
    beam_input.nodes={{10,map[0],{0,0,0}},{11,map[1],{40,0,0}},
                      {55,6,{60,0,0}},{99,SIZE_MAX,{0,0,1000}}};
    beam_input.connections={{8000,0,{0,1,3}},{8001,0,{1,2,3}}};
    beam_input.declaration.input.inertia_per_length=0;
    spring_property.source_property_id=9000;
    auto& p=spring_property.property;
    p.mass_kg=.001;
    p.isotropic_inertia_kg_m2=1e-8;
    for(unsigned i=0;i<4;++i) {
      p.stiffness[i]=1000;
      p.damping[i]=.01;
      p.failure_negative[i]=-1e20;
      p.failure_positive[i]=1e20;
      p.failure_weight[i]=1;
      p.failure_exponent[i]=2;
    }
    for(unsigned e=0;e<2;++e) {
      auto& c=spring_input[e];
      c.source_element_id=100+e; // Legal WID overlap with structural shell EID.
      for(unsigned local=0;local<2;++local) {
        const auto n=local?(e?6:map[4]):map[e];
        c.global_node[local]=n;
        c.source_node_id[local]=nodes[n].source_id;
        c.position[local]=nodes[n].position;
      }
    }
  }
  fe::NodalNodeDomain Domain(std::uint64_t source=1) const {
    fe::NodalNodeDomain domain;
    EXPECT_TRUE(domain.Initialize({source,nodes.data(),nodes.size()}));
    return domain;
  }
  fe::ShellNodeMap Map(const fe::NodalNodeDomain& domain) const {
    fe::ShellNodeMap result;
    EXPECT_TRUE(result.Initialize(shells,domain));
    return result;
  }
  beam::Model Beams() const {
    auto input=beam_input.Input();
    input.global_node_count=nodes.size();
    beam::Model result;
    EXPECT_TRUE(result.Initialize(input));
    return result;
  }
  fe::Type13NodeContributions Contributions(const fe::NodalNodeDomain& domain) const {
    fe::Type13NodeContributions result;
    EXPECT_TRUE(result.Initialize(Beams(),domain));
    return result;
  }
  spring::Model Springs(std::uint64_t source=1) const {
    spring::ModelInput input;
    input.source_instance_id=source;
    input.global_node_count=nodes.size();
    input.source_units={1,1,1};
    input.properties=&spring_property;
    input.property_count=1;
    input.connections=spring_input.data();
    input.connection_count=spring_input.size();
    spring::Model result;
    const auto report=result.Initialize(input);
    EXPECT_TRUE(report)<<report.message;
    return result;
  }
};
inline constexpr unsigned ValueCount=15;
inline std::array<double,ValueCount> Values(const fe::NodalCoefficientTotals& c) {
  return {c.mass,c.isotropic_inertia,c.shell.mass,c.shell.isotropic_inertia,
    c.shell.physical_inertia,c.shell.added_inertia,c.type25.mass,c.type25.isotropic_inertia,
    c.type13.mass,c.type13.isotropic_inertia,c.type13.added_inertia,c.element_mass,
    c.solid18_mass,c.solid24_mass,c.solid6z_mass};
}
inline void Exact(const fe::NodalCoefficientLedger& a,const fe::NodalCoefficientLedger& b) {
  ASSERT_EQ(a.nodes().size(),b.nodes().size());
  for(std::size_t n=0;n<a.nodes().size();++n) {
    const auto x=Values(a.nodes()[n].coefficients),y=Values(b.nodes()[n].coefficients);
    for(unsigned k=0;k<x.size();++k) EXPECT_EQ(Bits(x[k]),Bits(y[k]));
    const auto& u=a.nodes()[n].occurrences;
    const auto& v=b.nodes()[n].occurrences;
    EXPECT_EQ(u.qeph,v.qeph); EXPECT_EQ(u.t3,v.t3); EXPECT_EQ(u.qbat,v.qbat);
    EXPECT_EQ(u.type25,v.type25); EXPECT_EQ(u.type13,v.type13);
    EXPECT_EQ(u.element_mass,v.element_mass);
    EXPECT_EQ(u.solid18,v.solid18); EXPECT_EQ(u.solid24,v.solid24); EXPECT_EQ(u.solid6z,v.solid6z);
  }
  const auto x=Values(a.totals()),y=Values(b.totals());
  for(unsigned k=0;k<x.size();++k) EXPECT_EQ(Bits(x[k]),Bits(y[k]));
}
} // namespace coefficient_test
