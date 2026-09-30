// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_mapped/Fixture.h"
#include "../type25/EvaluationValues.h"
#include "../type25/Fixture.h"
#include "lib_src/elements/type25/Type25Math.h"
#include "lib_src/elements/type25/mapped/Startup.h"
#include "lib_src/elements/type25/mapped/Stiffness.h"
#include "lib_src/elements/type25/Type25BatchStorage.h"

namespace type25_mapped_test {
namespace fe=tl::fea;
namespace spring=fe::type25;
namespace mapped=spring::mapped;
namespace batch=spring::batch_detail;
using qbat_binding_test::Bits;
struct Fixture {
  qbat_mapped_test::Fixture base;
  std::vector<spring::ConnectionInput> connections;
  std::array<spring::PropertyInput,2> properties;
  spring::Model model;
  fe::NodalCoefficientLedger ledger;
  fe::ShellPhysicalBinding physical;
  explicit Fixture(bool failing=false,bool damping=false,bool rigid_endpoint=false) {
    const auto& old=base.mechanics.springs;
    connections.assign(old.connections(),old.connections()+old.connection_count());
    properties[0]=old.properties()[0];
    properties[1]=properties[0];
    properties[1].source_property_id++;
    for (auto& p:properties) for (auto& d:p.property.damping) d=0;
    if (failing) for (auto& limit:properties[1].property.failure_positive) limit=1e-12;
    if (damping) properties[1].property.damping[3]=.01;
    connections.back().property_index=1;
    if (rigid_endpoint) {
      const auto node=base.mechanics.domain.Find(10);
      connections.back().global_node[0]=node;
      connections.back().source_node_id[0]=10;
      connections.back().position[0]=base.mechanics.domain.nodes()[node].position;
    }
    spring::ModelInput input;
    input.source_instance_id=old.source_instance_id();
    input.global_node_count=old.global_node_count();
    input.source_units=old.source_units();
    input.properties=properties.data();
    input.property_count=properties.size();
    input.connections=connections.data();
    input.connection_count=connections.size();
    EXPECT_TRUE(model.Initialize(input));
    EXPECT_TRUE(ledger.InitializeWithSolids({{&base.mechanics.shells,&model},&base.point,&base.mechanics.solids}));
    EXPECT_TRUE(physical.Initialize({base.physical.shells(),base.physical.catalog(),
        base.physical.failure(),nullptr},ledger));
    // Zero damping changes no physical coefficient. Preserve the independently
    // constructed owner fixture's source M/J and authentic constrained inverses.
    if (!rigid_endpoint) for (std::size_t node=0;node<ledger.nodes().size();++node) {
      EXPECT_EQ(Bits(ledger.nodes()[node].coefficients.mass),Bits(base.mechanics.m[node]));
      EXPECT_EQ(Bits(ledger.nodes()[node].coefficients.isotropic_inertia),Bits(base.mechanics.j[node]));
    }
  }
  fe::NodalCinWitnessSource Witnesses() const { return base.Witnesses(); }
  spring::BatchConfig Config() const {
    spring::BatchConfig c;
    c.owner=base.Config().owner;
    c.configuration_id=81;
    c.qualification_id=rigid_assembly_owner_test::Qualification;
    c.element_count=model.connection_count();
    return c;
  }
};
inline void Exact(const spring::Evaluation& a,const spring::Evaluation& b) {
  const auto av=type25_test::EvaluationValues(a);
  const auto bv=type25_test::EvaluationValues(b);
  for (std::size_t k=0;k<av.size();++k) EXPECT_EQ(Bits(av[k]),Bits(bv[k]))<<k;
  EXPECT_EQ(a.history.active,b.history.active);
}
} // namespace type25_mapped_test
