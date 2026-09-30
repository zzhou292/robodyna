#pragma once
#include <gtest/gtest.h>
#include "lib_src/assembly/NodalMassBinding.h"
#include "lib_src/elements/type25/Type25Model.h"
#include "../t3/mixed_binding/ShellBatchBindingFixture.h"
#include <type_traits>

namespace nodal_mass_test {
namespace fe=tl::fea;
namespace spring=fe::type25;
using Status=fe::NodalMassStatus;
using shell_binding_test::Bytes;
using shell_binding_test::Exact;
inline fe::ShellBatchBinding Shells() {
  fe::ShellBatchBinding result;
  EXPECT_EQ(result.Initialize(shell_binding_test::Edge()).status,fe::ShellBindingStatus::Success);
  return result;
}
struct SpringInput {
  spring::PropertyInput property;
  std::array<spring::ConnectionInput,2> connection;
  explicit SpringInput(const fe::ShellBatchBinding& shells) {
    property.source_property_id=8001;
    auto& p=property.property; p.mass_kg=.001; p.isotropic_inertia_kg_m2=1.e-8;
    for(unsigned i=0;i<4;++i) {
      p.stiffness[i]=1000.; p.damping[i]=.01;
      p.failure_negative[i]=-1.e20; p.failure_positive[i]=1.e20;
      p.failure_weight[i]=1.; p.failure_exponent[i]=2.;
    }
    for(unsigned c=0;c<2;++c) {
      auto& input=connection[c]; input.source_element_id=9001+c;
      for(unsigned e=0;e<2;++e) {
        const auto n=e?shells.node_count()-1:c;
        input.global_node[e]=n; input.source_node_id[e]=shells.nodes()[n].source_id;
        input.position[e]=shells.nodes()[n].position;
      }
    }
  }
  spring::ModelInput Input(std::size_t nodes) const {
    spring::ModelInput input;
    input.source_instance_id=77; input.global_node_count=nodes; input.source_units={1.,1.,1.};
    input.properties=&property; input.property_count=1;
    input.connections=connection.data(); input.connection_count=connection.size();
    return input;
  }
};
inline spring::Model Connectors(const SpringInput& fixture,std::size_t nodes) {
  spring::Model result; const auto report=result.Initialize(fixture.Input(nodes));
  EXPECT_TRUE(report)<<report.message; return result;
}
inline void Near(double value,long double expected) {
  EXPECT_LE(std::abs(static_cast<long double>(value)-expected),
      2.e-12L*std::max(std::abs(expected),1.e-30L));
}
static_assert(std::is_nothrow_copy_constructible_v<fe::NodalMassBinding>);
static_assert(std::is_nothrow_move_constructible_v<fe::NodalMassBinding>);
static_assert(!std::is_copy_assignable_v<fe::NodalMassBinding>);
} // namespace nodal_mass_test
