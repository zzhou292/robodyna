#include "Fixture.h"
#include "../nodal_mass/NodalMassTestSupport.h"

namespace qbat_binding_test {
TEST(QbatBinding, CombinedType25MassUsesEveryShellLayerAndEachEndpointOnce) {
  Fixture f;
  Binding shell;
  ASSERT_EQ(shell.InitializeFormulations(f.Input()).status,Status::Success);
  const nodal_mass_test::SpringInput input(shell);
  const auto connectors=nodal_mass_test::Connectors(input,shell.node_count());
  fe::NodalMassBinding mass;
  ASSERT_TRUE(mass.Initialize(shell,connectors));
  ASSERT_TRUE(mass.Matches(shell));
  Exact(mass.totals().shell,shell.totals());
  for(std::size_t n=0;n<shell.node_count();++n) {
    const auto& actual=mass.nodes()[n].coefficients;
    Exact(actual.shell,shell.nodes()[n].native);
    double expected_mass=shell.nodes()[n].native.mass;
    double expected_inertia=shell.nodes()[n].native.isotropic_inertia;
    for(std::size_t e=0;e<2*connectors.connection_count();++e) {
      const auto& endpoint=connectors.endpoint_mass()[e];
      if(endpoint.global_node!=n) continue;
      expected_mass+=endpoint.mass_kg;
      expected_inertia+=endpoint.isotropic_inertia_kg_m2;
    }
    EXPECT_EQ(Bits(actual.mass),Bits(expected_mass));
    EXPECT_EQ(Bits(actual.isotropic_inertia),Bits(expected_inertia));
  }
  auto other=f;
  other.b.reference.initial_a11_pa*=2;
  Binding different;
  ASSERT_EQ(different.InitializeFormulations(other.Input()).status,Status::Success);
  Exact(different.totals(),shell.totals());
  EXPECT_FALSE(mass.Matches(different)); // Equal M/J does not erase formulation identity.
}
} // namespace qbat_binding_test
