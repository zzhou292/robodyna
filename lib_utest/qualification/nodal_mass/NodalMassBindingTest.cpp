#include "NodalMassTestSupport.h"

namespace nodal_mass_test {
TEST(NodalMassBinding, RepeatedEndpointsAddOnceWithoutChangingShellPartitions) {
  const auto shells=Shells(); const SpringInput input(shells);
  const auto connectors=Connectors(input,shells.node_count()); fe::NodalMassBinding binding;
  ASSERT_TRUE(binding.Initialize(shells,connectors));
  ASSERT_EQ(binding.nodes().size(),shells.node_count());
  long double total_mass=0,total_inertia=0;
  for(std::size_t n=0;n<shells.node_count();++n) {
    const auto& node=binding.nodes()[n]; const auto& native=shells.nodes()[n].native;
    const unsigned incidence=n==4?2u:n<2?1u:0u;
    EXPECT_EQ(node.source_id,shells.nodes()[n].source_id); Exact(node.coefficients.shell,native);
    const long double mass=incidence*.5L*input.property.property.mass_kg;
    const long double inertia=incidence*.5L*input.property.property.isotropic_inertia_kg_m2;
    Near(node.coefficients.connector_mass,mass); Near(node.coefficients.connector_inertia,inertia);
    Near(node.coefficients.mass,native.mass+mass);
    Near(node.coefficients.isotropic_inertia,native.isotropic_inertia+inertia);
    total_mass+=native.mass+mass; total_inertia+=native.isotropic_inertia+inertia;
  }
  Exact(binding.totals().shell,shells.totals());
  Near(binding.totals().connector_mass,2.L*input.property.property.mass_kg);
  Near(binding.totals().connector_inertia,2.L*input.property.property.isotropic_inertia_kg_m2);
  Near(binding.totals().mass,total_mass); Near(binding.totals().isotropic_inertia,total_inertia);
  EXPECT_TRUE(binding.Matches(shells)); EXPECT_TRUE(binding.Matches(connectors));
  EXPECT_EQ(binding.source_instance_id(),77u);
}

TEST(NodalMassBinding, WrongEndpointIdentityAndCoordinateBitsRejectWithoutPublishing) {
  const auto shells=Shells(); const SpringInput original(shells);
  const auto valid=Connectors(original,shells.node_count());
  for(unsigned failure=0;failure<2;++failure) {
    auto altered=original;
    // Last endpoint appears twice; preserve internal spring consistency while
    // making the complete connector input disagree with the shell producer.
    for(auto& c:altered.connection) {
      if(failure==0) c.source_node_id[1]+=10;
      else c.position[1].z=-0.;
    }
    const auto invalid=Connectors(altered,shells.node_count()); fe::NodalMassBinding binding;
    const auto report=binding.Initialize(shells,invalid);
    EXPECT_EQ(report.status,failure?Status::PositionMismatch:Status::IdentityMismatch);
    EXPECT_EQ(report.node,shells.node_count()-1); EXPECT_FALSE(binding.prepared());
    EXPECT_EQ(binding.nodes().size(),0u); ASSERT_TRUE(binding.Initialize(shells,valid));
    EXPECT_TRUE(binding.Matches(valid)); EXPECT_FALSE(binding.Matches(invalid));
  }
}

TEST(NodalMassBinding, FullContributorPhysicsAndSourceIdentityAreRequired) {
  const auto shells=Shells(); const SpringInput original(shells);
  const auto valid=Connectors(original,shells.node_count()); fe::NodalMassBinding binding;
  ASSERT_TRUE(binding.Initialize(shells,valid));
  auto changed=original; changed.property.property.stiffness[3]*=2.;
  const auto other=Connectors(changed,shells.node_count()); fe::NodalMassBinding equivalent_mass;
  ASSERT_TRUE(equivalent_mass.Initialize(shells,other));
  EXPECT_EQ(Bytes(binding.totals().mass),Bytes(equivalent_mass.totals().mass));
  EXPECT_FALSE(binding.Matches(other)); EXPECT_FALSE(binding.Matches(equivalent_mass));
  spring::Model foreign; auto foreign_input=original.Input(shells.node_count());
  foreign_input.source_instance_id++;
  ASSERT_TRUE(foreign.Initialize(foreign_input)); EXPECT_FALSE(binding.Matches(foreign));
  auto shell_input=shell_binding_test::Edge(); shell_input.qeph.young_modulus*=2.;
  fe::ShellBatchBinding other_shells;
  ASSERT_EQ(other_shells.Initialize(shell_input).status,fe::ShellBindingStatus::Success);
  EXPECT_FALSE(binding.Matches(other_shells));
}

TEST(NodalMassBinding, LifetimeResourceFailureAndRetryPreserveImmutableComposition) {
  const auto build=[] {
    const auto shells=Shells(); SpringInput input(shells);
    const auto connectors=Connectors(input,shells.node_count()); fe::NodalMassBinding binding;
    EXPECT_TRUE(binding.Initialize(shells,connectors));
    input.property.property.mass_kg=100.; input.connection[1].source_element_id=1;
    return binding;
  };
  const auto held=build(); fe::NodalMassBinding copied(held),moved(std::move(copied));
  EXPECT_EQ(held.nodes().data(),moved.nodes().data()); EXPECT_TRUE(copied.Matches(held));
  const auto shells=Shells(); const SpringInput input(shells);
  const auto connectors=Connectors(input,shells.node_count());
  for(unsigned failure=0;failure<4;++failure) {
    fe::NodalMassLimits limits;
    if(failure==0) limits.max_nodes=shells.node_count()-1;
    if(failure==1) limits.max_host_bytes=held.host_bytes()-1;
    if(failure==2) limits.max_nodes=2049;
    if(failure==3) limits.max_host_bytes=0;
    fe::NodalMassBinding retry;
    EXPECT_EQ(retry.Initialize(shells,connectors,limits).status,Status::ResourceLimit);
    EXPECT_FALSE(retry.prepared()); ASSERT_TRUE(retry.Initialize(shells,connectors));
    EXPECT_TRUE(retry.Matches(held));
    EXPECT_EQ(retry.Initialize(shells,connectors).status,Status::AlreadyInitialized);
    EXPECT_TRUE(retry.Matches(held));
  }
}
} // namespace nodal_mass_test
