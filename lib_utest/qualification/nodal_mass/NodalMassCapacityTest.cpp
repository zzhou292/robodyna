#include "NodalMassTestSupport.h"
#include <vector>

namespace nodal_mass_test {
TEST(NodalMassBinding, FullActiveNodeExtentIncludesTheFinalSparseConnectorEndpoint) {
  std::vector<fe::ShellQephBindingInput> parents(512);
  const auto first=shell_binding_test::Edge();
  for(std::size_t p=0;p<parents.size();++p) {
    auto& parent=parents[p]; parent.source_parent_id=10000+p; parent.reference=first.qeph;
    for(unsigned n=0;n<4;++n) {
      parent.nodes[n]=4*p+n; parent.reference.node_ids[n]=1000+4*p+n;
      parent.reference.position[n].x+=2.*p;
    }
  }
  fe::ShellBatchCollectionInput input;
  input.qeph=parents.data(); input.qeph_count=parents.size(); input.node_count=2048;
  fe::ShellBatchBinding shells;
  ASSERT_EQ(shells.Initialize(input,fe::ShellHostBindingLimits{}).status,fe::ShellBindingStatus::Success);
  const SpringInput spring_input(shells); const auto connectors=Connectors(spring_input,2048);
  fe::NodalMassBinding binding; ASSERT_TRUE(binding.Initialize(shells,connectors));
  EXPECT_EQ(binding.node_count(),2048u); EXPECT_EQ(binding.nodes()[2047].source_id,3047u);
  Near(binding.nodes()[2047].coefficients.connector_mass,spring_input.property.property.mass_kg);
  EXPECT_EQ(binding.nodes()[2046].coefficients.connector_mass,0.);
  Exact(binding.nodes()[2047].coefficients.shell,shells.nodes()[2047].native);
  EXPECT_TRUE(binding.Matches(connectors));
  RecordProperty("host_payload_bytes",std::to_string(binding.host_bytes()));
}
} // namespace nodal_mass_test
