#pragma once
#include <gtest/gtest.h>
#include "lib_src/assembly/ShellNodeMap.h"
#include "../t3/mixed_binding/ShellBatchBindingFixture.h"
#include <vector>

namespace nodal_domain_test {
namespace fe=tl::fea;
using S=fe::NodalDomainStatus;
using shell_binding_test::Bytes;
inline std::vector<fe::NodalDomainNode> Nodes(const fe::ShellBatchBinding& binding) {
  std::vector<fe::NodalDomainNode> result;
  for(const auto& node:binding.active_nodes()) result.push_back({node.source_id,node.position});
  return result;
}
inline fe::NodalNodeDomain Domain(const std::vector<fe::NodalDomainNode>& nodes,std::uint64_t source=71) {
  fe::NodalNodeDomain value;
  const auto report=value.Initialize({source,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle());
  EXPECT_EQ(report.status,S::Success)<<report.message;
  return value;
}
inline fe::ShellBatchBinding Binding() {
  fe::ShellBatchBinding binding;
  EXPECT_EQ(binding.Initialize(shell_binding_test::Edge()).status,fe::ShellBindingStatus::Success);
  return binding;
}
} // namespace nodal_domain_test
