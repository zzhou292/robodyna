// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "../type13/source_fixture/YarisType13SourceFixture.h"
#include <iterator>

namespace type13_model_test {
namespace original=yaris_type13_fixture;
struct Source {
  type13_test::Fixture material;
  t::ModelPropertyInput property{2000486,material.Input()};
  std::vector<t::ModelNode> nodes;
  std::vector<t::ModelConnection> connections;
  Source() {
    for(std::size_t n=0;n<std::size(original::Nodes);++n) {
      const auto& source=original::Nodes[n];
      nodes.push_back({source.id,n?n-1:SIZE_MAX,{source.native[0],source.native[1],source.native[2]}});
    }
    for(const auto& source:original::Beams) {
      t::ModelConnection c{source.id,0,{source.nodes[0],source.nodes[1],source.nodes[2]}};
      for(unsigned local=0;local<4;++local)c.endpoint_release[local]=source.releases[local];
      connections.push_back(c);
    }
  }
  t::ModelInput Input() const { return {1,{1000,.001,1},nodes.data(),&property,connections.data(),
      nodes.size(),1,connections.size(),7493}; }
};
} // namespace type13_model_test
