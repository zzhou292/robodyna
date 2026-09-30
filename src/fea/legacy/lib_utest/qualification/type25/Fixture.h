// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type25/Type25Model.h"
#include "lib_src/elements/type25/Type25Frame.h"
#include <array>

namespace type25_test {
namespace spring=tl::fea::type25;
inline spring::Property Property() {
  spring::Property p;p.mass_kg=.001;p.isotropic_inertia_kg_m2=1e-8;
  for(unsigned i=0;i<4;++i) {
    p.stiffness[i]=i<2?1e8:1000;p.failure_negative[i]=-1e30;p.failure_positive[i]=1e30;
    p.failure_weight[i]=1;p.failure_exponent[i]=2;
  }
  return p;
}
struct Fixture {
  std::array<spring::PropertyInput,2> properties{{{41,Property()},{42,Property()}}};
  std::array<spring::ConnectionInput,3> connections{};
  Fixture() {
    for(unsigned i=0;i<3;++i) {
      auto& c=connections[i];c.source_element_id=101+i;c.source_node_id[0]=201+2*i;c.source_node_id[1]=202+2*i;
      c.global_node[0]=2*i;c.global_node[1]=2*i+1;c.property_index=i%2;
      c.position[0]={.01*i,.02*i,.03*i};c.position[1]={.01*i+.007,.02*i+.002,.03*i+.001};
    }
  }
  spring::ModelInput Input() const {
    spring::ModelInput in;in.source_instance_id=17;in.global_node_count=6;
    in.properties=properties.data();in.property_count=properties.size();
    in.connections=connections.data();in.connection_count=connections.size();in.source_units={1000,.001,1};return in;
  }
};
} // namespace type25_test
