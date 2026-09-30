#pragma once
#include "Fixture.h"
#include <vector>

namespace type25_test {
struct VehicleModelFixture {
  std::array<spring::PropertyInput,2> properties=Fixture{}.properties;
  std::vector<spring::ConnectionInput> connections;
  std::size_t nodes=0;
  VehicleModelFixture(std::size_t count=2828,std::size_t node_count=359785):connections(count),nodes(node_count) {
    for(std::size_t e=0;e<count;++e) {
      auto& c=connections[e];c.source_element_id=1000000+e;c.property_index=e%2;
      c.global_node[0]=e+1==count?nodes-4:2*e;c.global_node[1]=e+1==count?nodes-1:2*e+1;
      for(unsigned k=0;k<2;++k) {
        const auto n=c.global_node[k];c.source_node_id[k]=2000000+n;
        const auto cell=n/4,local=n%4;
        c.position[k]={2.*cell+((local==1||local==2)?1.:0.),local>=2?1.:0.,0.};
      }
    }
  }
  spring::ModelInput Input() const {
    spring::ModelInput in;in.source_instance_id=917;in.global_node_count=nodes;in.source_units={1,1,1};
    in.connections=connections.data();in.connection_count=connections.size();
    in.properties=properties.data();in.property_count=properties.size();in.limits=spring::ModelLimits::Vehicle();return in;
  }
};
} // namespace type25_test
