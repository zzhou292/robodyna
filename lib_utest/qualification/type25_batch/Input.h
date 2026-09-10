// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type25/Type25Batch.h"
#include "lib_src/elements/type25/Type25Math.h"
#include "lib_src/elements/type25/Type25Model.h"
#include "lib_src/assembly/NodalMassBinding.h"
#include <gtest/gtest.h>
#include <vector>

namespace type25_batch_test {
namespace fe=tl::fea;
namespace spring=fe::type25;
struct Input {
  fe::ShellBatchBinding shells;
  spring::PropertyInput property;
  std::vector<spring::ConnectionInput> connections;
  spring::Model model;
  fe::NodalMassBinding mass;
  bool Initialize(std::size_t count=129) {
    fe::ShellBatchBindingInput s;s.node_count=5;s.qeph_nodes={0,1,2,3};s.t3_nodes={1,4,2};
    const tl::math::Vec3 x[5]={{0,0,0},{1,0,0},{1,1,0},{0,1,0},{1.75,.25,0}};
    s.qeph.density=s.t3.density=1024;s.qeph.thickness=s.t3.thickness=1./32;
    s.qeph.young_modulus=s.t3.young_modulus=2e6;s.qeph.poisson_ratio=s.t3.poisson_ratio=.3;
    for(unsigned i=0;i<4;++i){s.qeph.position[i]=x[i];s.qeph.node_ids[i]=100+i;}
    for(unsigned i=0;i<3;++i){s.t3.position[i]=x[s.t3_nodes[i]];s.t3.node_ids[i]=100+s.t3_nodes[i];}
    if(shells.Initialize(s).status!=fe::ShellBindingStatus::Success)return false;
    property.source_property_id=8001;auto& p=property.property;p.mass_kg=.001;p.isotropic_inertia_kg_m2=1e-8;
    for(unsigned i=0;i<4;++i) {
      p.stiffness[i]=1000;p.damping[i]=.01;p.failure_negative[i]=-1e20;p.failure_positive[i]=1e20;
      p.failure_weight[i]=1;p.failure_exponent[i]=2;
    }
    connections.resize(count);
    for(std::size_t c=0;c<count;++c) {
      auto& input=connections[c];input.source_element_id=(std::uint64_t{1}<<54)+c+1;
      // The last node appears only in the final connection, so a fault there
      // tests ordered late rejection beyond every earlier complete candidate.
      input.global_node[0]=c%2;input.global_node[1]=c+1==count?4:3;
      for(unsigned e=0;e<2;++e) {
        const auto n=input.global_node[e];input.source_node_id[e]=shells.nodes()[n].source_id;
        input.position[e]=shells.nodes()[n].position;
      }
    }
    spring::ModelInput request;request.source_instance_id=77;request.global_node_count=5;
    request.source_units={1,1,1};request.properties=&property;request.property_count=1;
    request.connections=connections.data();request.connection_count=count;
    return bool(model.Initialize(request))&&bool(mass.Initialize(shells,model));
  }
  spring::BatchConfig Config(fe::NodalStamp stamp) const {
    spring::BatchConfig c;c.owner=stamp;c.element_count=model.connection_count();c.configuration_id=7;c.qualification_id=8;return c;
  }
};
} // namespace type25_batch_test
