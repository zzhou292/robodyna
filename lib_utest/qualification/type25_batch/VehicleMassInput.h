#pragma once
#include "Input.h"
#include "lib_utest/qualification/type25/VehicleModelFixture.h"

namespace type25_batch_test {
// Uniform independent shell cells provide complete native M/J coverage. This is
// a capacity fixture at source-sized counts, not original Yaris geometry.
struct VehicleMassInput {
  fe::ShellBatchBinding shells;
  type25_test::VehicleModelFixture source;
  spring::Model model;
  fe::NodalMassBinding mass;
  VehicleMassInput(std::size_t nodes,std::size_t connections):source(connections,nodes) {
    for(auto& p:source.properties)for(unsigned i=0;i<4;++i){p.property.stiffness[i]=1000;p.property.damping[i]=.01;}
  }
  bool Initialize() {
    const auto n=source.nodes;const auto count=n/4;const auto rem=n%4;
    if(n<8||(rem!=0&&rem!=1))return false;
    std::vector<fe::ShellQephBindingInput> q(count);
    for(std::size_t p=0;p<count;++p) {
      auto& parent=q[p];parent.source_parent_id=10000+p;
      auto& r=parent.reference;r.density=1024;r.thickness=1./32;r.young_modulus=2e6;r.poisson_ratio=.3;
      const tl::math::Vec3 x[4]={{2.*p,0,0},{2.*p+1,0,0},{2.*p+1,1,0},{2.*p,1,0}};
      for(unsigned a=0;a<4;++a) {
        parent.nodes[a]=4*p+a;r.node_ids[a]=static_cast<std::uint32_t>(2000000+4*p+a);r.position[a]=x[a];
      }
    }
    fe::ShellT3BindingInput t;
    if(rem) {
      t.source_parent_id=10000+count;t.nodes={n-3,n-2,n-1};
      auto& r=t.reference;r.density=1024;r.thickness=1./32;r.young_modulus=2e6;r.poisson_ratio=.3;
      r.position[0]=q.back().reference.position[2];r.position[1]=q.back().reference.position[3];
      r.position[2]={2.*(count-1)+.5,2,0};
      for(unsigned a=0;a<3;++a)r.node_ids[a]=2000000+t.nodes[a];
    }
    const fe::ShellBatchCollectionInput in{q.data(),rem?&t:nullptr,count,rem?1u:0u,n};
    const auto built=shells.Initialize(in,fe::ShellHostBindingLimits::Vehicle());
    EXPECT_EQ(built.status,fe::ShellBindingStatus::Success)<<built.message;if(built.status!=fe::ShellBindingStatus::Success)return false;
    for(auto& c:source.connections)for(unsigned a=0;a<2;++a) {
      c.source_node_id[a]=shells.nodes()[c.global_node[a]].source_id;
      c.position[a]=shells.nodes()[c.global_node[a]].position;
    }
    const auto model_result=model.Initialize(source.Input());EXPECT_TRUE(model_result)<<model_result.message;if(!model_result)return false;
    const auto combined=mass.Initialize(shells,model,fe::NodalMassLimits::Vehicle());EXPECT_TRUE(combined)<<combined.message;return bool(combined);
  }
  spring::BatchConfig Config(const fe::NodalStamp& stamp) const {
    auto c=spring::BatchConfig::Vehicle();c.owner=stamp;c.element_count=model.connection_count();c.configuration_id=7;c.qualification_id=8;return c;
  }
};
} // namespace type25_batch_test
