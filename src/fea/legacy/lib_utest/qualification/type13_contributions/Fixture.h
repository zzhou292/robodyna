#pragma once
#include "lib_src/assembly/Type13NodeContributions.h"
#include "../type13_model/Fixture.h"
#include <array>
#include <cstring>

namespace type13_contribution_test {
namespace fe=tl::fea;
namespace t=fe::type13;
using S=fe::NodalDomainStatus;
using type13_model_test::Fixture;
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> result{};
  std::memcpy(result.data(),&value,sizeof(T));
  return result;
}
inline t::Model Model(const Fixture& fixture) {
  t::Model model; EXPECT_TRUE(model.Initialize(fixture.Input())); return model;
}
inline std::vector<fe::NodalDomainNode> Nodes(const t::Model& model) {
  std::vector<fe::NodalDomainNode> result(model.global_node_count());
  for(std::size_t n=0;n<model.node_count();++n) {
    const auto& node=model.nodes()[n];
    if(node.global_node!=SIZE_MAX) result[node.global_node]={node.source_id,
        tl::math::fixed3::Scale(node.position_native,model.units().length_to_m)};
  }
  for(std::size_t n=0;n<result.size();++n) if(!result[n].source_id) result[n]={1000+n,{0,0,0}};
  return result;
}
inline fe::NodalNodeDomain Domain(const std::vector<fe::NodalDomainNode>& nodes,std::uint64_t source=1) {
  fe::NodalNodeDomain domain;
  EXPECT_TRUE(domain.Initialize({source,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
  return domain;
}
inline void Same(const t::EndpointContribution& a,const t::EndpointContribution& b) {
  EXPECT_EQ(a.source_element_id,b.source_element_id); EXPECT_EQ(a.source_property_id,b.source_property_id);
  EXPECT_EQ(a.source_node_id,b.source_node_id); EXPECT_EQ(a.global_node,b.global_node);
  EXPECT_EQ(Bytes(a.coefficients.mass_kg),Bytes(b.coefficients.mass_kg));
  EXPECT_EQ(Bytes(a.coefficients.isotropic_inertia_kg_m2),Bytes(b.coefficients.isotropic_inertia_kg_m2));
  EXPECT_EQ(Bytes(a.coefficients.added_inertia_kg_m2),Bytes(b.coefficients.added_inertia_kg_m2));
}
inline void Check(const fe::Type13NodeContributions& adapter,const t::Model& model) {
  ASSERT_EQ(adapter.record_count(),2*model.connection_count());
  for(std::size_t e=0;e<model.connection_count();++e) for(unsigned local=0;local<2;++local) {
    const auto& record=adapter.records()[2*e+local];
    EXPECT_EQ(record.model_connection,e); EXPECT_EQ(record.endpoint,local);
    t::EndpointContribution expected; ASSERT_TRUE(model.Endpoint(e,local,expected));
    Same(record.value,expected);
    const auto& position=adapter.domain()->nodes()[record.value.global_node].position;
    const auto prepared=model.startup(e)->reference.position_m[local];
    EXPECT_EQ(Bytes(position.x),Bytes(prepared.x)); EXPECT_EQ(Bytes(position.y),Bytes(prepared.y));
    EXPECT_EQ(Bytes(position.z),Bytes(prepared.z));
  }
}
} // namespace type13_contribution_test
