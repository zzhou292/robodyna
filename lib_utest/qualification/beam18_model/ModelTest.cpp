// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <type_traits>
namespace beam18_model_test {
static_assert(std::is_nothrow_copy_constructible_v<b::Model>);
static_assert(!std::is_copy_assignable_v<b::Model>);
TEST(Beam18Model, OwnsCurvesPoolsMidRetainsSourceOrderAndExcludesN3) {
  Fixture f; const auto domain=f.structural.Domain(); const auto m=f.Model(domain);
  ASSERT_EQ(m.parents().size(),2); ASSERT_EQ(m.materials().size(),1);
  EXPECT_TRUE(m.domain()->SharesStorage(domain));
  EXPECT_NE(m.materials()[0].value.curve.plastic_strain,f.x.data());
  const auto expected=f.y[1]; f.y[1]+=10;
  EXPECT_EQ(Bits(m.materials()[0].value.curve.yield_stress_pa[1]),Bits(expected));
  EXPECT_EQ(m.parents()[0].material_index,m.parents()[1].material_index);
  fe::Beam18NodeContributions c; ASSERT_TRUE(c.Initialize(m)); ASSERT_EQ(c.record_count(),4);
  for(std::size_t p=0;p<2;++p) for(unsigned slot=0;slot<2;++slot) {
    const auto& row=c.records()[2*p+slot]; const auto& reference=m.parents()[p].reference;
    EXPECT_EQ(row.model_parent,p); EXPECT_EQ(row.endpoint,slot);
    EXPECT_EQ(row.value.source_node_id,reference.input().source_node_id[slot]);
    EXPECT_EQ(row.value.global_node,m.parents()[p].domain_nodes[slot]);
    EXPECT_EQ(Bits(row.value.coefficients.mass_kg),Bits(reference.endpoint().mass_kg));
    EXPECT_EQ(Bits(row.value.coefficients.native_total_inertia_kg_m2),Bits(reference.endpoint().native_total_inertia_kg_m2));
  }
  b::EndpointContribution sentinel; sentinel.source_element_id=37; const auto before=Bytes(sentinel);
  EXPECT_FALSE(m.Endpoint(0,2,sentinel)); EXPECT_EQ(Bytes(sentinel),before);
  EXPECT_TRUE(c.Matches(m,domain));
}
TEST(Beam18Model, ChangedMidLateDuplicateAndPositionFailWithoutPublication) {
  Fixture f; const auto domain=f.structural.Domain(); b::Model retry; const auto before=Bytes(retry);
  auto y=f.y; y.back()+=1; auto changed=f.rows;
  changed.back().material.curve.yield_stress_pa=y.data(); auto input=f.Input(); input.parents={changed.data(),changed.size()};
  auto r=retry.Initialize(domain,input); EXPECT_EQ(r.status,b::ModelStatus::IdentityMismatch); EXPECT_EQ(r.parent,1);
  EXPECT_EQ(Bytes(retry),before);
  changed=f.rows; auto raw=changed.back().reference.input(); raw.source_element_id=f.rows[0].reference.input().source_element_id;
  ASSERT_EQ(b::InitializeReference(raw,changed.back().reference),b::Status::Success);
  input.parents={changed.data(),changed.size()}; r=retry.Initialize(domain,input);
  EXPECT_EQ(r.status,b::ModelStatus::DuplicateIdentity); EXPECT_EQ(r.parent,1);
  raw=f.rows.back().reference.input(); raw.position[1].x=std::nextafter(raw.position[1].x,1.);
  ASSERT_EQ(b::InitializeReference(raw,changed.back().reference),b::Status::Success);
  r=retry.Initialize(domain,input); EXPECT_EQ(r.status,b::ModelStatus::PositionMismatch); EXPECT_EQ(r.parent,1);
  EXPECT_EQ(Bytes(retry),before); ASSERT_TRUE(retry.Initialize(domain,f.Input()));
  const auto published=Bytes(retry); EXPECT_EQ(retry.Initialize(domain,{}).status,b::ModelStatus::AlreadyInitialized);
  EXPECT_EQ(Bytes(retry),published);
}
TEST(Beam18Model, ExactPeakAndSnapshotBudgetBoundariesAndLifetime) {
  Fixture f; const auto d=f.structural.Domain(); const auto m=f.Model(d);
  auto limits=b::ModelLimits{}; limits.max_host_bytes=m.startup_payload_bytes()-1;
  b::Model retry; EXPECT_EQ(retry.Initialize(d,f.Input(),limits).status,b::ModelStatus::ResourceLimit);
  ++limits.max_host_bytes; ASSERT_TRUE(retry.Initialize(d,f.Input(),limits)); EXPECT_TRUE(retry.Matches(m));
  EXPECT_FALSE(retry.SharesStorage(m)); b::Model copy(m); EXPECT_TRUE(copy.SharesStorage(m));
  fe::Beam18NodeContributions c; ASSERT_TRUE(c.Initialize(m)); auto cap=fe::Beam18ContributionLimits{};
  cap.max_host_bytes=c.startup_payload_bytes()-1; fe::Beam18NodeContributions candidate;
  EXPECT_EQ(candidate.Initialize(m,cap).status,fe::NodalDomainStatus::ResourceLimit);
  ++cap.max_host_bytes; ASSERT_TRUE(candidate.Initialize(m,cap)); EXPECT_TRUE(candidate.Matches(c));
  const auto survivor=[] { Fixture local; const auto domain=local.structural.Domain(); return local.Contributions(domain); }();
  EXPECT_TRUE(survivor.Matches(c)); EXPECT_EQ(survivor.model()->materials()[0].value.curve.yield_stress_pa[1],250e6);
}
TEST(Beam18Model, CountRangeSourceAndPresentOrientationChecks) {
  Fixture f; const auto d=f.structural.Domain(); b::Model model; auto input=f.Input();
  input.parents={reinterpret_cast<const b::ParentInput*>(1),1025};
  EXPECT_EQ(model.Initialize(d,input).status,b::ModelStatus::ResourceLimit);
  input=f.Input(); input.source_instance_id=9; EXPECT_EQ(model.Initialize(d,input).status,b::ModelStatus::InvalidInput);
  f.structural.nodes.push_back({99999,{.1,.2,.3}}); const auto present=f.structural.Domain();
  ASSERT_TRUE(model.Initialize(present,f.Input()));
  f.structural.nodes.back().position.z=std::nextafter(.3,1.); const auto different=f.structural.Domain();
  b::Model wrong; const auto r=wrong.Initialize(different,f.Input());
  EXPECT_EQ(r.status,b::ModelStatus::PositionMismatch); EXPECT_EQ(r.local,2);
}
TEST(Beam18Model, WorkingUnitsKeepEndpointBitsAndDistinctMidIdentity) {
  auto raw=beam18_test::Input(); b::ParentInput row;
  ASSERT_EQ(b::InitializeReference(raw,row.reference),b::Status::Success);
  row.material=beam18_force_test::Material(row.reference);
  fe::NodalDomainNode nodes[2]{{raw.source_node_id[1],row.reference.geometry().endpoint_m[1]},
      {raw.source_node_id[0],row.reference.geometry().endpoint_m[0]}};
  fe::NodalNodeDomain domain; ASSERT_TRUE(domain.Initialize({77,nodes,2})); b::Model model;
  ASSERT_TRUE(model.Initialize(domain,{77,{&row,1},b::ModelProfile::CircularFourPointLaw44V1}));
  b::EndpointContribution endpoint; ASSERT_TRUE(model.Endpoint(0,0,endpoint)); EXPECT_EQ(endpoint.global_node,1);
  EXPECT_EQ(Bits(endpoint.coefficients.mass_kg),Bits(row.reference.native_mass().endpoint_mass*1000));
  EXPECT_EQ(Bits(endpoint.coefficients.native_total_inertia_kg_m2),Bits(row.reference.native_mass().endpoint_total_inertia*.001));
  EXPECT_EQ(model.materials()[0].value.material.native_units,tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond);
}
} // namespace beam18_model_test
