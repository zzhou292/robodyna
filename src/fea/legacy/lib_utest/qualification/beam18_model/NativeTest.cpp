// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../beam18_reference/NativeOracle.h"
#include "lib_src/assembly/NodalDomainIdentity.h"
#ifdef BEAM18_MODEL_ORIGINAL
#include "OriginalFixture.h"
#endif
namespace beam18_model_test {
TEST(Beam18ModelNative, NativeEndpointValuesReachSnapshotInBothUnits) {
  for(bool working:{false,true}) {
    auto reference=beam18_force_test::Reference();
    if(working) ASSERT_EQ(b::InitializeReference(beam18_test::Input(),reference),b::Status::Success);
    b::ParentInput row{reference,beam18_force_test::Material(reference)};
    fe::NodalDomainNode nodes[2]{{reference.input().source_node_id[1],reference.geometry().endpoint_m[1]},
        {reference.input().source_node_id[0],reference.geometry().endpoint_m[0]}};
    fe::NodalNodeDomain domain; ASSERT_TRUE(domain.Initialize({19,nodes,2})); b::Model model;
    ASSERT_TRUE(model.Initialize(domain,{19,{&row,1},b::ModelProfile::CircularFourPointLaw44V1}));
    const auto native=beam18_test::Native(reference.input()); beam18_test::Compare(reference,native);
    fe::Beam18NodeContributions contribution; ASSERT_TRUE(contribution.Initialize(model));
    for(const auto& record:contribution.records()) {
      EXPECT_NEAR(record.value.coefficients.mass_kg,native.values[27],3e-13*std::abs(native.values[27]));
      EXPECT_NEAR(record.value.coefficients.native_total_inertia_kg_m2,native.values[28],3e-13*std::abs(native.values[28]));
    }
  }
}
#ifdef BEAM18_MODEL_ORIGINAL
TEST(Beam18ModelNative, Original142SourceOrderCurveOwnershipAndNativeLedgerScatter) {
  coefficient_test::Fixture structural;
  std::vector<b::ParentInput> rows;
  std::vector<beam18_test::NativeResult> native;
  std::vector<double> x(std::begin(law44_solid_test::X),std::end(law44_solid_test::X));
  std::vector<double> y(std::begin(law44_solid_test::Y),std::end(law44_solid_test::Y));
  for(unsigned p=0;p<std::size(beam18_test::original::Cells);++p) {
    const auto input=beam18_test::original::Input(p); SCOPED_TRACE(input.source_element_id);
    b::ParentInput row; ASSERT_EQ(b::InitializeReference(input,row.reference),b::Status::Success);
    row.material=beam18_force_test::Material(row.reference);
    row.material.curve={x.data(),y.data(),static_cast<std::uint32_t>(x.size())}; rows.push_back(row);
    native.push_back(beam18_test::Native(input)); beam18_test::Compare(row.reference,native.back());
    ASSERT_FALSE(HasFatalFailure());
    for(unsigned local=0;local<3;++local) {
      const auto id=input.source_node_id[local]; const auto position=tl::math::fixed3::Scale(input.position[local],.001);
      const auto found=std::find_if(structural.nodes.begin(),structural.nodes.end(),[&](auto n){return n.source_id==id;});
      if(found==structural.nodes.end()) structural.nodes.push_back({id,position});
      else EXPECT_TRUE(fe::nodal_domain_detail::SamePosition(found->position,position));
    }
  }
  ASSERT_EQ(rows.size(),142); const auto domain=structural.Domain(); const auto map=structural.Map(domain);
  b::Model model; ASSERT_TRUE(model.Initialize(domain,{1,{rows.data(),rows.size()},b::ModelProfile::CircularFourPointLaw44V1}));
  ASSERT_EQ(model.materials().size(),4); x.clear(); y.clear(); rows.clear();
  fe::Beam18NodeContributions snapshot; ASSERT_TRUE(snapshot.Initialize(model)); ASSERT_EQ(snapshot.record_count(),284);
  fe::NodalCoefficientLedger ledger,base;
  ASSERT_TRUE(base.Initialize({&map})); ASSERT_TRUE(ledger.InitializeWithBeams({{{&map}},&snapshot}));
  std::vector<double> mass(domain.node_count()),inertia(domain.node_count());
  std::vector<unsigned> counts(domain.node_count());
  for(std::size_t p=0;p<model.parents().size();++p) {
    EXPECT_EQ(model.parents()[p].reference.input().source_element_id,beam18_test::original::Cells[p].eid);
    for(unsigned local=0;local<2;++local) {
      const auto node=model.parents()[p].domain_nodes[local];
      mass[node]+=native[p].values[27]; inertia[node]+=native[p].values[28]; ++counts[node];
    }
  }
  std::size_t endpoint_nodes=0;
  for(std::size_t n=0;n<domain.node_count();++n) {
    const auto& row=ledger.nodes()[n]; if(counts[n]) ++endpoint_nodes;
    EXPECT_EQ(row.occurrences.beam18,counts[n]);
    EXPECT_NEAR(row.coefficients.beam18.mass,mass[n],3e-13*std::max(mass[n],1e-30));
    EXPECT_NEAR(row.coefficients.beam18.isotropic_inertia,inertia[n],3e-13*std::max(inertia[n],1e-30));
  }
  EXPECT_EQ(endpoint_nodes,146); EXPECT_EQ(ledger.scope().occurrences.beam18,284);
  EXPECT_EQ(domain.node_count(),structural.shells.node_count()+2+147);
  RecordProperty("model_retained_bytes",std::to_string(model.owned_payload_bytes()));
  RecordProperty("model_startup_bytes",std::to_string(model.startup_payload_bytes()));
  RecordProperty("ledger_startup_bytes",std::to_string(ledger.startup_payload_bytes()));
}
#endif
} // namespace beam18_model_test
