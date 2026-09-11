#include "../Source.h"
#include "modelio/vehicle_sections/tests/RigidSourceSupport.h"
#include "lib_src/assembly/ElementMassContributions.h"
#include <set>

namespace crash::modelio::vehicle::rigid_part::point_mass::test {
TEST(RigidPointMassSource, CompleteOriginalCensusAndAll54MappedZeroJContributions) {
    const auto& rigid=*vehicle::test::RigidResolution().rigid_source();
    const auto source=Source::Prepare(rigid);
    const auto& data=source.data();
    ASSERT_EQ(data.records.size(),155u);ASSERT_EQ(data.consumed.size(),54u);
    EXPECT_EQ(data.outside_records,101u);
    EXPECT_EQ(&source.rigid_source().data(),&rigid.data());
    std::set<std::uint64_t> ids,nodes;
    long double total=0;
    std::vector<tl::fea::NodalDomainNode> mapped;
    std::vector<tl::fea::ElementMassSource> mass_inputs;
    const auto& canonical=rigid.source().canonical().data();
    const auto& node_array=output::full_shell::source::FindArray(canonical,"node_ids");
    const auto& position_array=output::full_shell::source::FindArray(canonical,"node_positions");
    const auto source_ids=output::arrays::Decode<std::uint64_t>(node_array.descriptor,node_array.bytes);
    const auto positions=output::arrays::Decode<double>(position_array.descriptor,position_array.bytes);
    for(const auto& selected:data.consumed) {
        ASSERT_LT(selected.record,data.records.size());
        const auto& row=data.records[selected.record];const auto& v=row.value;
        EXPECT_TRUE(ids.insert(v.source_element_id).second);EXPECT_TRUE(nodes.insert(v.source_node_id).second);
        ASSERT_LT(row.retained_source,rigid.data().sources.size());
        const auto& block=rigid.data().sources[row.retained_source];
        const auto& card=block.cards.at(v.source_card_index).second;
        EXPECT_EQ(v.source_element_id,std::stoull(card.substr(0,8)));
        EXPECT_EQ(v.source_node_id,std::stoull(card.substr(8,8)));
        EXPECT_EQ(output::Bits(v.supplied_mass_source),output::Bits(std::stod(card.substr(16,16))));
        EXPECT_EQ(v.source_block_line,block.block.first_line);
        EXPECT_EQ(selected.root_index,rigid.root_for_node(v.source_node_id));
        total+=static_cast<long double>(v.supplied_mass_source)*1000;
        const auto found=std::lower_bound(source_ids.begin(),source_ids.end(),v.source_node_id);
        ASSERT_NE(found,source_ids.end());ASSERT_EQ(*found,v.source_node_id);
        const auto n=std::size_t(found-source_ids.begin());
        const auto local=mapped.size();
        mapped.push_back({v.source_node_id,{positions[3*n],positions[3*n+1],positions[3*n+2]}});
        mass_inputs.push_back({v.source_element_id,v.source_node_id,local,v.supplied_mass_source});
    }
    EXPECT_NEAR(static_cast<double>(total),7.380358,2e-14);
    tl::fea::NodalNodeDomain domain;
    const auto instance=rigid.topology().source_instance_id();
    ASSERT_TRUE(domain.Initialize({instance,mapped.data(),mapped.size()}));
    tl::fea::ElementMassContributions coefficients;
    ASSERT_TRUE(coefficients.Initialize(domain,{instance,canonical.inputs.units.mass_to_kg,
        mass_inputs.data(),mass_inputs.size()}));
    ASSERT_EQ(coefficients.records().size(),54u);
    for(std::size_t i=0;i<coefficients.records().size();++i) {
        EXPECT_EQ(output::Bits(coefficients.records()[i].mass_kg),
                  output::Bits(data.records[data.consumed[i].record].value.supplied_mass_kg));
        EXPECT_EQ(output::Bits(coefficients.records()[i].isotropic_inertia_kg_m2()),output::Bits(0.0));
    }
    auto cap=Limits{};cap.host_bytes=Source::Forecast(rigid)-1;
    EXPECT_THROW(Source::Prepare(rigid,cap),std::runtime_error);
    ++cap.host_bytes;const auto retry=Source::Prepare(rigid,cap);
    EXPECT_EQ(retry.data().startup_budget_bytes,cap.host_bytes);
    EXPECT_EQ(retry.data().consumed.size(),54u);
    RecordProperty("complete_source_forecast_bytes",data.startup_budget_bytes);
    // This 54-node coefficient fixture is not the global owner or a complete
    // rigid aggregate. Required original rubber and constraint stages are separate.
}
}
