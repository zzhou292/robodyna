#include "ActualSupport.h"
#include "lib_utest/qualification/solid_law44_point/TestSupport.h"
#include "lib_utest/qualification/solid18_reference/TestSupport.h"
#include "lib_utest/qualification/solid24_reference/JacobianSupport.h"
#include "lib_utest/qualification/solid6z_reference/TestSupport.h"

namespace crash::modelio::solid_source::test {
namespace {
constexpr auto RearPolicy=Policy::OriginalAdhesive18ExtendedRubberRearLaw44V3;
const VehicleSolidSource& Rear() {
    static const auto result=VehicleSolidSource::Prepare(vehicle::test::Canonical(),MemberBytes(),RearPolicy);
    return result;
}
}
TEST(VehicleRearSolidSource, Original306CellsKeepEightSourceSlotsAndActualNativeMaterials) {
    const auto& source=Rear();const auto& data=source.data();
    EXPECT_EQ(data.parts.size(),15u);EXPECT_EQ(data.rows.size(),3555u);
    EXPECT_EQ(data.solid18.size(),908u);EXPECT_EQ(data.solid24.size(),1991u);
    EXPECT_EQ(data.solid6z.size(),350u);ASSERT_EQ(data.solid18_law44.size(),306u);
    EXPECT_EQ(data.original_solids,15234u);EXPECT_EQ(data.outside_solids,11679u);
    const auto nodes=detail::Decode<std::uint64_t>(source.canonical().data(),"node_ids");
    const auto positions=detail::Decode<double>(source.canonical().data(),"node_positions");
    unsigned bars=0,tubes=0,repeated=0;
    for(const auto& row:data.rows) {
        if(row.family!=Family::Solid18Law44)continue;
        SCOPED_TRACE(row.element_id);
        const auto& part=data.parts.at(row.part_index);
        const auto& ref=data.solid18_law44.at(row.reference_index);
        ASSERT_TRUE(ref.prepared());EXPECT_EQ(part.material_law,MaterialLaw::Law44);
        const bool bar=part.id==2000016;
        EXPECT_TRUE(bar||part.id==2000392);bar?++bars:++tubes;
        const auto native=law44_solid_test::Parameters(bar);
        EXPECT_EQ(part.law44.material.young_pa,native.material.young_pa);
        EXPECT_EQ(part.law44.material.poisson_ratio,native.material.poisson_ratio);
        EXPECT_EQ(part.law44.material.density_kg_m3,native.material.density_kg_m3);
        EXPECT_EQ(part.law44.material.rate_c_per_s,native.material.rate_c_per_s);
        EXPECT_EQ(part.law44.material.rate_p,native.material.rate_p);
        EXPECT_EQ(part.law44.material.cutoff_hz,native.material.cutoff_hz);
        EXPECT_EQ(part.law44.material.native_units,native.material.native_units);
        ASSERT_EQ(part.law44.curve.count,native.curve.count);
        EXPECT_EQ(part.law44.curve.plastic_strain,data.rear_plastic_strain.data());
        EXPECT_EQ(part.law44.curve.yield_stress_pa,data.rear_yield_stress_pa.data());
        for(unsigned i=0;i<native.curve.count;++i) {
            EXPECT_EQ(output::Bits(part.law44.curve.plastic_strain[i]),output::Bits(native.curve.plastic_strain[i]));
            EXPECT_EQ(output::Bits(part.law44.curve.yield_stress_pa[i]),output::Bits(native.curve.yield_stress_pa[i]));
        }
        const auto& input=ref.input();
        EXPECT_EQ(input.source_element_id,row.element_id);EXPECT_EQ(input.source_part_id,row.part_id);
        EXPECT_EQ(input.source_material_id,part.material_id);EXPECT_EQ(input.source_section_id,part.section_id);
        EXPECT_EQ(input.profile.material_law,44u);EXPECT_EQ(input.profile.pressure,1u);
        EXPECT_EQ(input.profile.small_strain,2u);EXPECT_EQ(part.converter_isolid,18u);
        for(unsigned k=0;k<8;++k) {
            const auto node=row.canonical_nodes[k];ASSERT_LT(node,nodes.size());
            EXPECT_EQ(input.source_node_id[k],row.raw_node_ids[k]);EXPECT_EQ(input.source_node_id[k],nodes[node]);
            EXPECT_EQ(output::Bits(input.position_m[k].x),output::Bits(positions[3*node]));
            EXPECT_EQ(output::Bits(input.position_m[k].y),output::Bits(positions[3*node+1]));
            EXPECT_EQ(output::Bits(input.position_m[k].z),output::Bits(positions[3*node+2]));
            EXPECT_GT(ref.mass().source_nodal_mass_kg[k],0);
        }
        if(ref.topology()==tl::fea::solid18::law44::SourceTopology::RepeatedPairs56And78) {
            ++repeated;EXPECT_EQ(row.raw_node_ids[4],row.raw_node_ids[5]);
            EXPECT_EQ(row.raw_node_ids[6],row.raw_node_ids[7]);
        }
    }
    EXPECT_EQ(bars,210u);EXPECT_EQ(tubes,96u);EXPECT_EQ(repeated,109u);
    RecordProperty("selected_solids",data.rows.size());RecordProperty("rear_cells",306);
    RecordProperty("forecast_bytes",source.forecast().total_bytes);
    RecordProperty("owned_payload_bytes",data.owned_payload_bytes);
}
TEST(VehicleRearSolidSource, EveryOld3249ReferenceAndSourceRecordKeepsExactOrder) {
    const auto& old=Extended().data();const auto& next=Rear().data();std::size_t cursor=0;
    for(const auto& row:next.rows) {
        if(row.family==Family::Solid18Law44)continue;
        ASSERT_LT(cursor,old.rows.size());const auto& prior=old.rows[cursor++];
        EXPECT_EQ(row.element_id,prior.element_id);EXPECT_EQ(row.part_id,prior.part_id);
        EXPECT_EQ(row.raw_card,prior.raw_card);EXPECT_EQ(row.raw_node_ids,prior.raw_node_ids);
        EXPECT_EQ(row.canonical_nodes,prior.canonical_nodes);EXPECT_EQ(row.six_to_raw,prior.six_to_raw);
        EXPECT_EQ(row.family,prior.family);EXPECT_EQ(row.source_line,prior.source_line);
        if(row.family==Family::Solid18) {
            const auto& a=next.solid18.at(row.reference_index);const auto& b=old.solid18.at(prior.reference_index);
            InputBits(a.input(),b.input(),8);EqualBits(solid18_test::Values(a),solid18_test::Values(b));
        } else if(row.family==Family::Solid24) {
            const auto& a=next.solid24.at(row.reference_index);const auto& b=old.solid24.at(prior.reference_index);
            InputBits(a.input(),b.input(),8);EqualBits(solid24_test::Values(a),solid24_test::Values(b));
            EqualBits(solid24_test::JacobianValues(a),solid24_test::JacobianValues(b));
        } else {
            const auto& a=next.solid6z.at(row.reference_index);const auto& b=old.solid6z.at(prior.reference_index);
            InputBits(a.input(),b.input(),6);EqualBits(solid6z_test::Values(a),solid6z_test::Values(b));
        }
    }
    EXPECT_EQ(cursor,3249u);EqualBits(next.plastic_strain,old.plastic_strain);
    EqualBits(next.yield_stress_pa,old.yield_stress_pa);
    const auto copy=Rear();EXPECT_EQ(&copy.data(),&Rear().data());
}
TEST(VehicleRearSolidSource, WrongCurveRateBranchAndCountsRejectWithoutChangingSource) {
    const auto& original=Rear().data();
    for(bool rate:{false,true}) {
        auto data=original;auto part=*std::find_if(data.parts.begin(),data.parts.end(),
            [](const Part& p){return p.id==2000392;});
        auto source=data.sources.at(part.sources[2]);
        auto& control=source.cards[1].second;control.resize(80,' ');
        control.replace(rate?40:20,10,rate?"         1":"   2100010");
        EXPECT_THROW(detail::ReadRearMaterial(part,source,data),std::runtime_error);
    }
    Limits limits;limits.parents=3554;
    EXPECT_THROW(VehicleSolidSource::Preflight(vehicle::test::Canonical(),RearPolicy,limits),std::runtime_error);
    const auto forecast=VehicleSolidSource::Preflight(vehicle::test::Canonical(),RearPolicy);
    limits=Limits{};limits.host_bytes=forecast.total_bytes-1;
    EXPECT_THROW(VehicleSolidSource::Preflight(vehicle::test::Canonical(),RearPolicy,limits),std::runtime_error);
    ++limits.host_bytes;
    EXPECT_EQ(VehicleSolidSource::Preflight(vehicle::test::Canonical(),RearPolicy,limits).total_bytes,forecast.total_bytes);
    EXPECT_EQ(original.solid18_law44.size(),306u);
}
} // namespace crash::modelio::solid_source::test
