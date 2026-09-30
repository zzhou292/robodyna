#include "../Internal.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include "lib_utest/qualification/beam18_force/TestSupport.h"
#include "OriginalFixture.h"
#include <set>
namespace crash::modelio::beam18::test {
namespace {
constexpr auto Profile=Policy::OriginalCircularFourPointLaw44V1;
const std::string& MemberBytes(){static const auto bytes=output::ReadBounded(std::getenv("ROBO_STATIC_MEMBER"),42846753);return bytes;}
const Source& Original(){static const auto s=Source::Prepare(vehicle::test::Canonical(),MemberBytes(),Profile);return s;}
void InputBits(const native::Input& a,const native::Input& b) {
    EXPECT_EQ(a.source_element_id,b.source_element_id);EXPECT_EQ(a.source_part_id,b.source_part_id);
    EXPECT_EQ(a.source_section_id,b.source_section_id);EXPECT_EQ(a.source_material_id,b.source_material_id);
    EXPECT_EQ(a.profile,b.profile);EXPECT_EQ(a.units,b.units);EXPECT_EQ(a.local,b.local);
    for(unsigned n=0;n<3;++n){EXPECT_EQ(a.source_node_id[n],b.source_node_id[n]);
        EXPECT_EQ(output::Bits(a.position[n].x),output::Bits(b.position[n].x));
        EXPECT_EQ(output::Bits(a.position[n].y),output::Bits(b.position[n].y));
        EXPECT_EQ(output::Bits(a.position[n].z),output::Bits(b.position[n].z));}
    for(unsigned n=0;n<4;++n)EXPECT_EQ(a.release[n],b.release[n]);
    EXPECT_EQ(output::Bits(a.radius),output::Bits(b.radius));EXPECT_EQ(output::Bits(a.density),output::Bits(b.density));
    EXPECT_EQ(output::Bits(a.young),output::Bits(b.young));EXPECT_EQ(output::Bits(a.poisson),output::Bits(b.poisson));
}
}
TEST(Beam18SourceOriginal, All142QualifiedReferencesAndOriginal46PointMaterialArePreserved) {
    const auto& s=Original();const auto& d=s.data();
    ASSERT_EQ(d.rows.size(),142u);ASSERT_EQ(d.parts.size(),4u);ASSERT_EQ(d.nodes.size(),147u);
    EXPECT_EQ(d.canonical_endpoints.size(),146u);EXPECT_EQ(d.original_beams,4685u);EXPECT_EQ(d.outside_beams,4543u);
    for(unsigned i=0;i<142;++i) {
        const auto& row=d.rows[i];SCOPED_TRACE(row.element_id);
        const auto expected=beam18_test::original::Input(i);
        InputBits(row.reference.input(),expected);
        native::Reference ref;ASSERT_EQ(native::InitializeReference(expected,ref),native::Status::Success);
        EXPECT_EQ(beam18_test::Values(row.reference),beam18_test::Values(ref));
        const auto material=beam18_force_test::Material(ref);
        const auto& actual=d.parts.at(row.part_index).material;
        ASSERT_EQ(actual.curve.count,46u);
        EXPECT_EQ(actual.curve.plastic_strain,d.plastic_strain.data());
        EXPECT_EQ(actual.curve.yield_stress_pa,d.yield_stress_pa.data());
        for(unsigned p=0;p<46;++p){EXPECT_EQ(output::Bits(actual.curve.plastic_strain[p]),output::Bits(material.curve.plastic_strain[p]));
            EXPECT_EQ(output::Bits(actual.curve.yield_stress_pa[p]),output::Bits(material.curve.yield_stress_pa[p]));}
    }
    RecordProperty("beams",d.rows.size());RecordProperty("physical_endpoints",d.canonical_endpoints.size());
    RecordProperty("forecast_bytes",std::to_string(s.forecast().total_bytes));
    RecordProperty("owned_bytes",std::to_string(d.owned_payload_bytes));
}
TEST(Beam18SourceOriginal, ExactCapsMemberAuthenticationAndOrientationOnlyOwnership) {
    const auto& s=Original();const auto& canonical=s.canonical();
    auto cap=Limits{};cap.parents=141;EXPECT_THROW(Source::Preflight(canonical,Profile,cap),std::runtime_error);
    cap={};cap.nodes=146;EXPECT_THROW(Source::Preflight(canonical,Profile,cap),std::runtime_error);
    cap={};cap.host_bytes=s.forecast().total_bytes-1;EXPECT_THROW(Source::Preflight(canonical,Profile,cap),std::runtime_error);
    ++cap.host_bytes;EXPECT_EQ(Source::Preflight(canonical,Profile,cap).total_bytes,s.forecast().total_bytes);
    EXPECT_THROW(Source::Preflight(canonical,static_cast<Policy>(99)),std::runtime_error);
    EXPECT_THROW(Source::Prepare(canonical,"bad member",Profile),std::runtime_error);
    std::set<std::uint32_t> physical;for(const auto& row:s.data().rows)for(unsigned k=0;k<2;++k)
        physical.insert(s.data().nodes[row.nodes[k]].canonical_index);
    EXPECT_EQ(physical,(std::set<std::uint32_t>(s.data().canonical_endpoints.begin(),s.data().canonical_endpoints.end())));
    EXPECT_EQ(physical.size()+1,s.data().nodes.size());
    const auto copy=[] {auto source=Source::Prepare(vehicle::test::Canonical(),MemberBytes(),Profile);return Source(source);}();
    EXPECT_EQ(copy.data().parts[0].material.curve.plastic_strain,copy.data().plastic_strain.data());
    EXPECT_EQ(copy.data().yield_stress_pa,s.data().yield_stress_pa);
}
TEST(Beam18SourceOriginal, RawPartScopeAndLateCurveChangesRejectWithoutChangingPublishedSource) {
    const auto& s=Original();
    for(unsigned fault=0;fault<4;++fault) {
        auto data=s.data();auto& p=data.parts[0];
        auto& section=data.sources[p.sources[1]].cards[0].second;
        auto& material=data.sources[p.sources[2]].cards[1].second;
        if(fault==0){section.resize(80,' ');section.replace(10,10,"         2");}
        if(fault==1){section.resize(80,' ');section.replace(20,10,"         0");}
        if(fault==2){material.resize(80,' ');material.replace(40,10,"         1");}
        if(fault==3){auto& c=data.sources[p.curve_source].cards.back().second;c.resize(40,' ');c.replace(20,20,"                   1");}
        EXPECT_THROW(detail::ReadPart(p,data),std::runtime_error);
    }
    EXPECT_EQ(s.data().parts[0].material.curve.yield_stress_pa,s.data().yield_stress_pa.data());
}
} // namespace crash::modelio::beam18::test
