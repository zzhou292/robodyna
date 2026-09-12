#include "../WallComposition.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
namespace crash::output::physical_run::test {
namespace {
WallComposition Fixture(bool extended) {
    WallComposition c;
    c.profile=extended?CompositionProfile::ExtendedSolidsV4:CompositionProfile::RetainedV1;
    // Synthetic metadata controls; the extended domain value is not a source census.
    c.physical_nodes=extended?375000:372435;c.solid_parts=extended?16:9;
    c.point_mass_records=extended?150:148;
    c.solid_parents=extended?std::array<std::uint64_t,5>{908,1991,350,306,1345}:
        std::array<std::uint64_t,5>{908,1309,195,0,0};
    c.part_roots=20;c.plain_complete=extended?733:727;
    c.plain_restricted=extended?22:26;c.plain_omitted=extended?4:6;
    c.rigid_groups=c.part_roots+c.plain_complete+c.plain_restricted;
    c.rigid_members=extended?12925:12824;
    c.initial_mass_kg=std::nextafter(700.,701.);c.point_mass_kg=std::nextafter(16.,17.);
    return c;
}
WallComposition Supports() {
    auto c=Fixture(true);
    c.profile=CompositionProfile::VehicleSupportsV5;
    c.physical_nodes=376930;c.solid_parts=17;c.point_mass_records=154;
    c.solid_parents={908,1991,350,386,1345};
    c.structural_beam_parents=142;c.structural_beam_parts=4;
    c.plain_complete=755;c.plain_restricted=4;c.plain_omitted=0;
    c.rigid_groups=779;c.rigid_members=12961;
    return c; // Synthetic mass observations; source counts have a separate root gate.
}
Document Serialized(const Document& document) {
    rapidjson::StringBuffer bytes;rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(bytes);
    EXPECT_TRUE(document.Accept(writer));
    return array_json::Parse(std::string(bytes.GetString(),bytes.GetSize()),4096);
}
Document SetupDocument(const WallComposition& c,bool legacy=false) {
    Document d;d.SetObject();
    String(d,"schema",legacy?"robo_dyna.vehicle_wall_setup.v1":"robo_dyna.vehicle_wall_setup.v2");
    Integer(d,"physical_nodes",c.physical_nodes);
    if(!legacy)array_json::Child(d,"physical_composition",WallCompositionDocument(c));
    return d;
}
}
TEST(WallComposition, BothProfilesRoundTripExactMassBitsAndFiveFamilyCounts) {
    for(bool extended:{false,true}) {
        const auto expected=Fixture(extended);
        const auto document=Serialized(WallCompositionDocument(expected));
        const auto actual=ReadWallComposition(document);
        EXPECT_EQ(actual.profile,expected.profile);EXPECT_EQ(actual.solid_parents,expected.solid_parents);
        EXPECT_EQ(Bits(actual.initial_mass_kg),Bits(expected.initial_mass_kg));
        EXPECT_EQ(Bits(actual.point_mass_kg),Bits(expected.point_mass_kg));
        EXPECT_EQ(actual.physical_nodes,expected.physical_nodes);
        EXPECT_EQ(actual.rigid_members,expected.rigid_members);
        EXPECT_EQ(array_json::Text(document["joint_census"]),"unavailable_from_wall_setup");
    }
}
TEST(WallComposition, RejectsWrongProfileLateFamilyCountMassBitsAndRigidCensus) {
    const auto c=Fixture(true);
    auto bad=WallCompositionDocument(c);bad["solid_parents"]["law90_solid18"].SetUint64(1344);
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    bad=WallCompositionDocument(c);bad["solid_source_profile"].SetString("OriginalAdhesive18RubberHephS6zV1",bad.GetAllocator());
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    bad=WallCompositionDocument(c);bad["point_mass_binary64"].SetUint64(Bits(c.point_mass_kg)^1);
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    bad=WallCompositionDocument(c);bad["plain_restricted"].SetUint64(UINT64_MAX);
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    bad=WallCompositionDocument(c);bad["joint_census"].SetUint64(40);
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    bad=WallCompositionDocument(c);Integer(bad,"point_mass_records",150);
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    auto invalid=c;invalid.profile=static_cast<CompositionProfile>(19);
    EXPECT_THROW(WallCompositionDocument(invalid),std::exception);
    invalid=c;invalid.initial_mass_kg=std::numeric_limits<double>::infinity();
    EXPECT_THROW(WallCompositionDocument(invalid),std::exception);
    EXPECT_NO_THROW(ReadWallComposition(WallCompositionDocument(c)));
}
TEST(WallComposition, LegacyAbsenceAndNewRequiredReceiptPreserveSourceDomainBounds) {
    const auto c=Fixture(true);
    auto legacy=SetupDocument(Fixture(false),true);
    EXPECT_FALSE(ReadSetupComposition(legacy,359785,393165));
    array_json::Child(legacy,"physical_composition",WallCompositionDocument(c));
    EXPECT_THROW(ReadSetupComposition(legacy,359785,393165),std::exception);
    auto document=SetupDocument(c);
    ASSERT_TRUE(ReadSetupComposition(document,359785,393165));
    document["physical_nodes"].SetUint64(c.physical_nodes+1);
    EXPECT_THROW(ReadSetupComposition(document,359785,393165),std::exception);
    document=SetupDocument(c);document.RemoveMember("physical_composition");
    EXPECT_THROW(ReadSetupComposition(document,359785,393165),std::exception);
    document=SetupDocument(c);
    EXPECT_THROW(ReadSetupComposition(document,c.physical_nodes+1,393165),std::exception);
    EXPECT_THROW(ReadSetupComposition(document,359785,c.physical_nodes-1),std::exception);
    document["schema"].SetString("robo_dyna.vehicle_wall_setup.v99",document.GetAllocator());
    EXPECT_THROW(ReadSetupComposition(document,359785,393165),std::exception);
}
TEST(WallComposition, SupportsVersionKeepsExactTotalsAndRequiresQualifiedBeamCounts) {
    const auto expected=Supports();
    const auto document=Serialized(WallCompositionDocument(expected));
    EXPECT_EQ(array_json::Text(document["schema"]),"robo_dyna.wall_physical_composition.v2");
    EXPECT_EQ(array_json::Text(document["structural_beam_model_profile"]),"CircularFourPointLaw44V1");
    const auto actual=ReadWallComposition(document);
    EXPECT_EQ(actual.profile,CompositionProfile::VehicleSupportsV5);
    EXPECT_EQ(actual.structural_beam_parents,142u);EXPECT_EQ(actual.structural_beam_parts,4u);
    EXPECT_EQ(actual.solid_parents,expected.solid_parents);
    EXPECT_EQ(actual.point_mass_records,154u);
    EXPECT_EQ(Bits(actual.initial_mass_kg),Bits(expected.initial_mass_kg));
    EXPECT_EQ(Bits(actual.point_mass_kg),Bits(expected.point_mass_kg));
    const auto setup=SetupDocument(expected);
    EXPECT_EQ(array_json::Text(setup["schema"]),"robo_dyna.vehicle_wall_setup.v2");
    EXPECT_EQ(ReadSetupComposition(setup,359785,393165)->structural_beam_parents,142u);
    EXPECT_NO_THROW(CheckWallBeamObservation(true,actual));
    EXPECT_THROW(CheckWallBeamObservation(false,actual),std::exception);
    for(const auto* field:{"structural_beam_parents","structural_beam_parts","solid_parts","point_mass_records"}) {
        auto bad=WallCompositionDocument(expected);
        bad[field].SetUint64(bad[field].GetUint64()-1);
        EXPECT_THROW(ReadWallComposition(bad),std::exception);
    }
}
TEST(WallComposition, ForgedBeamVersionsAndLegacyClaimsRejectWithoutChangingAbsence) {
    const auto supports=Supports();
    auto bad=WallCompositionDocument(supports);
    bad["structural_beam_model_profile"].SetString("AnalyticLaw44",bad.GetAllocator());
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    bad=WallCompositionDocument(supports);
    bad["structural_beam_source_profile"].SetString("OriginalExtendedSolidsV4",bad.GetAllocator());
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    bad=WallCompositionDocument(supports);
    bad["schema"].SetString("robo_dyna.wall_physical_composition.v1",bad.GetAllocator());
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    for(const auto* field:{"structural_beam_source_profile","structural_beam_model_profile",
                           "structural_beam_parents","structural_beam_parts"})bad.RemoveMember(field);
    EXPECT_THROW(ReadWallComposition(bad),std::exception);
    for(bool extended:{false,true}) {
        const auto legacy=Fixture(extended);
        const auto document=WallCompositionDocument(legacy);
        EXPECT_EQ(array_json::Text(document["schema"]),"robo_dyna.wall_physical_composition.v1");
        EXPECT_FALSE(document.HasMember("structural_beam_parents"));
        EXPECT_NO_THROW(CheckWallBeamObservation(false,ReadWallComposition(document)));
        EXPECT_THROW(CheckWallBeamObservation(true,ReadWallComposition(document)),std::exception);
        auto invalid=legacy;invalid.structural_beam_parents=142;invalid.structural_beam_parts=4;
        EXPECT_THROW(WallCompositionDocument(invalid),std::exception);
    }
    EXPECT_NO_THROW(CheckWallBeamObservation(false,std::nullopt));
    EXPECT_THROW(CheckWallBeamObservation(true,std::nullopt),std::exception);
    EXPECT_NO_THROW(ReadWallComposition(WallCompositionDocument(supports)));
}
} // namespace crash::output::physical_run::test
