#include "../RigidPartDeclarations.h"
#include "modelio/vehicle_sections/tests/MidlayerFixture.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include <gtest/gtest.h>
namespace crash::modelio::vehicle::test {
namespace {
PartDisposition RigidFields() {
    PartDisposition p;p.part_id=p.material_id=p.section_id=2000043;p.shell_count=621;
    p.unresolved_sources={
        Block("*PART\n$ title\n437_tirefontdisk\n$ ids\n   2000043   2000043   2000043\n",985),
        Block("*SECTION_SHELL\n$ fields\n   2000043         2                   3\n$ thickness\n"
              "  4.000000  4.000000  4.000000  4.000000\n",990),
        Block("*MAT_RIGID\n$ material\n   2000043 1.7770E-8 2.0000E+5  0.300000\n$ constraints\n\n$ axes\n\n",995)};
    return p;
}
auto Read(const PartDisposition& p) {return rigid_part::ReadDeclaration(p,{1000,.001,1});}
}
TEST(VehicleRigidFields, LiteralRigidCardsSupplyLaw1CoefficientsWithoutChangingSourceRole) {
    const auto p=RigidFields();const auto d=Read(p);
    EXPECT_EQ(d.material.source.keyword,"*MAT_RIGID");
    EXPECT_EQ(d.material.law,assembly::MaterialLaw::LayeredLaw1);
    EXPECT_EQ(output::Bits(d.material.density_kg_m3),output::Bits(1.7770e-8*(1000/(.001*.001*.001))));
    EXPECT_EQ(d.material.young_pa,2e11);EXPECT_EQ(d.material.poisson_ratio,.3);
    EXPECT_EQ(d.material.cards[0].blank_mask,240);EXPECT_EQ(d.material.cards[1].blank_mask,255);
    EXPECT_EQ(d.material.cards[2].blank_mask,255);EXPECT_EQ(d.section.cards[0].blank_mask,244);
    EXPECT_EQ(d.section.through_thickness_points,3);EXPECT_EQ(d.section.source_elform,2);
    for(auto t:d.section.thickness_m) EXPECT_EQ(t,.004);
    const auto native=assembly::detail::NativeMaterial(d.material);
    EXPECT_EQ(native.law,tl::fea::ShellSectionLaw::LayeredLaw1Nip3);
    EXPECT_EQ(native.curve_id,0);
}
TEST(VehicleRigidFields, OriginalBlankAndExplicitZeroRemainDistinct) {
    auto p=RigidFields();SetField(p.unresolved_sources[0],989,3,"0");
    const auto d=Read(p);
    EXPECT_TRUE(d.part_cards[0].values[3].has_value());EXPECT_EQ(*d.part_cards[0].values[3],0);
    EXPECT_FALSE(Read(RigidFields()).part_cards[0].values[3].has_value());
    SetField(p.unresolved_sources[2],999,0,"0");
    EXPECT_THROW(Read(p),std::runtime_error); // Original CMO is blank, no source option is invented.
}
TEST(VehicleRigidFields, WrongIdsNipOptionsAndLateThicknessRejectWithoutPriorMutation) {
    const auto p=RigidFields();const auto old=Read(p);
    for (const auto change:{std::array<unsigned,3>{0,989,2}, {1,992,3}, {1,994,3}, {2,997,3}, {2,1001,7}}) {
        auto bad=p;SetField(bad.unresolved_sources[change[0]],change[1],change[2],"7");
        EXPECT_THROW(Read(bad),std::runtime_error);
    }
    auto bad=p;bad.material_id++;EXPECT_THROW(Read(bad),std::runtime_error);
    bad=p;SetField(bad.unresolved_sources[1],994,4,"1");EXPECT_THROW(Read(bad),std::runtime_error);
    EXPECT_EQ(old.material.source.raw_text,p.unresolved_sources[2].raw_text);
    EXPECT_EQ(Read(p).section.thickness_m,old.section.thickness_m);
}
TEST(VehicleRigidFields, FiniteSourceOverflowBadShapeAndUnitsAreRejected) {
    auto p=RigidFields();SetField(p.unresolved_sources[2],997,2,"1e308");
    EXPECT_THROW(Read(p),std::runtime_error);
    p=RigidFields();p.unresolved_sources[2].raw_text+="\n";p.unresolved_sources[2].last_line++;
    p.unresolved_sources[2].sha256=output::Sha256(p.unresolved_sources[2].raw_text);
    EXPECT_THROW(Read(p),std::runtime_error);
    p=RigidFields();EXPECT_THROW(rigid_part::ReadDeclaration(p,{1,1,1}),std::runtime_error);
}
}
