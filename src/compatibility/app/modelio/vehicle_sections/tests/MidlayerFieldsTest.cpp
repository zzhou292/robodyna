#include "MidlayerFixture.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include <gtest/gtest.h>

namespace crash::modelio::vehicle::test {
TEST(VehicleMidlayerFields, OriginalBlankFieldsAndNativeOnePointPolicyStaySeparate) {
    const auto declaration=resolution::ReadMidlayer(MidlayerFields(),{1000,.001,1});
    const auto& material=declaration.material;
    EXPECT_EQ(declaration.section.source_elform,9);
    EXPECT_EQ(declaration.section.through_thickness_points,1);
    EXPECT_EQ(declaration.section.cards[0].blank_mask,244);
    EXPECT_EQ(declaration.section.cards[1].blank_mask,240);
    EXPECT_EQ(material.cards[1].blank_mask,243);
    EXPECT_EQ(material.cards[1].names[4],"vp");
    EXPECT_EQ(material.cards[3].names[7],"es8");
    for(auto f:{0,1,4}) EXPECT_FALSE(material.cards[1].values[f]);
    EXPECT_EQ(output::Bits(*material.cards[1].values[2]),output::Bits(0.0));
    EXPECT_EQ(output::Bits(material.young_pa),output::Bits(250.0*(1000/(.001*1*1))));
    EXPECT_EQ(output::Bits(material.density_kg_m3),output::Bits(1e-9*(1000/(.001*.001*.001))));
    EXPECT_EQ(declaration.failure_strain,2.5);
    const auto native=assembly::detail::NativeMaterial(material,assembly::detail::NativeLaw44Rate::FilteredZeroC);
    EXPECT_EQ(native.material_id,2000524);
    EXPECT_EQ(native.curve_id,0);
    EXPECT_EQ(native.rate.policy,tl::material::ShellPlasticityRatePolicy::FilteredZeroC);
    EXPECT_EQ(native.rate.cowper_symonds_c_per_s,0);
    EXPECT_EQ(native.rate.cowper_symonds_p,1);
    EXPECT_EQ(native.rate.cutoff_hz,10000);
    EXPECT_EQ(*declaration.material.supplied_etan_pa,1e6);
}
TEST(VehicleMidlayerFields, LateUnsupportedAndNonfiniteSourceFieldsPreservePriorThenRetry) {
    const auto original=MidlayerFields();
    auto visible=resolution::ReadMidlayer(original,{1000,.001,1});
    for(unsigned fault=0;fault<12;++fault) {
        SCOPED_TRACE(fault);
        auto bad=original;
        switch(fault) {
        case 0:SetField(bad.unresolved_sources[1],10114,1,"16");break;
        case 1:SetField(bad.unresolved_sources[1],10114,3,"3");break;
        case 2:SetField(bad.unresolved_sources[1],10116,4,"0");break;
        case 3:SetField(bad.unresolved_sources[2],10121,0,"0");break;
        case 4:SetField(bad.unresolved_sources[2],10121,4,"0");break;
        case 5:SetField(bad.unresolved_sources[2],10121,3,"-0");break;
        case 6:SetField(bad.unresolved_sources[2],10119,6,"0");break;
        case 7:SetField(bad.unresolved_sources[2],10119,2,"nan");break;
        case 8:SetField(bad.unresolved_sources[2],10119,5,"251");break;
        case 9:SetField(bad.unresolved_sources[2],10125,7,"1");break;
        case 10:bad.material_id=7;break;
        case 11:SetField(bad.unresolved_sources[2],10119,2,"1e308");break;
        }
        EXPECT_THROW(visible=resolution::ReadMidlayer(bad,{1000,.001,1}),std::runtime_error);
        EXPECT_EQ(visible.material.source.raw_text,original.unresolved_sources[2].raw_text);
        EXPECT_EQ(visible.failure_strain,2.5);
    }
    EXPECT_THROW(resolution::ReadMidlayer(original,{1,.001,1}),std::runtime_error);
    visible=resolution::ReadMidlayer(original,{1000,.001,1});
    EXPECT_EQ(visible.section.thickness_m[3],.0005);
}
TEST(VehicleMidlayerFields, NumericalCardCapsAndCompleteLineCoverageAreChecked) {
    auto block=MidlayerFields().unresolved_sources[2];
    EXPECT_THROW(detail::ReadSourceCards(block,0,0),std::runtime_error);
    EXPECT_THROW(detail::ReadSourceCards(block,2,4),std::runtime_error);
    ++block.last_line;
    EXPECT_THROW(detail::ReadSourceCards(block),std::runtime_error);
    block=MidlayerFields().unresolved_sources[2];
    block.raw_text.resize(64*1024+1,' ');
    auto part=MidlayerFields();
    part.unresolved_sources[2]=block;
    EXPECT_THROW(resolution::ReadMidlayer(part,{1000,.001,1}),std::runtime_error);
}
TEST(VehicleMidlayerFields, ProfileIdentityIncludesInterpretationWithoutInventedArtifact) {
    const assembly::ArtifactIdentity artifact{9,std::string(64,'a')};
    const ResolutionKey old{artifact,ResolutionProfile::Artifact};
    const ResolutionKey current{artifact,ResolutionProfile::OriginalMidlayerV1};
    EXPECT_FALSE(old==current);
    EXPECT_TRUE(current==ResolutionKey({artifact,ResolutionProfile::OriginalMidlayerV1}));
}
} // namespace crash::modelio::vehicle::test
