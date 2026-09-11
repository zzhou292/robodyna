#include "GlassSourceSupport.h"
#include <gtest/gtest.h>

namespace crash::modelio::vehicle::test {
TEST(VehicleMidlayerSource, CompleteOverlayRetainsSourceRowsAndIndependentNativeIndices) {
    const auto& base=GlassResolution();
    const auto current=VehicleSectionResolution::ResolveOriginalMidlayer(base,ResolutionProfile::OriginalMidlayerV1);
    EXPECT_EQ(current.parents().data(),base.parents().data());
    EXPECT_EQ(&current.source().canonical().data(),&base.source().canonical().data());
    EXPECT_EQ(current.identity().sha256,base.identity().sha256);
    EXPECT_FALSE(current.resolution_key()==base.resolution_key());
    ASSERT_EQ(current.parents().size(),349645);
    EXPECT_EQ(current.counts().midlayer_parts,1);
    EXPECT_EQ(current.counts().midlayer_shells,4251);
    EXPECT_EQ(current.counts().unresolved_shells,5102);
    NativeFormulationCounts expected;
    std::size_t prior=0,midlayer=0,unresolved=0;
    for(std::size_t e=0;e<current.parents().size();++e) {
        const auto& row=current.parents()[e];
        const auto p=row.part_index;
        const auto* mapping=current.native_mapping(e);
        ASSERT_NE(mapping,nullptr);
        const auto* native=current.native_parent(e);
        if(current.parts()[p].status==SectionDisposition::Unresolved) {
            ++unresolved;
            EXPECT_EQ(mapping->family,tl::fea::ShellBindingFamily::None);
            EXPECT_EQ(mapping->family_index,SIZE_MAX);
            EXPECT_EQ(native,nullptr);
            continue;
        }
        ASSERT_NE(native,nullptr);
        const auto family=row.topology==SourceShellTopology::T3 ? tl::fea::ShellBindingFamily::T3 :
            (base.source().parts()[p].part_id==2000524 ? tl::fea::ShellBindingFamily::Qbat : tl::fea::ShellBindingFamily::Qeph);
        auto& count=family==tl::fea::ShellBindingFamily::T3 ? expected.t3 :
            (family==tl::fea::ShellBindingFamily::Qbat ? expected.qbat : expected.qeph);
        EXPECT_EQ(mapping->family,family);
        EXPECT_EQ(mapping->family_index,count++);
        EXPECT_EQ(native->source.family_index,mapping->family_index);
        EXPECT_EQ(native->source.source_parent_id,row.source_parent_id);
        if(current.parts()[p].status!=SectionDisposition::Midlayer) {
            ++prior;
            EXPECT_EQ(current.material(p),base.material(p));
            EXPECT_EQ(current.section(p),base.section(p));
            EXPECT_EQ(current.native_material(p),base.native_material(p));
            EXPECT_EQ(native->policy,base.native_parent(e)->policy);
        } else {
            ++midlayer;
            EXPECT_EQ(native->source.source_part_id,2000524);
            EXPECT_EQ(native->source.material_id,2000524);
            EXPECT_EQ(native->source.section_id,2000524);
            EXPECT_EQ(native->policy,tl::fea::ShellFailurePolicy::ConstantAllPoints);
            EXPECT_EQ(native->constant.failure_strain,2.5);
            EXPECT_EQ(current.section(p)->source_elform,9);
            EXPECT_EQ(current.section(p)->through_thickness_points,1);
            EXPECT_EQ(current.section_formulation(p),tl::fea::ShellSectionFormulation::OneThicknessPoint);
            EXPECT_EQ(base.material(p),nullptr);
            if(family==tl::fea::ShellBindingFamily::T3) EXPECT_EQ(row.source_parent_id,2357656);
        }
    }
    EXPECT_EQ(prior,340292);
    EXPECT_EQ(midlayer,4251);
    EXPECT_EQ(unresolved,5102);
    EXPECT_EQ(expected.qbat,4250);
    EXPECT_EQ(current.native_counts().qeph,expected.qeph);
    EXPECT_EQ(current.native_counts().t3,expected.t3);
    EXPECT_EQ(current.native_counts().qbat,expected.qbat);
    EXPECT_EQ(current.native_mapping(current.parents().size()),nullptr);
    RecordProperty("resolution_profile",OriginalMidlayerProfileName);
    RecordProperty("overlay_forecast_bytes",std::to_string(current.startup_budget_bytes()));
}
TEST(VehicleMidlayerSource, InvalidProfilesAndExactCapRejectWithoutChangingBase) {
    const auto& base=GlassResolution();
    constexpr auto profile=ResolutionProfile::OriginalMidlayerV1;
    const auto forecast=VehicleSectionResolution::ForecastOriginalMidlayer(base,profile);
    auto limits=ResolutionLimits{};
    limits.host_bytes=forecast-1;
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalMidlayer(base,profile,limits),std::runtime_error);
    limits=ResolutionLimits{};
    limits.parents=349644;
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalMidlayer(base,profile,limits),std::runtime_error);
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalMidlayer(base,ResolutionProfile::Artifact),std::runtime_error);
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalMidlayer(base,static_cast<ResolutionProfile>(99)),std::runtime_error);
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalMidlayer(Resolution(),profile),std::runtime_error);
    limits=ResolutionLimits{};
    limits.host_bytes=forecast;
    const auto current=VehicleSectionResolution::ResolveOriginalMidlayer(base,profile,limits);
    EXPECT_THROW(VehicleSectionResolution::ResolveOriginalMidlayer(current,profile),std::runtime_error);
    EXPECT_EQ(base.counts().unresolved_shells,9353);
    EXPECT_EQ(base.material(PartIndex(2000524)),nullptr);
    EXPECT_EQ(current.counts().unresolved_shells,5102);
    EXPECT_EQ(current.startup_budget_bytes(),forecast);
}
} // namespace crash::modelio::vehicle::test
