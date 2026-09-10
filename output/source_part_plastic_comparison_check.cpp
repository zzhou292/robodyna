#include "SourcePartPlasticComparisonTestSupport.h"
#include <gtest/gtest.h>

namespace crash::output::plastic_comparison {
namespace {
TEST(SourcePlasticSensitivity, MatchesPhysicalInputsWhileAllowingOnlyResolutionDerivedChanges) {
    const auto coarse=test::Configuration(),fine=test::Configuration(true);
    const auto before_coarse=wc::Encode(coarse),before_fine=wc::Encode(fine);
    const auto s=MatchConfiguration(coarse,fine);
    EXPECT_DOUBLE_EQ(s.coarse_step,2*s.fine_step);EXPECT_EQ(s.coarse_stride,256u);EXPECT_EQ(s.fine_stride,512u);
    for(unsigned variant=0;variant<8;++variant) {
        auto changed=test::Copy(fine);
        if(variant==0)changed["rate_cowper_symonds_c_per_s"].SetDouble(8001);
        if(variant==1)changed["rate_cowper_symonds_p"].SetDouble(7);
        if(variant==2)changed["rate_filter_cutoff_hz"].SetDouble(10001);
        if(variant==3)changed["reference_nodes"][0]["mass_kg"].SetDouble(.3);
        if(variant==4)changed["wall_setup"]["declared_leading_gap_m"].SetDouble(.01);
        if(variant==5)changed["requested_horizon_s"].SetDouble(.01);
        if(variant==6)changed["frame_every"].SetUint64(511);
        if(variant==7)changed["future_physical_extension"]["must_match"].SetBool(false);
        const auto held=wc::Encode(changed);
        EXPECT_THROW(MatchConfiguration(coarse,changed),std::runtime_error);
        EXPECT_EQ(wc::Encode(changed),held);
    }
    EXPECT_EQ(wc::Encode(coarse),before_coarse);EXPECT_EQ(wc::Encode(fine),before_fine);
}

TEST(SourcePlasticSensitivity, AlignsSharedPhysicalCadenceAndLeavesDifferentStepOneTimesUncompared) {
    const auto s=MatchConfiguration(test::Configuration(),test::Configuration(true));
    EXPECT_EQ(SharedStrideCount(s,131072,262144)+1,513u);
    EXPECT_EQ(SharedStrideCount(s,131071,262144)+1,512u);
    EXPECT_THROW(SharedStrideCount(s,255,512),std::runtime_error);
    auto coarse=test::Frame(),fine=test::Copy(coarse);
    coarse["accepted_time_s"].SetDouble(s.coarse_step);
    fine["accepted_time_s"].SetDouble(s.fine_step);
    EXPECT_THROW(Difference(coarse,fine),std::runtime_error);
    coarse["accepted_time_s"].SetDouble(s.cadence);fine["accepted_time_s"].SetDouble(s.cadence);
    for(double value:Difference(coarse,fine))EXPECT_DOUBLE_EQ(value,0);
}

TEST(SourcePlasticSensitivity, ComparesEndpointMotionAndEveryOriginalMaterialPointWithoutMidpointAliasing) {
    const auto coarse=test::Frame();auto fine=test::Copy(coarse);
    fine["velocity_xyz_m_per_s"][350].SetDouble(1000); // Different midpoint fields are deliberately irrelevant.
    for(unsigned n=0;n<117;++n)fine["orientation_wxyz"][4*n].SetDouble(-1);
    for(double value:Difference(coarse,fine))EXPECT_DOUBLE_EQ(value,0);
    fine["position_xyz_m"][350].SetDouble(.002);
    fine["synchronized_velocity_xyz_m_per_s"][350].SetDouble(.25);
    fine["synchronized_omega_world_xyz_rad_per_s"][350].SetDouble(2);
    fine["plastic_sections"][93][12][2][5].SetDouble(.001);
    fine["plastic_sections"][93][3].SetDouble(.04);
    fine["cumulative_plastic_work_J"].SetDouble(.04);
    fine["synchronized_kinetic_J"].SetDouble(.1);
    fine["total_internal_work_J"].SetDouble(.2);
    fine["cumulative_wall_kick_impulse_N_s"].SetDouble(.01);
    fine["energy_residual_J"].SetDouble(-.0001);
    auto& a=fine.GetAllocator();Value contact(rapidjson::kObjectType);
    const double potential[]{.03,.03,.03,0},reaction[]{5,0,0};
    contact.AddMember("potential_J",FiniteArray(fine,potential,4),a);
    contact.AddMember("resultant_N",FiniteArray(fine,reaction,3),a);fine["contact"].Swap(contact);
    const auto held=wc::Encode(fine);const auto values=Difference(coarse,fine);
    const Differences expected{.002,0,.25,2,.001,.04,.04,.1,.2,.03,.33,.01,5,.0001};
    for(unsigned i=0;i<Channels;++i)EXPECT_NEAR(values[i],expected[i],1e-16) << Names[i];
    EXPECT_EQ(wc::Encode(fine),held);
}

TEST(SourcePlasticSensitivity, RejectsLateParentPointAndIdentityFaultsWithoutChangingInputs) {
    const auto original=test::Frame();const auto before=wc::Encode(original);
    for(unsigned variant=0;variant<5;++variant) {
        auto changed=test::Copy(original);auto& last=changed["plastic_sections"][93];
        if(variant==0)last[1].SetUint64(9999);
        if(variant==1)last[12][2].PopBack();
        if(variant==2)last[12][2][5].SetDouble(-1e-5);
        if(variant==3)last[12][2][5].SetDouble(.3001);
        if(variant==4)last[3].SetDouble(-1e-8);
        const auto held=wc::Encode(changed);
        EXPECT_THROW(Difference(original,changed),std::runtime_error);
        EXPECT_EQ(wc::Encode(changed),held);
    }
    EXPECT_EQ(wc::Encode(original),before);
}
} // namespace
} // namespace crash::output::plastic_comparison
