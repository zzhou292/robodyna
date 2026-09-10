#include "benchmarks/source_assembly_pilot/ComparisonInput.h"
#include "benchmarks/source_assembly_pilot/Metrics.h"
#include "output/ReplayBundleTestSupport.h"
#include <gtest/gtest.h>
#include <cstdlib>

namespace crash::benchmarks::assembly_pilot {
TEST(AssemblyPilot,ExactPhysicalTicksRejectRoundedRatiosOverflowAndDuplicateEpochs) {
    const double h=0x1p-26;EXPECT_EQ(StepMultiple(h,8*h),8u);EXPECT_EQ(StepMultiple(h,4*h),4u);
    EXPECT_EQ(StepMultiple(h,3*h),3u);
    EXPECT_THROW(StepMultiple(.1,.1*3),std::runtime_error);
    EXPECT_THROW(StepMultiple(h,std::nextafter(8*h,INFINITY)),std::runtime_error);
    EXPECT_THROW(StepMultiple(h,0),std::runtime_error);
    EXPECT_THROW(StepMultiple(h,h/2),std::runtime_error);
    EXPECT_THROW(Tick(UINT64_MAX,2),std::runtime_error);
    EXPECT_THROW(CommonFrames({0,1,1},1,{0,2},1),std::runtime_error);
    const auto common=CommonFrames({0,16,32,48,64},1,{0,2,4,5},8);
    ASSERT_EQ(common.size(),3u);EXPECT_EQ(common.back(),(std::pair<std::size_t,std::size_t>{2,2}));
    EXPECT_EQ(CommonFrames({0,16},1,{0,1},8).size(),1u); // Explicit initial-only prefix coverage.
}
TEST(AssemblyPilot,QuaternionSignAndSmallRotationAndScalarRmsRemainPhysical) {
    EXPECT_EQ(RotationDistance({1,0,0,0},{-1,0,0,0}),0);
    EXPECT_NEAR(RotationDistance({1,0,0,0},{std::cos(1e-8),0,0,std::sin(1e-8)}),2e-8,1e-22);
    EXPECT_THROW(RotationDistance({0,0,0,0},{1,0,0,0}),std::runtime_error);
    EXPECT_THROW(RotationDistance({NAN,0,0,0},{1,0,0,0}),std::runtime_error);
    Metric m;m.units="m";m.Add(1,4,7,123);m.Add(2,6,8,124);
    EXPECT_EQ(m.maximum,4);EXPECT_EQ(m.worst_source_id,124u);EXPECT_EQ(m.square,25);
    EXPECT_EQ(m.count,2u);EXPECT_EQ(m.nonzero,2u);
    EXPECT_THROW(m.Add(1,NAN),std::runtime_error);EXPECT_EQ(m.count,2u);
}
TEST(AssemblyPilot,PhysicalConfigurationKeepsUnknownMechanicsAndDiscardsOnlyOperationalDeclarations) {
    auto a=rd::Json(R"({"input":{"mass":1,"groups":[1]},"wall_setup":{"law":7},"deformation_limits":{"strain":0.2},"requested_horizon_s":1,"fixed_dt_s":1,"requested_steps":1,"owner_id":4,"future_physics":1})");
    Document b;b.CopyFrom(a,b.GetAllocator());b["fixed_dt_s"].SetDouble(.5);b["requested_steps"].SetUint64(2);b["owner_id"].SetUint64(9);
    b.AddMember("observe_force_stage",true,b.GetAllocator());EXPECT_TRUE(PhysicalConfiguration(a)==PhysicalConfiguration(b));
    b["input"]["mass"].SetDouble(2);EXPECT_FALSE(PhysicalConfiguration(a)==PhysicalConfiguration(b));
    b["input"]["mass"].SetDouble(1);b["future_physics"].SetInt(2);EXPECT_FALSE(PhysicalConfiguration(a)==PhysicalConfiguration(b));
}
const char* Fixture(){return std::getenv("ROBO_DYNA_PILOT_REFERENCE");}
TEST(AssemblyPilot,ActualPublicSelfComparisonJoinsAll65EndpointsWithoutAConvergenceVerdict) {
    if(!Fixture())GTEST_SKIP()<<"Set explicit actual archive fixture";
    const auto report=Compare({Fixture(),Fixture()});
    EXPECT_STREQ(report["status"].GetString(),"reported");EXPECT_FALSE(report.HasMember("converged"));
    const auto& pair=report["comparisons"][0];ASSERT_EQ(pair["common_saved_frames"].GetUint64(),65u);
    EXPECT_TRUE(pair["has_positive_common_time"].GetBool());EXPECT_TRUE(pair["shared_terminal_is_saved_in_both"].GetBool());
    EXPECT_TRUE(pair["reference_terminal_compared"].GetBool());EXPECT_TRUE(pair["candidate_terminal_compared"].GetBool());
    EXPECT_EQ(pair["last_compared_time_s"].GetDouble(),1024*0x1p-26);
    for(const auto& sample:pair["series"].GetArray())for(const auto& metric:sample["metrics"].GetObject())
        EXPECT_EQ(metric.value["max_abs_difference"].GetDouble(),0)<<metric.name.GetString();
    const auto& onset=report["runs"][0]["first_contact_record"];
    EXPECT_EQ(onset["epoch"].GetUint64(),42u);EXPECT_EQ(onset["previous_endpoint_s"].GetDouble(),41*0x1p-26);
    EXPECT_EQ(onset["endpoint_s"].GetDouble(),42*0x1p-26);
}
TEST(AssemblyPilot,ActualFrameMathReportsLastSourceLayerAndDoesNotCompareMidpointChannels) {
    if(!Fixture())GTEST_SKIP()<<"Set explicit actual archive fixture";
    const auto path=std::filesystem::path(Fixture())/"accepted-001024.fields.json";
    auto a=rd::Json(output::ReadBounded(path,32*1024*1024));Document b;b.CopyFrom(a,b.GetAllocator());
    auto same=Difference(a,b);for(const auto& [name,m]:same)EXPECT_EQ(m.maximum,0)<<name;
    auto& nodal=b["nodal_fields"];nodal["velocity_xyz_m_per_s"][0].SetDouble(1e10);
    b["diagnostics"]["motion"]["after"]["effective_total_J"].SetDouble(1e20);
    same=Difference(a,b);for(const auto& [name,m]:same)EXPECT_EQ(m.maximum,0)<<name;
    auto& point=b["sections"]["sections"][914][9][2];point[5].SetDouble(point[5].GetDouble()+.01);point[0].SetDouble(point[0].GetDouble()+123);
    const auto diff=Difference(a,b);EXPECT_NEAR(diff.at("layer_equivalent_plastic_strain").maximum,.01,1e-16);
    EXPECT_EQ(diff.at("layer_native_stress_XX").worst_index,2744u);EXPECT_EQ(diff.at("layer_native_stress_XX").maximum,123);
    EXPECT_EQ(diff.at("layer_native_stress_XX").worst_source_id,b["sections"]["source_parents"][914][1].GetUint64());
    point[0].SetDouble(NAN);
    EXPECT_THROW(Difference(a,b),std::runtime_error);
}
TEST(AssemblyPilot,ValidatedRunRejectsLateRehashedPhaseAndUnhashedTruncation) {
    if(!Fixture())GTEST_SKIP()<<"Set explicit actual archive fixture";
    output::test_support::ModifiedReplayBundle clone(Fixture());const std::string file="accepted-001024.fields.json";
    auto fields=clone.Read(file);fields["stamp"]["velocity_time"].SetDouble(0);clone.Replace(file,fields);clone.Rehash(file);
    assembly_pilot::Run invalid;
    EXPECT_THROW(invalid.Open(clone.directory),std::runtime_error);
    clone.ReplaceBytes(file,"{");
    EXPECT_THROW(invalid.Open(clone.directory),std::runtime_error);
}
} // namespace crash::benchmarks::assembly_pilot
