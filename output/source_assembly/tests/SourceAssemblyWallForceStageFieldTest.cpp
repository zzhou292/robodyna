#include "SourceAssemblyWallForceStageTestSupport.h"
#include "output/ReplayBundleTestSupport.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include <limits>

namespace crash::output::assembly::test {
namespace {
using cases::source_assembly_observation::ForceStageSummary;
std::string JsonText(const Document& d) {rapidjson::StringBuffer b;rapidjson::PrettyWriter<rapidjson::StringBuffer> w(b);d.Accept(w);return {b.GetString(),b.GetSize()};}

}
TEST(SourceAssemblyForceStageFields,OptionalDeclarationAndNullInitialLeaveDisabledDocumentBytesUnchanged) {
    WallFields f;const auto legacy=wall_fields::FrameDocument(f.View());
    auto view=f.View();view.observe_force_stage=true;auto enabled=wall_fields::FrameDocument(view);
    ASSERT_TRUE(enabled.HasMember("force_stage_kinetic"));EXPECT_TRUE(enabled["force_stage_kinetic"].IsNull());
    enabled.RemoveMember("force_stage_kinetic");EXPECT_EQ(JsonText(legacy),JsonText(enabled));
    auto config=Configuration();const auto off=wall_fields::ConfigurationDocument(f.bindings,f.setup,config,f.surface,Request());
    EXPECT_FALSE(off.HasMember("observe_force_stage"));config.observe_force_stage=true;
    auto on=wall_fields::ConfigurationDocument(f.bindings,f.setup,config,f.surface,Request());
    EXPECT_TRUE(on["observe_force_stage"].GetBool());on.EraseMember("observe_force_stage");EXPECT_EQ(JsonText(off),JsonText(on));
}
TEST(SourceAssemblyForceStageFields,CompleteSyntheticPhaseChannelsFitExistingWorstWidthFrameReservation) {
    WallFields f;f.Interval();f.Interval();auto stage=SyntheticStage(f);auto view=f.View();view.observe_force_stage=true;view.force_stage=&stage;
    const auto off=wall_fields::FrameDocument(f.View());auto on=wall_fields::FrameDocument(view);const auto& stored=on["force_stage_kinetic"];
    EXPECT_EQ(stored["base_epoch"].GetUint64(),1u);EXPECT_EQ(stored["enclosing_epoch"].GetUint64(),2u);
    EXPECT_EQ(stored["phase"]["force_time_s"].GetDouble(),f.stamp.reaction_time);
    EXPECT_EQ(stored["phase"]["previous_frame_time_s"].GetDouble(),0);
    EXPECT_EQ(stored["phase"]["previous_drift_dt_s"].GetDouble(),f.stamp.fixed_dt);
    EXPECT_EQ(stored["ordinary_native_nodes"][4u].GetDouble(),5);EXPECT_EQ(stored["grouped_native_members"][4u].GetDouble(),18);
    EXPECT_EQ(stored["aggregate_groups"][4u].GetDouble(),1);EXPECT_EQ(stored["aggregate_groups"][11u].GetDouble(),2);
    EXPECT_EQ(stored["native_total_J"].GetDouble(),23);EXPECT_EQ(stored["effective_total_J"].GetDouble(),35);
    EXPECT_EQ(stored["replacement_J"].GetDouble(),12);
    const auto worst=test_support::WorstScalarWidth(JsonText(on));EXPECT_LE(worst,WallFieldBytes);RecordProperty("enabled_worst_scalar_frame_bytes",worst);
    on.RemoveMember("force_stage_kinetic");EXPECT_EQ(JsonText(off),JsonText(on));
}
TEST(SourceAssemblyForceStageFields,MissingUndeclaredAndFalseAcceptedAssociationNeverSerialize) {
    WallFields f;auto stage=SyntheticStage(f);auto view=f.View();view.observe_force_stage=true;view.force_stage=&stage;
    EXPECT_THROW(wall_fields::FrameDocument(view),std::runtime_error);f.Interval();stage=SyntheticStage(f);
    for(unsigned kind=0;kind<12;++kind) {
        SCOPED_TRACE(kind);auto value=stage;view=f.View();view.observe_force_stage=true;view.force_stage=&value;
        if(kind==0)view.observe_force_stage=false;if(kind==1)view.force_stage=nullptr;
        if(kind==2)++value.owner_id;if(kind==3)++value.source.member_count;if(kind==4)++value.base_epoch;
        if(kind==5)++value.attempt;if(kind==6)++value.enclosing_epoch;if(kind==7)value.enclosing_time+=f.stamp.fixed_dt;
        if(kind==8)value.phase.force_time=f.stamp.time;if(kind==9)value.phase.durations.previous_drift_dt=f.stamp.fixed_dt;
        if(kind==10)value.phase.input_velocity_time=f.stamp.velocity_time;
        if(kind==11)value.native_total=std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(wall_fields::FrameDocument(view),std::runtime_error);
    }
}
} // namespace crash::output::assembly::test
