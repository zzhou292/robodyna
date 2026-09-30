#include "SourceAssemblyWallArtifactTestSupport.h"
#include "output/source_assembly/SourceAssemblyWallArtifacts.h"
#include "output/source_assembly/SourceAssemblyConnectorFields.h"
#include "case/source_assembly/tests/SourceBracketTestSupport.h"
#include "case/source_assembly_dynamics/tests/Fixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cuda_runtime_api.h>

namespace crash::output::assembly::test {
TEST(SourceBracketArtifactLive, ActualWeldFieldsRemainWithAcceptedOwnerAfterLateRejectedAttemptAndRetry) {
    int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
    const auto bindings=prepared::BracketBindings();prepared::WallInput wall;
    cases::source_assembly::SourceAssemblyWallSetup setup;
    auto r=setup.Initialize(bindings,wall.canonical,wall.bytes,prepared::WallSettings());ASSERT_TRUE(r)<<r.message;
    dynamics::SourceAssemblyWallCase run;auto config=Configuration();config.fixed_dt*=4;config.observe_force_stage=true;
    const auto startup=run.Initialize(bindings,setup,config);ASSERT_TRUE(startup)<<startup.message;
    Directory dir;auto request=Request();request.steps=64;request.frame_every=32;
    SourceAssemblyWallArtifacts writer(dir.path.string(),run,request);writer.WriteFrame(run);
    const auto initial=ReadJson(dir.path/"accepted-000000.fields.json");
    ASSERT_TRUE(initial.HasMember("connectors"));EXPECT_TRUE(initial["force_stage_kinetic"].IsNull());
    EXPECT_FALSE(initial["connectors"]["diagnostics"]["has_completed_interval"].GetBool());
    EXPECT_EQ(initial["diagnostics"]["shells"]["base_connector_kinetic_J"][0u].GetDouble(),0);
    std::uint64_t retained_attempt=0;
    for(unsigned step=0;step<64;++step) {
        const auto base=run.owner()->accepted();const auto result=run.Step();ASSERT_TRUE(result)<<result.message;
        writer.RecordInterval(base,run);
        if(step==31) {
            const auto accepted=run.owner()->accepted();const auto view=run.accepted_connectors();
            retained_attempt=view.diagnostics->attempt;const auto work=view.elements[0].history.internal_work_J[0];
            using Access=cases::source_assembly_dynamics::SourceAssemblyDynamicsTestAccess;
            const auto rejected=Access::RejectLate(run,Access::Fault::LastWallFace);ASSERT_FALSE(rejected);
            EXPECT_TRUE(fe::trial_identity::SameStamp(accepted,run.owner()->accepted()));
            EXPECT_EQ(run.accepted_connectors().diagnostics->attempt,retained_attempt);
            EXPECT_EQ(Bits(run.accepted_connectors().elements[0].history.internal_work_J[0]),Bits(work));
        }
        if((step+1)%32==0)writer.WriteFrame(run);
    }
    writer.Finish(run,0);
    const auto manifest=ReadJson(dir.path/"manifest.json");EXPECT_TRUE(manifest["horizon_complete"].GetBool());
    EXPECT_EQ(manifest["saved_frames"].GetUint64(),3u);
    EXPECT_STREQ(manifest["scope"].GetString(),wall_fields::ConnectorScope);
    EXPECT_STREQ(ReadJson(dir.path/"configuration.json")["scope"].GetString(),wall_fields::ConnectorScope);
    EXPECT_EQ(ReadBounded(dir.path/"source-assembly-inventory.json",4*1024*1024),bindings.source().data().authenticated_bytes);
    const auto middle=ReadJson(dir.path/"accepted-000032.fields.json");
    EXPECT_EQ(middle["connectors"]["diagnostics"]["attempt"].GetUint64(),retained_attempt);
    EXPECT_EQ(middle["force_stage_kinetic"]["attempt"].GetUint64(),retained_attempt);
    const auto final=ReadJson(dir.path/"accepted-000064.fields.json");
    EXPECT_EQ(final["nodal_fields"]["position_xyz_m"].Size(),3279u);
    EXPECT_EQ(final["sections"]["source_parents"].Size(),959u);EXPECT_EQ(final["contact"]["nodes"].Size(),1093u);
    const auto view=run.accepted_connectors();const auto& serialized=final["connectors"];
    EXPECT_EQ(serialized["diagnostics"]["epoch"].GetUint64(),run.owner()->accepted().epoch);
    EXPECT_EQ(serialized["diagnostics"]["attempt"].GetUint64(),view.diagnostics->attempt);
    EXPECT_EQ(serialized["elements"][0u]["source_element_id"].GetUint64(),2101297u);
    for(unsigned channel=0;channel<4;++channel)EXPECT_EQ(
        Bits(serialized["elements"][0u]["history"]["internal_work_J"][channel].GetDouble()),
        Bits(view.elements[0].history.internal_work_J[channel]));
    const auto& kinetic=final["diagnostics"]["shells"]["connector_kinetic_J"];
    EXPECT_EQ(Bits(kinetic[0u].GetDouble()),Bits(run.diagnostics()->shells.kinetic.connector_translation));
    EXPECT_EQ(Bits(kinetic[1u].GetDouble()),Bits(run.diagnostics()->shells.kinetic.connector_rotation));
    EXPECT_GT(run.diagnostics()->contact_intervals,0u);
    EXPECT_EQ(ParseCsvLedgerSegments(manifest[kCsvLedgerSegmentsField])[0].interval_count,64u);
}
} // namespace crash::output::assembly::test
