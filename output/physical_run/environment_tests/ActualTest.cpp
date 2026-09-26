#include "../RunArchive.h"
#include "../Replay.h"
#include "../ViewerInput.h"
#include "case/vehicle_runtime/source/tests/ActualFixture.h"
#include "case/vehicle_dynamics/output/VehicleAcceptedFrames.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
namespace crash::output::physical_run::environment_test {
namespace {
namespace runtime=cases::vehicle_runtime;
namespace fixture=runtime::source_test;
namespace dynamics=cases::vehicle_dynamics;
constexpr std::size_t TestWorkspace=64u<<20;
constexpr std::size_t MappingCap=512u<<20;
constexpr std::size_t ReplayCap=512u<<20;
const Profile Observation{true,true,true,false,false};
runtime::Source Source() {return runtime::Source::WithEnvironment(fixture::OwnerSource());}
const physical_frames::Mapping& Mapping() {
    static const auto value=physical_frames::Mapping::Prepare(Source(),MappingCap);return value;
}
records::Identity Identity() {
    records::Identity id;id.run=UINT64_C(0x454e564f555450);id.topology=UINT64_C(0x454e5653454c46);return id;
}
records::Context ForecastContext() {
    auto id=Identity();id.owner=1;id.source_instance=Source().physical().domain()->source_instance_id();
    id.configuration=fixture::RuntimeConfig().configuration_id;id.qualification=fixture::RuntimeConfig().qualification_id;
    return Mapping().source_mapping().MakeFrameContext(id,fixture::RuntimeConfig().reserved_step_s);
}
struct Plan {
    dynamics::Forecast runtime;
    physical_frames::Forecast capture;
    Forecast archive;
    std::size_t mapping_phase=0,owner_phase=0,live_output_phase=0,prior_peak=0,qualification_peak=0;
};
Plan ForecastAll() {
    Plan p;p.runtime=dynamics::VehiclePhysicalDynamics::Preflight(Source(),fixture::DynamicsConfig());
    const auto context=ForecastContext();p.capture=physical_frames::PhysicalAcceptedFrames::Preflight(Mapping(),context);
    p.archive=RunArchive::PreflightWithEnvironment(Mapping(),context,
        MakeEnvironmentRequest(context,2,2*context.fixed_dt(),3),Observation);
    // Source constructor scratch is retired. Mapping preparation precedes owner
    // creation; capture/writer/reader coexist only with the admitted live graph.
    p.mapping_phase=fixture::Add(p.runtime.startup.retained_source_upper_bound,MappingCap);
    p.owner_phase=fixture::Add(p.runtime.peak_host_upper_bound,Mapping().payload_bytes());
    p.live_output_phase=fixture::Add(p.runtime.peak_host_upper_bound,
        fixture::Add(p.capture.peak_bytes,fixture::Add(p.archive.peak_host_bytes,fixture::Add(ReplayCap,TestWorkspace))));
    for(auto old:p.runtime.startup.prior_construction_bytes)p.prior_peak=std::max(p.prior_peak,old);
    p.qualification_peak=std::max({p.prior_peak,p.mapping_phase,p.owner_phase,p.live_output_phase});
    Require(p.qualification_peak<=fixture::GuardBytes,"Combined accepted-output forecast exceeds unchanged10GiB guard");
    return p;
}
std::filesystem::path Destination() {
    const auto* raw=std::getenv("ROBO_ENVELOPE_OUTPUT_DESTINATION");
    Require(raw&&*raw,"Missing create-only accepted-output destination");const std::filesystem::path path(raw);
    Require(std::filesystem::create_directory(path),"Accepted-output destination already exists");return path;
}
void ForecastFields(Document& doc,const Plan& p) {
    Integer(doc,"prior_source_peak_bytes",p.prior_peak);Integer(doc,"mapping_phase_bytes",p.mapping_phase);
    Integer(doc,"owner_phase_bytes",p.owner_phase);Integer(doc,"live_output_phase_bytes",p.live_output_phase);
    Integer(doc,"qualification_peak_bytes",p.qualification_peak);Integer(doc,"device_bytes",p.runtime.startup.device_bytes);
    Integer(doc,"capture_peak_bytes",p.capture.peak_bytes);Integer(doc,"archive_peak_bytes",p.archive.peak_host_bytes);
    Integer(doc,"archive_forecast_bytes",p.archive.archive.archive.forecast_bytes);
}
void SameSample(const Sample& saved,const physical_frames::PhysicalAcceptedFrames& capture) {
    ASSERT_NE(capture.frame(),nullptr);ASSERT_NE(capture.activity(),nullptr);
    const auto& actual=*capture.frame();ASSERT_TRUE(records::SameStamp(saved.frame.stamp,actual.stamp));
    ASSERT_EQ(saved.frame.position_xyz.size(),actual.position_xyz.size());
    for(std::size_t i=0;i<actual.position_xyz.size();++i)
        ASSERT_EQ(Bits(saved.frame.position_xyz[i]),Bits(actual.position_xyz[i]));
    ASSERT_EQ(saved.frame.plastic_points,actual.plastic_points);
    ASSERT_EQ(saved.activity.words(),capture.activity()->words());
}
}
TEST(EnvelopeAcceptedOutputActual, CompleteSourceForecastAndExactVehiclePrefix) {
    const auto p=ForecastAll();const auto context=ForecastContext();const auto& map=Mapping();
    ASSERT_NE(map.environment(),nullptr);EXPECT_EQ(map.physical_node_count(),376934u);
    EXPECT_EQ(context.nodes(),359785u);EXPECT_EQ(context.parents().size(),349645u);EXPECT_EQ(context.points(),956346u);
    EXPECT_EQ(map.render_counts().qeph,324094u);EXPECT_EQ(map.physical_counts().qeph,324095u);
    EXPECT_EQ(map.physical_counts().qbat,4250u);EXPECT_EQ(map.physical_counts().t3,21301u);
    EXPECT_EQ(map.physical().execution()->parents().size(),349646u);
    std::size_t global=0;for(const auto& row:map.parents())global+=row.law==tl::fea::ShellSectionLaw::GlobalLaw1Npt0;
    EXPECT_EQ(global,27177u);ASSERT_NE(map.source_mapping().execution(),nullptr);
    for(auto index:map.physical_nodes())ASSERT_LT(index,376930u);
    EXPECT_THROW(map.execution(),std::exception);
    physical_frames::Limits exact;exact.host_bytes=p.capture.peak_bytes;
    EXPECT_EQ(physical_frames::PhysicalAcceptedFrames::Preflight(map,context,exact).peak_bytes,p.capture.peak_bytes);
    --exact.host_bytes;
    EXPECT_THROW(physical_frames::PhysicalAcceptedFrames::Preflight(map,context,exact),std::exception);
    Limits archive_exact{p.archive.peak_host_bytes};const auto request=MakeEnvironmentRequest(context,2,2*context.fixed_dt(),3);
    EXPECT_EQ(RunArchive::PreflightWithEnvironment(map,context,request,Observation,archive_exact).peak_host_bytes,p.archive.peak_host_bytes);
    --archive_exact.host_bytes;
    EXPECT_THROW(RunArchive::PreflightWithEnvironment(map,context,request,Observation,archive_exact),std::exception);
    Document doc;doc.SetObject();String(doc,"schema","robo_dyna.envelope_accepted_output_forecast.v1");
    ForecastFields(doc,p);WriteJson(Destination()/"forecast.json",doc);
}
TEST(EnvelopeAcceptedOutputActual, ActualCommonOwnerFramesRollbackAndClosedArchiveRoundTrip) {
    const auto p=ForecastAll();const auto source=Source();const auto& map=Mapping();
    auto run=dynamics::VehiclePhysicalDynamics::Prepare(source,fixture::DynamicsConfig());
    dynamics::capture::VehicleAcceptedFrames capture(map,run,Identity());capture.Capture(run);
    const auto& producer=capture.frames();ASSERT_NE(producer.frame(),nullptr);
    const auto initial_positions=producer.frame()->position_xyz;
    const auto initial_activity=producer.activity()->words();
    const auto root=Destination(),archive_path=root/"archive";ASSERT_TRUE(std::filesystem::create_directory(archive_path));
    auto archive=RunArchive::PrepareWithEnvironment(archive_path,map,producer.context(),
        MakeEnvironmentRequest(producer.context(),2,2*producer.context().fixed_dt(),3),Observation);
    archive.Sample(*producer.frame(),*producer.activity());
    run.PrepareStep();
    // Accepted readback stays on the published slab during a private attempt.
    capture.Capture(run);
    EXPECT_EQ(producer.frame()->position_xyz,initial_positions);EXPECT_EQ(producer.activity()->words(),initial_activity);
    EXPECT_THROW(CaptureAcceptedInterval(run,producer,Observation),std::exception);
    run.DiscardStep();capture.Capture(run);
    EXPECT_EQ(producer.frame()->position_xyz,initial_positions);EXPECT_EQ(producer.frame()->stamp.epoch,0u);
    for(unsigned step=0;step<2;++step) {
        run.PrepareStep();run.CommitStep();
        archive.Append(CaptureAcceptedInterval(run,producer,Observation));
        capture.Capture(run);archive.Sample(*producer.frame(),*producer.activity());
    }
    ASSERT_EQ(run.accepted().epoch,2u);EXPECT_NE(producer.frame()->position_xyz,initial_positions);
    const auto manifest=archive.Finish();
    const auto replay=Replay::Open(archive_path,manifest,map.source_mapping().source().data().inputs,map.source_mapping().digest());
    ASSERT_TRUE(replay.environment());EXPECT_FALSE(replay.wall());EXPECT_FALSE(replay.wall_composition());
    EXPECT_TRUE(replay.configuration().environment);EXPECT_FALSE(replay.configuration().profile.native_contact);
    ASSERT_TRUE(replay.index().horizon_complete);ASSERT_EQ(replay.index().frames.size(),3u);
    EXPECT_EQ(replay.ReadSample(0).frame.position_xyz,initial_positions);
    ASSERT_NO_FATAL_FAILURE(SameSample(replay.ReadSample(2),producer));
    const auto& wall=map.environment()->wall();ASSERT_TRUE(replay.wall_mesh());
    ASSERT_EQ(replay.environment()->files.size(),3u);EXPECT_EQ(replay.environment()->part_id,wall.ids().part);
    for(unsigned i=0;i<4;++i) {
        const auto& p=wall.geometry().reference_m[i];const double expected[]{p.x,p.y,p.z};
        for(unsigned k=0;k<3;++k)EXPECT_EQ(Bits(replay.wall_mesh()->GetCoordsVertices()[i][k]),Bits(expected[k]));
    }
    const auto viewer=WriteViewerInput(root,"viewer-input.json",{"archive",manifest,
        map.source_mapping().source().data().inputs,map.source_mapping().digest()});
    const auto loaded=ReadViewerInput(root,viewer);EXPECT_EQ(loaded.mapping_sha256,map.source_mapping().digest());
    Document doc;doc.SetObject();String(doc,"schema","robo_dyna.envelope_accepted_output_actual.v1");ForecastFields(doc,p);
    Integer(doc,"physical_nodes",map.physical_node_count());Integer(doc,"vehicle_render_nodes",producer.context().nodes());
    Integer(doc,"vehicle_parents",producer.context().parents().size());Integer(doc,"physical_parents",349646);
    Integer(doc,"native_plastic_values",producer.context().points());Integer(doc,"accepted_steps",run.accepted().epoch);
    Number(doc,"actual_time_s",run.accepted().time);Boolean(doc,"contact_enabled_in_this_coupon",false);
    Boolean(doc,"visualization_only_not_restart",true);String(doc,"mapping_sha256",map.source_mapping().digest());
    WriteJson(root/"qualification.json",doc);
}
}
