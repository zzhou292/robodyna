#include "SourceAssemblyWallForceStageTestSupport.h"
#include "case/source_assembly/tests/SourceBracketTestSupport.h"
#include "output/source_assembly/SourceAssemblyConnectorFields.h"
#include "output/ReplayBundleTestSupport.h"
#include "lib_src/elements/type25/Type25Math.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include <limits>

namespace crash::output::assembly::test {
namespace {
using prepared::BracketBindings;
std::string Json(const Document& d) {rapidjson::StringBuffer b;rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(b);Require(d.Accept(writer),"JSON test formatting failed");return {b.GetString(),b.GetSize()};}
cases::source_assembly::SourceAssemblyWallSetup Wall(const cases::source_assembly::SourceAssemblyBindings& b) {
    prepared::WallInput wall;cases::source_assembly::SourceAssemblyWallSetup value;
    const auto r=value.Initialize(b,wall.canonical,wall.bytes,prepared::WallSettings());Require(bool(r),r.message);return value;
}
struct BracketFields {
    cases::source_assembly::SourceAssemblyBindings bindings=BracketBindings();
    cases::source_assembly::SourceAssemblyWallSetup setup=Wall(bindings);
    WallFields fields{bindings,setup};tl::fea::type25::Evaluation element;
    BracketFields() {
        namespace spring=tl::fea::type25;const auto& model=*bindings.connectors();const auto& c=model.connections()[0];
        spring::EndpointKinematics nodes[2]{{c.position[0],{8,0,0},{}},{c.position[1],{8,0,0},{}}};
        Require(spring::Evaluate(model.source_units(),model.properties()[0].property,model.references()[0],
            model.initial_histories()[0],nodes,fields.stamp.fixed_dt,element)==spring::Status::Success,"Synthetic connector value fixture failed");
        auto& d=fields.diagnostics.shells.connector;d.source_instance_id=bindings.source_instance_id();d.owner_id=fields.stamp.owner_id;
        d.configuration_id=setup.settings()->configuration_id;d.qualification_id=setup.settings()->qualification_id;
        d.phase=spring::BatchPhase::Accepted;d.valid=true;d.element_count=d.active_count=1;d.minimum_native_dt=element.critical_dt_s;
        fields.diagnostics.shells.has_connector=true;fields.diagnostics.shells.kinetic.connector_translation=.032;
        fields.diagnostics.motion.after.connector.translation=.032;fields.captured=fields.diagnostics.shells;
    }
    void Interval() {
        const auto old=fields.stamp;fields.Interval();auto& d=fields.diagnostics.shells.connector;
        d.epoch=fields.stamp.epoch;d.base_epoch=old.epoch;d.attempt=fields.diagnostics.shells.qeph.attempt;
        d.time=fields.stamp.time;d.base_time=old.time;d.velocity_time=fields.stamp.velocity_time;d.base_velocity_time=old.velocity_time;
        d.kick_dt=fields.stamp.reaction_kick_dt;d.has_completed_interval=d.accepted_force_assembled=true;
        fields.captured=fields.diagnostics.shells;
    }
    wall_fields::FrameView View() const {auto v=fields.View();v.connectors={&fields.diagnostics.shells.connector,&element,1};return v;}
};
}
TEST(SourceBracketFields, SourcePropertyEndpointLedgerAndShellPartitionsRemainExplicit) {
    BracketFields f;const auto config=wall_fields::ConfigurationDocument(f.bindings,f.setup,Configuration(),f.fields.surface,Request());
    const auto& input=config["input"];ASSERT_TRUE(input.HasMember("connectors"));const auto& c=input["connectors"];
    EXPECT_STREQ(c["kind"].GetString(),wall_fields::ConnectorKind);EXPECT_STREQ(c["policy"].GetString(),wall_fields::ConnectorPolicy);
    EXPECT_EQ(c["connection_count"].GetUint64(),1u);EXPECT_EQ(c["source_units_to_SI"][0u].GetDouble(),1000);
    EXPECT_EQ(c["connections"][0u]["source_element_id"].GetUint64(),2101297u);
    EXPECT_EQ(c["connections"][0u]["source_card_lines"][1u].GetUint64(),26563u);
    const auto& p=c["properties"][0u];EXPECT_EQ(p["generated_property_id"].GetUint64(),383348001108u);
    EXPECT_EQ(p["mass_kg"].GetDouble(),.001);EXPECT_EQ(p["isotropic_inertia_kg_m2"].GetDouble(),1e-8);
    EXPECT_EQ(p["stiffness"][0u].GetDouble(),1e8);EXPECT_EQ(p["stiffness"][3u].GetDouble(),1000);
    for(unsigned e=0;e<2;++e) {
        const auto& row=c["endpoint_contributions"][e];ASSERT_EQ(row.Size(),9u);const auto n=row[3u].GetUint64();
        const auto& shell=input["native_nodes"][static_cast<unsigned>(n)];
        EXPECT_EQ(row[5u].GetDouble(),.0005);EXPECT_EQ(row[6u].GetDouble(),5e-9);
        EXPECT_EQ(Bits(row[7u].GetDouble()),Bits(shell[2u].GetDouble()+.0005));
        EXPECT_EQ(Bits(row[8u].GetDouble()),Bits(shell[3u].GetDouble()+5e-9));
    }
    EXPECT_TRUE(config.HasMember("connector_storage_limits"));EXPECT_TRUE(config.HasMember("connector_work_scope"));
    EXPECT_LT(wall_files::JsonBytes(config,WallConfigurationBytes),WallConfigurationBytes);
}
TEST(SourceBracketFields, AcceptedInitialAndSignedIntervalValuesRetainOnlyTheirDeclaredPhase) {
    BracketFields f;const auto initial=wall_fields::FrameDocument(f.View());const auto& c=initial["connectors"];
    EXPECT_EQ(c["diagnostics"]["epoch"].GetUint64(),0u);EXPECT_FALSE(c["diagnostics"]["has_completed_interval"].GetBool());
    EXPECT_EQ(initial["diagnostics"]["shells"]["base_connector_kinetic_J"][0u].GetDouble(),0);
    EXPECT_EQ(initial["diagnostics"]["motion"]["after"]["connector_kinetic_J"][0u].GetDouble(),.032);
    f.Interval();f.element.history.internal_work_J[0]=-.25;
    auto& d=f.fields.diagnostics.shells.connector;d.internal_work_J[0]=d.internal_work_increment_J[0]=-.25;
    f.fields.captured=f.fields.diagnostics.shells;const auto value=wall_fields::FrameDocument(f.View());
    EXPECT_EQ(value["connectors"]["elements"][0u]["history"]["internal_work_J"][0u].GetDouble(),-.25);
    EXPECT_EQ(value["connectors"]["diagnostics"]["base_time_s"].GetDouble(),0);
    EXPECT_EQ(value["connectors"]["diagnostics"]["time_s"].GetDouble(),f.fields.stamp.fixed_dt);
    EXPECT_EQ(value["diagnostics"]["native_internal_work_J"].GetDouble(),0); // Shell-only diagnostic remains separate.
}
TEST(SourceBracketFields, MissingStaleAndLateCorruptedConnectorCannotProduceAcceptedFields) {
    BracketFields f;f.Interval();const auto original=Json(wall_fields::FrameDocument(f.View()));
    auto view=f.View();view.connectors={};EXPECT_THROW(wall_fields::FrameDocument(view),std::runtime_error);
    auto diagnostic=*f.View().connectors.diagnostics;view=f.View();view.connectors.diagnostics=&diagnostic;
    ++diagnostic.attempt;EXPECT_THROW(wall_fields::FrameDocument(view),std::runtime_error);
    const auto saved=f.element;f.element.endpoints[1].couple_Nm.z=std::numeric_limits<double>::infinity();
    EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);f.element=saved;
    f.element.history.internal_work_J[3]=1;EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);f.element=saved;
    EXPECT_EQ(Json(wall_fields::FrameDocument(f.View())),original);
    WallFields six;EXPECT_THROW(wall_fields::ConnectorFrameDocument(six.View()),std::runtime_error);
    auto undeclared=six.View();undeclared.connectors=f.View().connectors;
    EXPECT_THROW(wall_fields::FrameDocument(undeclared),std::runtime_error);
}
TEST(SourceBracketFields, CompleteSevenPartAndMaximumConnectorEncodingFitExistingFrameReserve) {
    BracketFields f;f.Interval();auto stage=SyntheticStage(f.fields);stage.connector.translation=.032;
    auto view=f.View();view.observe_force_stage=true;view.force_stage=&stage;auto document=wall_fields::FrameDocument(view);
    // Formatting/byte-cap fixture only. Replication is never supplied to a
    // solver or accepted-frame validator. Every copied numeric token gets its
    // worst binary64/integer width through the existing shared estimator.
    auto& rows=document["connectors"]["elements"];Value first;first.CopyFrom(rows[0u],document.GetAllocator());
    while(rows.Size()<wall_fields::MaxArchivedConnectors){Value copy;copy.CopyFrom(first,document.GetAllocator());rows.PushBack(copy,document.GetAllocator());}
    const auto worst=test_support::WorstScalarWidth(Json(document));EXPECT_LE(worst,WallFieldBytes);
    RecordProperty("seven_part_128_connectors_worst_scalar_frame_bytes",worst);
    const auto plan=PlanWallArchive(Request(),source::PinnedYarisSevenPartInventory().bytes,f.setup.placed_wall()->source_manifest()->size());
    EXPECT_LE(plan.forecast_bytes,kArtifactExtendedTotalCap);
    WallFields six;const auto legacy=wall_fields::FrameDocument(six.View());EXPECT_FALSE(legacy.HasMember("connectors"));
    EXPECT_FALSE(legacy["diagnostics"]["shells"].HasMember("connector_kinetic_J"));
}
} // namespace crash::output::assembly::test
