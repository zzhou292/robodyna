#include "../Internal.h"
#include "../../mixed_main/tests/ActualFixture.h"
#include "../../nodal_seed/tests/ActualMembers.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/BoundedArrayJson.h"
#include <gtest/gtest.h>
#include <sstream>
#include <cstdlib>
namespace crash::cases::vehicle_self_contact::native::initial_controls::test {
namespace {
constexpr std::size_t ExportBytes=1u<<20,GuardBytes=std::size_t{18}<<30;
const MainSource& Main(){return post_gapm::test::ActualPostGapmSource();}
const nodal_seed::test::ActualMembers& Members(){static const nodal_seed::test::ActualMembers m(Main().mixed().initial().selection().canonical());return m;}
const Wall& DeclaredWall() {
    static const auto value=[] {
        const auto* name=std::getenv("ROBO_NATIVE_WALL_MANIFEST");output::Require(name&&*name,"Missing authenticated wall manifest");
        const auto bytes=case_data::ReadPinnedWallManifest(name);std::istringstream stream(bytes);case_data::CanonicalWall original;
        output::Require(original.Load(stream).status==case_data::WallStatus::Ok,"Original wall provenance rejected");
        vehicle_wall::native::Declaration declaration;declaration.profile=vehicle_wall::native::Profile::EnvelopeFixedElasticV1;
        const auto& domain=Main().gap_operands().corrected().pre_correction().physical().source_domain();
        const auto made=Wall::Prepare(domain,Members().Input(),original,bytes,declaration);
        output::Require(made.report.status==vehicle_wall::native::Status::Ready&&made.source,made.report.reason.c_str());return *made.source;
    }();return value;
}
std::filesystem::path Destination(){const auto* name=std::getenv("ROBO_INITIALIZER_CONTROLS_OUTPUT");
    output::Require(name&&*name,"Missing create-only initializer-controls output");const std::filesystem::path p(name);
    output::Require(std::filesystem::create_directory(p),"Initializer-controls output exists");return p;}
void ForecastFields(output::Document& d,const Forecast& f) {
    output::Integer(d,"upstream_retained_bytes",f.upstream_retained);output::Integer(d,"wall_retained_bound_bytes",f.wall_retained_bound);
    output::Integer(d,"prior_peak_bytes",f.prior_peak);output::Integer(d,"import_workspace_bytes",f.import_workspace);
    output::Integer(d,"namespace_workspace_bytes",f.namespace_workspace);output::Integer(d,"output_bytes",f.output_values);
    output::Integer(d,"retained_bytes",f.retained_bytes);output::Integer(d,"current_phase_bytes",f.current_phase);
    output::Integer(d,"product_peak_bytes",f.peak_bytes);output::Integer(d,"qualification_peak_bytes",f.peak_bytes+ExportBytes);
}
}
TEST(InitializerControlsActual, CompleteSharedSourceForecastAndOneByteShortAdmission) {
    ASSERT_EQ(Main().startup_input().node_count,376930u);ASSERT_EQ(DeclaredWall().domain().node_count(),376934u);
    const auto f=InitializerControlsSource::Preflight(Main(),Members().Input(),&DeclaredWall());
    ASSERT_LE(f.peak_bytes+ExportBytes,GuardBytes);
    Limits exact;exact.host_bytes=f.peak_bytes;
    EXPECT_EQ(InitializerControlsSource::Preflight(Main(),Members().Input(),&DeclaredWall(),exact).peak_bytes,f.peak_bytes);
    --exact.host_bytes;const auto rejected=InitializerControlsSource::Prepare(Main(),Members().Input(),&DeclaredWall(),exact);
    EXPECT_EQ(rejected.report.status,Status::ResourceLimit);EXPECT_FALSE(rejected.source);
    output::Document d;d.SetObject();output::String(d,"schema","robo_dyna.initializer_controls_forecast.v1");ForecastFields(d,f);
    output::WriteJson(Destination()/"forecast.json",d);
}
TEST(InitializerControlsActual, WholeOriginalAndAddedWallNamespaceAndExactGapPhaseAreSourceDerived) {
    const auto f=InitializerControlsSource::Preflight(Main(),Members().Input(),&DeclaredWall());ASSERT_LE(f.peak_bytes+ExportBytes,GuardBytes);
    const auto made=InitializerControlsSource::Prepare(Main(),Members().Input(),&DeclaredWall());
    ASSERT_EQ(made.report.status,Status::Ready)<<made.report.reason;ASSERT_TRUE(made.source);const auto& source=*made.source;
    ASSERT_EQ(source.interfaces().size(),3u);EXPECT_EQ(source.provenance().original_interfaces,2u);EXPECT_EQ(source.provenance().declared_interfaces,1u);
    EXPECT_EQ(source.wall_interface_id(),DeclaredWall().ids().interface);EXPECT_NE(source.self_interface_id(),source.wall_interface_id());
    std::uint64_t previous=0;unsigned type2=0,type25=0;
    for(std::size_t i=0;i<source.interfaces().size();++i){const auto& row=source.interfaces()[i];
        EXPECT_GT(row.native_id,previous);previous=row.native_id;EXPECT_EQ(row.native_storage_ordinal,i+1);
        type2+=row.kind==InterfaceKind::Type2;type25+=row.kind==InterfaceKind::Type25;}
    EXPECT_EQ(type2,1u);EXPECT_EQ(type25,2u);EXPECT_EQ(source.gaps().shell_supports,337092u);
    EXPECT_EQ(output::Bits(source.gaps().secondary_maximum),output::Bits(Main().gap_report().maximum_secondary));
    EXPECT_EQ(output::Bits(source.gaps().global_search_gap),output::Bits(source.gaps().secondary_maximum+source.gaps().pre_ini_main_maximum));
    const auto& pop=source.native_population();EXPECT_FALSE(pop.exact_count_available);EXPECT_EQ(pop.lower,376934u);
    EXPECT_EQ(pop.complete_original_nodes,393165u);EXPECT_LE(pop.lower,pop.upper);EXPECT_LE(pop.upper,1500000u);
    EXPECT_EQ(pop.discrete_bound,35u);EXPECT_EQ(pop.rigid_definition_bound,2u*(922u+770u));EXPECT_EQ(pop.rigid_wall_bound,18u);
    ASSERT_TRUE(source.wall_raw_controls());EXPECT_EQ(source.raw_controls().reader_idel,1);
    EXPECT_EQ(source.wall_raw_controls()->reader_idel,0);EXPECT_EQ(source.wall_raw_controls()->reader_gap_mode,2);
    EXPECT_EQ(Main().post_gapm().final_solid_erosion,n::startup::SolidErosion::Enabled);
    output::Document d;d.SetObject();output::String(d,"schema","robo_dyna.initializer_controls_source.v1");ForecastFields(d,f);
    output::String(d,"source_digest",source.provenance().source_digest);output::String(d,"output_digest",source.provenance().output_digest);
    output::Integer(d,"self_native_id",source.self_interface_id());output::Integer(d,"wall_native_id",source.wall_interface_id());
    output::Number(d,"pre_ini_main_gap_maximum_native",source.gaps().pre_ini_main_maximum);
    output::Number(d,"secondary_gap_maximum_native",source.gaps().secondary_maximum);output::Number(d,"global_search_gap_native",source.gaps().global_search_gap);
    output::Integer(d,"native_population_lower",pop.lower);output::Integer(d,"native_population_upper",pop.upper);
    output::Boolean(d,"native_exact_count_available",false);output::Boolean(d,"history_or_runtime_ready",false);
    output::Value rows(rapidjson::kArrayType);
    for(const auto& row:source.interfaces()){output::Document r;r.SetObject();output::Integer(r,"native_id",row.native_id);
        output::Integer(r,"native_storage_ordinal",row.native_storage_ordinal);output::Integer(r,"type",row.kind==InterfaceKind::Type2?2:25);
        output::String(r,"origin",row.origin==InterfaceOrigin::OriginalDefinition?"original_definition":"declared_addition");
        output::String(r,"file",row.source.filename);output::String(r,"keyword",row.source.keyword);output::String(r,"sha256",row.source.sha256);
        output::Value v;v.CopyFrom(r,d.GetAllocator());rows.PushBack(v,d.GetAllocator());}
    d.AddMember("interfaces",rows,d.GetAllocator());output::WriteJson(Destination()/"source.json",d);
}
}
