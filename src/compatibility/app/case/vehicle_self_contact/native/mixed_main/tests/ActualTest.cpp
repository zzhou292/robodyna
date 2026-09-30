#include "ActualFixture.h"
#include "../Internal.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <algorithm>
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::post_gapm::test {
namespace {
constexpr std::size_t ExportBytes=2u<<20;
std::filesystem::path Destination() {
    const auto* value=std::getenv("ROBO_POST_GAPM_MAIN_OUTPUT");
    output::Require(value&&*value,"Missing explicit post-GAPM source output");
    const std::filesystem::path out(value);output::Require(std::filesystem::create_directory(out),"Post-GAPM output exists");return out;
}
void Counts() {
    ASSERT_EQ(ActualMixedSource().sides().primary_count,341504u);
    ASSERT_EQ(ActualMixedSource().sides().main_count,678596u);
    ASSERT_EQ(ActualMixedSource().initial().geometry().nodes.size(),376930u);
    ASSERT_EQ(ActualGapOperands().counts().shells,349645u);
}
void ForecastFields(output::Document& doc,const Forecast& f) {
    output::Integer(doc,"mixed_retained_bytes",f.mixed_retained);output::Integer(doc,"gap_additional_retained_bytes",f.gap_additional_retained);
    output::Integer(doc,"input_map_bytes",f.input_maps);output::Integer(doc,"query_workspace_bytes",f.query_workspace);
    output::Integer(doc,"output_value_bytes",f.output_values);output::Integer(doc,"temporary_main_bytes",f.temporary_mains);
    output::Integer(doc,"gap_scratch_bytes",f.gap_scratch);output::Integer(doc,"metadata_bytes",f.metadata);
    output::Integer(doc,"mixed_input_construction_peak_bytes",f.mixed_input_construction_peak);
    output::Integer(doc,"gap_input_construction_peak_bytes",f.gap_input_construction_peak);
    output::Integer(doc,"prior_construction_peak_bytes",f.prior_construction_peak);output::Integer(doc,"current_phase_bytes",f.current_phase);
    output::Integer(doc,"product_peak_bytes",f.peak_bytes);output::Integer(doc,"retained_bytes",f.retained_bytes);
    output::Integer(doc,"qualification_export_bytes",ExportBytes);output::Integer(doc,"qualification_peak_bytes",f.peak_bytes+ExportBytes);
}
}
TEST(PostGapmMainActual, CompleteForecastAndOneByteShortRejectBeforeSourcePublication) {
    ASSERT_NO_FATAL_FAILURE(Counts());
    const auto& mixed=ActualMixedSource();const auto& gaps=ActualGapOperands();
    const auto forecast=PostGapmMainSource::Preflight(mixed,gaps);
    ASSERT_LE(forecast.peak_bytes,(std::size_t{10}<<30)-ExportBytes);
    auto exact=Limits{};exact.host_bytes=forecast.peak_bytes;
    EXPECT_EQ(PostGapmMainSource::Preflight(mixed,gaps,exact).peak_bytes,forecast.peak_bytes);
    --exact.host_bytes;const auto rejected=PostGapmMainSource::Prepare(mixed,gaps,ActualCombine(),exact);
    EXPECT_EQ(rejected.report.status,Status::ResourceLimit);EXPECT_FALSE(rejected.source);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.post_gapm_main_forecast.v1");
    ForecastFields(doc,forecast);output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(PostGapmMainActual, EveryMixedPrimaryHasSignedSourceKPhysicalOwnerAndCompletedGapFields) {
    ASSERT_NO_FATAL_FAILURE(Counts());
    const auto forecast=PostGapmMainSource::Preflight(ActualMixedSource(),ActualGapOperands());
    ASSERT_LE(forecast.peak_bytes,(std::size_t{10}<<30)-ExportBytes);
    const auto& source=ActualPostGapmSource();const auto& c=source.counts();
    const auto post=source.post_gapm();const auto input=source.startup_input();
    EXPECT_EQ(c.primaries,341504u);EXPECT_EQ(c.expanded,678596u);
    EXPECT_EQ(c.shell_owners,337092u);EXPECT_EQ(c.solid_owners,4412u);
    EXPECT_EQ(c.pre_shell_internal,268u);EXPECT_EQ(c.final_internal,268u);EXPECT_EQ(c.primary_reversals,157u);
    EXPECT_EQ(c.negative_exterior_first_volumes,193u);
    EXPECT_EQ(c.solid_matches[0],336351u);EXPECT_EQ(c.solid_matches[1],4800u);EXPECT_EQ(c.solid_matches[2],353u);
    EXPECT_EQ(c.positive_coefficients+c.zero_coefficients+c.negative_coefficients,c.expanded);
    EXPECT_EQ(post.incoming_solid_erosion,s::SolidErosion::Enabled);EXPECT_EQ(post.final_solid_erosion,s::SolidErosion::Enabled);
    EXPECT_EQ(post.pre_shell_internal_count,268u);EXPECT_EQ(input.node_count,376930u);
    EXPECT_EQ(input.primary,source.mixed().primary());EXPECT_EQ(input.raw_origins,source.mixed().sides().raw_origins);
    ASSERT_EQ(source.coefficients().size(),c.expanded);ASSERT_EQ(source.main_gaps().size(),c.expanded);
    ASSERT_TRUE(source.gap_report().completed);EXPECT_EQ(source.gap_report().status,n::source_gaps::Status::Ok);
    ASSERT_EQ(source.primary_owners().size(),c.primaries);
    std::size_t negative=0;std::uint64_t previous=0;
    const auto& geometry=source.mixed().initial().geometry();
    for(auto node:source.secondary_nodes()) {
        ASSERT_LT(node,geometry.nodes.size());EXPECT_GT(geometry.nodes[node].source_id,previous);previous=geometry.nodes[node].source_id;
    }
    for(std::size_t i=0;i<c.expanded;++i) {
        ASSERT_TRUE(std::isfinite(source.coefficients()[i]));negative+=source.coefficients()[i]<0.;
        ASSERT_NE(post.final_support[i].first.kind,s::PhysicalSupportKind::Unspecified);
        ASSERT_NE(post.final_support[i].first.source_element_id,0u);
        const auto p=source.mixed().sides().expanded_to_primary[i];ASSERT_LT(p,c.primaries);
        const auto& owner=source.primary_owners()[p];EXPECT_EQ(owner.source_element,post.final_support[i].first.source_element_id);
        if(i>=c.primaries) {
            EXPECT_EQ(post.final_support[i].second_solid_source_id,0u);EXPECT_GT(source.coefficients()[i],0.);
            EXPECT_NE(owner.kind,s::PhysicalSupportKind::EightSlotSolid);
        }
    }
    EXPECT_EQ(negative,c.negative_coefficients);
    for(const auto value:source.secondary_gaps())EXPECT_GE(value,0.);
    EXPECT_LE(source.forecast().retained_bytes,forecast.retained_bytes);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.post_gapm_main_source.v1");
    output::String(doc,"source_digest",source.provenance().source_digest);output::String(doc,"output_digest",source.provenance().output_digest);
    output::String(doc,"mixed_digest",source.provenance().mixed_digest);output::String(doc,"gap_operand_digest",source.provenance().gap_operand_digest);
    output::Integer(doc,"primaries",c.primaries);output::Integer(doc,"expanded",c.expanded);
    output::Integer(doc,"shell_owners",c.shell_owners);output::Integer(doc,"solid_owners",c.solid_owners);
    output::Integer(doc,"pre_shell_internal",c.pre_shell_internal);output::Integer(doc,"final_internal",c.final_internal);
    output::Integer(doc,"primary_reversals",c.primary_reversals);output::Integer(doc,"negative_exterior_first_volumes",c.negative_exterior_first_volumes);
    output::Integer(doc,"negative_first_volumes",c.negative_first_volumes);output::Integer(doc,"negative_second_volumes",c.negative_second_volumes);
    output::Integer(doc,"positive_coefficients",c.positive_coefficients);output::Integer(doc,"zero_coefficients",c.zero_coefficients);
    output::Integer(doc,"negative_coefficients",c.negative_coefficients);output::Integer(doc,"secondary_nodes",source.secondary_nodes().size());
    output::Integer(doc,"main_nodes",source.main_nodes().size());output::Number(doc,"minimum_secondary_gap",source.gap_report().minimum_secondary);
    output::Number(doc,"maximum_secondary_gap",source.gap_report().maximum_secondary);
    output::String(doc,"main_gap_phase","I25INI_GAP_N_before_I25BUC_VOX1");
    output::Boolean(doc,"solid_erosion_enabled",true);output::Boolean(doc,"normals_ready",false);output::Boolean(doc,"contact_runtime_ready",false);
    ForecastFields(doc,source.forecast());output::WriteJson(Destination()/"source.json",doc);
}
}
