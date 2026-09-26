#include "../Internal.h"
#include "../../mixed_main/tests/ActualFixture.h"
#include "case/vehicle_wall/native/physical/tests/ActualFixture.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::mixed_starter::test {
namespace main_test=post_gapm::test;
namespace wall_test=vehicle_wall::native::physical_test;
namespace {
constexpr std::size_t ExportBytes=2u<<20;
const DomainEmbedding& Embedding() {
    static const auto value=[] {
        const auto& main=main_test::ActualPostGapmSource();
        const auto& wall=wall_test::Wall();
        const auto& original=detail::OriginalDomain(main);
        output::Require(wall.vehicle_origin().domain().SharesStorage(original),
            "Actual wall and contact must reuse the same original domain");
        const auto count=wall.domain().node_count()-original.node_count();
        const auto* suffix=wall.domain().nodes().data()+original.node_count();
        return DomainEmbedding::Prepare(wall.vehicle_origin().source(),original,wall.domain(),{suffix,count});
    }();return value;
}
std::filesystem::path Destination() {
    const auto* text=std::getenv("ROBO_MIXED_STARTER_OUTPUT");
    output::Require(text&&*text,"Missing create-only mixed Starter source output");
    const std::filesystem::path path(text);
    output::Require(std::filesystem::create_directory(path),"Mixed Starter output exists");return path;
}
std::size_t QualificationBytes(const Forecast& f) {
    // The qualification's WallSource is an external live geometry handle.
    // Conservatively retain its full published peak beyond this stage's live
    // phase; no private payload accounting or second physical model is used.
    const auto wall=wall_test::Wall().forecast().peak_bytes;
    output::Require(f.current_phase<=SIZE_MAX-wall,"Mixed Starter qualification sum overflows");
    const auto peak=std::max(f.prior_construction_peak,f.current_phase+wall);
    output::Require(peak<=(std::size_t{10}<<30)-ExportBytes,
        "Complete mixed Starter qualification exceeds unchanged10GiB guard");
    return peak+ExportBytes;
}
void Counts() {
    const auto& main=main_test::ActualPostGapmSource();
    ASSERT_EQ(main.mixed().sides().node_count,376930u);
    ASSERT_EQ(main.mixed().sides().primary_count,341504u);
    ASSERT_EQ(main.mixed().sides().main_count,678596u);
    ASSERT_EQ(Embedding().domain().node_count(),376934u);
}
}
TEST(MixedStarterActual, CompleteCombinedForecastRejectsOneByteShortBeforeBuild) {
    ASSERT_NO_FATAL_FAILURE(Counts());
    const auto& main=main_test::ActualPostGapmSource();
    const auto forecast=MixedStarterSource::Preflight(main,Embedding());
    const auto qualified=QualificationBytes(forecast);
    auto exact=Limits{};exact.host_bytes=forecast.peak_bytes;
    EXPECT_EQ(MixedStarterSource::Preflight(main,Embedding(),exact).peak_bytes,forecast.peak_bytes);
    --exact.host_bytes;
    const auto rejected=MixedStarterSource::Prepare(main,Embedding(),exact);
    EXPECT_EQ(rejected.report.status,Status::ResourceLimit);EXPECT_FALSE(rejected.source);
    auto doc=ForecastDocument(forecast);
    output::Integer(doc,"qualification_peak_bytes",qualified);
    output::Integer(doc,"external_wall_reservation",wall_test::Wall().forecast().peak_bytes);
    output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(MixedStarterActual, OneBuildRetainsEveryVehicleOriginAndGenuineStarterCacheOnCombinedDomain) {
    ASSERT_NO_FATAL_FAILURE(Counts());
    const auto& main=main_test::ActualPostGapmSource();
    const auto forecast=MixedStarterSource::Preflight(main,Embedding());
    const auto qualified=QualificationBytes(forecast);
    const auto made=MixedStarterSource::Prepare(main,Embedding());
    auto doc=ResultDocument(made);
    output::Integer(doc,"qualification_peak_bytes",qualified);
    output::WriteJson(Destination()/"starter.json",doc); // Preserve typed failure diagnostics too.
    ASSERT_EQ(made.report.status,Status::Ready)<<made.report.reason;
    ASSERT_TRUE(made.source);
    const auto& source=*made.source;const auto& value=source.snapshot();
    ASSERT_TRUE(source.embedding());
    EXPECT_TRUE(source.domain().SharesStorage(wall_test::Wall().domain()));
    EXPECT_EQ(value.node_count,376934u);EXPECT_EQ(value.primary_count,341504u);
    EXPECT_EQ(value.shell_primary_count,337092u);EXPECT_EQ(value.main_count,678596u);
    EXPECT_EQ(value.raw_origin_count,341613u);
    EXPECT_EQ(value.post_gapm->pre_shell_internal_count,268u);
    EXPECT_EQ(value.post_gapm->final_solid_erosion,s::SolidErosion::Enabled);
    EXPECT_EQ(main.mixed().sides().node_count,376930u);
    const auto input=source.startup_input();const auto original=main.startup_input();
    for(std::size_t i=0;i<original.node_count;++i) {
        ASSERT_EQ(input.node_source_ids[i],original.node_source_ids[i]);
        const auto a=input.positions.at(std::uint32_t(i)),b=original.positions.at(std::uint32_t(i));
        ASSERT_EQ(output::Bits(a.x),output::Bits(b.x));
        ASSERT_EQ(output::Bits(a.y),output::Bits(b.y));
        ASSERT_EQ(output::Bits(a.z),output::Bits(b.z));
    }
    for(unsigned i=0;i<4;++i) {
        const auto at=original.node_count+i;const auto point=input.positions.at(std::uint32_t(at));
        const auto expected=wall_test::Wall().geometry().reference_native[i];
        EXPECT_EQ(input.node_source_ids[at],wall_test::Wall().ids().nodes[i]);
        EXPECT_EQ(output::Bits(point.x),output::Bits(expected.x));
        EXPECT_EQ(output::Bits(point.y),output::Bits(expected.y));
        EXPECT_EQ(output::Bits(point.z),output::Bits(expected.z));
    }
    std::size_t partners=0,internal=0;
    for(std::size_t i=0;i<value.main_count;++i) {
        const auto& before=main.mixed().sides().mains[i];const auto& after=value.mains[i];
        ASSERT_EQ(after.source_id,before.source_id);ASSERT_EQ(after.global_id,before.global_id);
        ASSERT_EQ(after.segment_type,before.segment_type);
        const bool primary=i<value.primary_count;
        for(unsigned k=0;k<4;++k) {
            const auto slot=primary?value.post_gapm->primary_corners[i].source_corner[k]:k;
            ASSERT_EQ(after.nodes[k],before.nodes[slot]);
            const auto normal=value.starter.face_normals[4*i+k];
            ASSERT_TRUE(std::isfinite(normal.x)&&std::isfinite(normal.y)&&std::isfinite(normal.z));
        }
        if(primary){partners+=value.primary_to_partner[i]!=0;internal+=value.post_gapm->final_support[i].second_solid_source_id!=0;}
    }
    EXPECT_EQ(partners,337092u);EXPECT_EQ(internal,268u);
    for(std::size_t i=0;i<value.starter.reference_count;++i) {
        const auto& ref=value.starter.references[i];ASSERT_GE(ref.boundary,0);ASSERT_LE(ref.boundary,1);
        for(const auto normal:ref.bisector)ASSERT_TRUE(std::isfinite(normal.x)&&std::isfinite(normal.y)&&std::isfinite(normal.z));
    }
    EXPECT_EQ(c::ValidateMixedSource(detail::NormalTopology(value),value).status,c::Status::Ok);
    EXPECT_LE(source.forecast().retained_bytes,forecast.retained_bytes);
    EXPECT_EQ(source.provenance().post_gapm_digest,main.provenance().output_digest);
}
}
