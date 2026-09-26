#include "../Internal.h"
#include "case/vehicle_runtime/source/tests/ActualFixture.h"
#include "case/vehicle_self_contact/native/mixed_main/tests/ActualFixture.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <cmath>
namespace crash::cases::vehicle_wall::native::wall_interface::test {
namespace upstream=vehicle_self_contact::native::post_gapm::test;
namespace owner_test=vehicle_runtime::source_test;
namespace {
constexpr std::size_t ExportBytes=2u<<20;
Declaration Declared(){return {Profile::AllRetainedVehicleNodesToFixedMeshV1,1,1};}
std::filesystem::path Destination() {
    const auto* text=std::getenv("ROBO_FINITE_WALL_SOURCE_OUTPUT");
    output::Require(text&&*text,"Missing create-only finite-wall source output");
    const std::filesystem::path path(text);
    output::Require(std::filesystem::create_directory(path),"Finite-wall source output exists");return path;
}
void ForecastFields(output::Document& doc,const Forecast& f) {
    output::Integer(doc,"owner_retained_bytes",f.owner_retained);output::Integer(doc,"owner_prior_peak_bytes",f.owner_prior_peak);
    output::Integer(doc,"vehicle_retained_bytes",f.vehicle_retained);output::Integer(doc,"vehicle_prior_peak_bytes",f.vehicle_prior_peak);
    output::Integer(doc,"own_retained_bytes",f.own_retained);output::Integer(doc,"coefficient_scratch_bytes",f.coefficient_scratch);
    output::Integer(doc,"topology_scratch_bytes",f.topology_scratch);output::Integer(doc,"gap_scratch_bytes",f.gap_scratch);
    output::Integer(doc,"constraint_packing_bytes",f.constraint_packing);output::Integer(doc,"gap_shell_copy_bytes",f.gap_shell_copy);
    output::Integer(doc,"digest_scratch_bytes",f.digest_scratch);output::Integer(doc,"own_peak_bytes",f.own_peak);
    output::Integer(doc,"coexistence_peak_bytes",f.coexistence_peak);output::Integer(doc,"complete_construction_bound",f.complete_construction_bound);
    output::Integer(doc,"qualification_peak_bytes",f.complete_construction_bound+ExportBytes);
    output::Boolean(doc,"fits_component_10GiB_guard",f.complete_construction_bound+ExportBytes<=10ull<<30);
}
void AdmitActual(const Forecast& f) {
    // Existing full-case20GB allowance, with the explicit owning18GiB RSS
    // invocation selected only after root reviews this complete forecast.
    output::Require(f.complete_construction_bound<=Limits{}.coexistence_bytes-ExportBytes &&
        f.complete_construction_bound<=(18ull<<30)-ExportBytes,
        "Finite-wall complete source exceeds the existing full-case allowance");
}
}
TEST(FiniteWallActual, CompleteSourceForecastAndOneByteShortAdmissionAreExplicit) {
    const auto& owner=owner_test::OwnerSource();const auto& vehicle=upstream::ActualPostGapmSource();
    ASSERT_EQ(owner.physical().domain()->node_count(),376934u);
    ASSERT_EQ(vehicle.startup_input().node_count,376930u);
    const auto f=FiniteWallContactSource::Preflight(owner,vehicle,Declared());AdmitActual(f);
    auto cap=Limits{};cap.coexistence_bytes=f.complete_construction_bound;
    EXPECT_EQ(FiniteWallContactSource::Preflight(owner,vehicle,Declared(),cap).complete_construction_bound,f.complete_construction_bound);
    --cap.coexistence_bytes;const auto rejected=FiniteWallContactSource::Prepare(owner,vehicle,Declared(),cap);
    EXPECT_EQ(rejected.report.status,Status::ResourceLimit);EXPECT_FALSE(rejected.source);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.finite_wall_interface_forecast.v1");
    ForecastFields(doc,f);output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(FiniteWallActual, AllRetainedNodesUseActualWallTopologyKAndInterfaceSpecificGaps) {
    const auto& owner=owner_test::OwnerSource();const auto& vehicle=upstream::ActualPostGapmSource();
    const auto f=FiniteWallContactSource::Preflight(owner,vehicle,Declared());AdmitActual(f);
    const auto made=FiniteWallContactSource::Prepare(owner,vehicle,Declared());
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.finite_wall_interface_source.v1");
    output::Integer(doc,"status",std::uint64_t(made.report.status));output::String(doc,"reason",made.report.reason);
    output::Integer(doc,"numerical_stage",std::uint64_t(made.report.numerical_stage));ForecastFields(doc,f);
    if(made.source) {
        output::String(doc,"output_digest",made.source->provenance().output_digest);
        output::Integer(doc,"interface_id",made.source->provenance().interface_id);
        output::Integer(doc,"nodes",made.source->nodes().size());output::Integer(doc,"secondaries",made.source->secondary_nodes().size());
        output::Boolean(doc,"final_initial_history_available",false);output::Boolean(doc,"final_removal_csr_available",false);
    }
    output::WriteJson(Destination()/"source.json",doc);
    ASSERT_EQ(made.report.status,Status::Prepared)<<made.report.reason;ASSERT_TRUE(made.source);
    const auto& source=*made.source;const auto& wall=source.wall();
    EXPECT_TRUE(source.domain().SharesStorage(*owner.physical().domain()));
    EXPECT_EQ(source.nodes().size(),376934u);EXPECT_EQ(source.secondary_nodes().size(),376934u);
    EXPECT_EQ(source.starter().primary_count,1u);EXPECT_EQ(source.starter().main_count,2u);
    EXPECT_EQ(source.starter().mains[0].source_id,wall.ids().shell);
    EXPECT_EQ(source.starter().primary_to_partner[0],2u);EXPECT_EQ(source.provenance().interface_id,wall.ids().interface);
    const auto original=vehicle.gap_operands().corrected().coefficients();
    for(std::size_t i=0;i<original.size();++i)ASSERT_EQ(output::Bits(source.global_coefficients()[i]),output::Bits(original[i]));
    std::uint64_t previous=0;std::size_t positive_vehicle_gaps=0;
    for(std::size_t row=0;row<source.secondary_nodes().size();++row) {
        const auto node=source.secondary_nodes()[row];ASSERT_LT(node,source.nodes().size());
        const auto id=source.nodes()[node].source_id;ASSERT_GT(id,previous);previous=id;
        EXPECT_EQ(id,source.domain().nodes()[node].source_id);
        ASSERT_EQ(output::Bits(source.secondary_coefficients()[row]),output::Bits(source.global_coefficients()[node]));
        ASSERT_TRUE(std::isfinite(source.secondary_gaps()[row]));ASSERT_GE(source.secondary_gaps()[row],0.);
        if(node<original.size())positive_vehicle_gaps+=source.secondary_gaps()[row]>0;
        else {
            EXPECT_EQ(source.nodes()[node].constraint,7);EXPECT_EQ(source.nodes()[node].skew,1);
            EXPECT_EQ(output::Bits(source.secondary_gaps()[row]),output::Bits(wall.geometry().native_half_gap));
        }
    }
    EXPECT_GT(positive_vehicle_gaps,0u); // Genuine structural beam contribution survives the mask.
    for(std::size_t i=0;i<2;++i) {
        EXPECT_EQ(output::Bits(source.main_coefficients()[i]),output::Bits(wall.geometry().component_primary_stiffness_native));
        for(unsigned k=0;k<4;++k) {
            EXPECT_EQ(output::Bits(source.main_gaps()[i].corner[k]),output::Bits(wall.geometry().native_half_gap));
            const auto a=source.starter().starter.face_normals[4*i+k],b=source.fixed_ready().normals.face_normals[4*i+k];
            EXPECT_TRUE(std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z));
            EXPECT_TRUE(std::isfinite(b.x)&&std::isfinite(b.y)&&std::isfinite(b.z));
        }
    }
    EXPECT_EQ(source.controls().runtime.friction_coefficients.base,.6);
    EXPECT_EQ(source.provenance().stage,Stage::PreparedBeforeGeneralInitialization);
}
}
