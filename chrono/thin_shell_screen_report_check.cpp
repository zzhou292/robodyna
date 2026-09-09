#include "ThinShellScreenReport.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <gtest/gtest.h>
#include <limits>

namespace {
namespace ref=crash::reference;
namespace io=crash::output;
io::Document Provenance() {
    io::Document d; d.SetObject(); io::String(d,"test_source","synthetic report contract"); return d;
}
TEST(ThinShellScreenReport, RejectedMeasurementsRemainVisibleWithoutSimulationAdmission) {
    ref::ThinShellScreenDiagnostic screen;
    for(std::size_t i=0;i<screen.fixtures.size();++i) {
        auto& f=screen.fixtures[i]; f.fixture_index=i;
        f.parameters=ref::ThinShellFixtureParameters(i); f.edge_m=f.parameters.length/2;
        f.diagnostic="Unresolved lowest branch";
        f.measurement[1].available=true;
        f.measurement[1].status=ref::ElasticCouponStatus::kSuccess;
        f.measurement[1].stiffness(3,7)=static_cast<double>(i)+.125;
        f.measurement[1].stiffness(7,3)=-2-static_cast<double>(i);
        f.policies[1].policy=ref::ThinShellInertiaPolicy::AreaCounterfactual;
        f.policies[0].spectrum[1].available=true;
        f.policies[0].spectrum[1].squared_frequency(0)=-.5;
        f.policies[0].spectrum[1].mass_modes(2,4)=.375;
    }
    const auto d=ref::ThinShellScreenReport(screen,Provenance());
    EXPECT_FALSE(d["screen_passed"].GetBool()); EXPECT_FALSE(d["simulation_ready"].GetBool());
    ASSERT_EQ(d["fixtures"].Size(),6u);
    for(unsigned i=0;i<6;++i) {
        const auto& f=d["fixtures"][i]; EXPECT_EQ(f["fixture_index"].GetUint64(),i);
        EXPECT_STREQ(f["diagnostic"].GetString(),"Unresolved lowest branch");
        const auto& matrix=f["raw_measurements_chrono_coarse_chrono_fine_tl_fine"][1]["stiffness"];
        EXPECT_EQ(matrix[3][7].GetDouble(),static_cast<double>(i)+.125);
        EXPECT_EQ(matrix[7][3].GetDouble(),-2-static_cast<double>(i));
        const auto& p=f["policies"][0]; EXPECT_FALSE(p["reference_passed"].GetBool());
        EXPECT_EQ(p["raw_spectrum_coarse_fine"][1]["squared_frequency"][0].GetDouble(),-.5);
        EXPECT_EQ(p["raw_spectrum_coarse_fine"][1]["mass_modes_columns"][2][4].GetDouble(),.375);
    }
    // Deeply nested values must survive all temporary allocators and roundtrip.
    rapidjson::StringBuffer bytes; rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);
    ASSERT_TRUE(d.Accept(writer)); io::Document reread; reread.Parse(bytes.GetString(),bytes.GetSize());
    ASSERT_FALSE(reread.HasParseError()); EXPECT_EQ(reread,d);
}
TEST(ThinShellScreenReport, SeparateArtificialInertiaAndModeMappingArePreserved) {
    ref::ThinShellScreenDiagnostic screen; auto& f=screen.fixtures[0];
    f.inertia.original[2].physical_tangential_inertia=.1;
    f.inertia.original[2].artificial_drilling_inertia=.1;
    f.inertia.added_tangential_inertia[2]=.7; f.inertia.added_drilling_inertia[2]=.7;
    f.inertia.counterfactual.total_isotropic_inertia[2]=.8;
    auto& m=f.hybrid_match; m.reference_pool.mode_count=1; m.candidate_pool.mode_count=2;
    m.reference_pool.coordinate_count=m.candidate_pool.coordinate_count=12;
    m.reference_modes[0]=0; m.candidate_modes[0]=3; m.candidate_modes[1]=8;
    m.reference_pool.translation[0][7]=.125; m.candidate_pool.translation[1][7]=-.5;
    m.comparison.mode_count=1; m.comparison.cluster_count=1;
    m.comparison.singleton_candidate[0]=1; m.comparison.clusters[0].mode_count=1;
    m.comparison.clusters[0].candidate_modes[0]=1;
    const auto d=ref::ThinShellScreenReport(screen,Provenance()); const auto& v=d["fixtures"][0];
    EXPECT_EQ(v["inertia"]["original_nodal_contributions"][2]["physical_tangential_inertia_kg_m2"].GetDouble(),.1);
    EXPECT_EQ(v["inertia"]["added_tangential_inertia_kg_m2"][2].GetDouble(),.7);
    EXPECT_EQ(v["inertia"]["counterfactual"]["total_isotropic_inertia_kg_m2"][2].GetDouble(),.8);
    EXPECT_EQ(v["hybrid_match"]["candidate_full_indices"][1].GetUint64(),8u);
    EXPECT_EQ(v["hybrid_match"]["candidate_pool"]["physical_translation"][1][7].GetDouble(),-.5);
    EXPECT_EQ(v["hybrid_match"]["comparison"]["singleton_candidate"][0].GetUint64(),1u);
}
TEST(ThinShellScreenReport, InvalidReadinessCapacityAndNonfiniteDataAreRejected) {
    ref::ThinShellScreenDiagnostic screen;
    screen.simulation_ready=true; EXPECT_THROW(ref::ThinShellScreenReport(screen,Provenance()),std::runtime_error);
    screen.simulation_ready=false; screen.fixtures[5].simulation_ready=true;
    EXPECT_THROW(ref::ThinShellScreenReport(screen,Provenance()),std::runtime_error);
    screen.fixtures[5].simulation_ready=false;
    screen.fixtures[5].measurement[2].stiffness(23,23)=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ref::ThinShellScreenReport(screen,Provenance()),std::runtime_error);
    screen.fixtures[5].measurement[2].stiffness(23,23)=0;
    screen.fixtures[5].hybrid_match.candidate_pool.mode_count=25;
    EXPECT_THROW(ref::ThinShellScreenReport(screen,Provenance()),std::runtime_error);
    screen.fixtures[5].hybrid_match.candidate_pool.mode_count=0;
    io::Document empty; empty.SetObject(); EXPECT_THROW(ref::ThinShellScreenReport(screen,empty),std::runtime_error);
}
} // namespace
