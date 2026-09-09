#include "chrono/core/ChMatrix.h"
#include "GuidedPlateStudy.h"
#include "guided_plate_study_fixture.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
using namespace crash::case_data;
namespace fixture=crash::case_data::study_test;
constexpr std::uint64_t kDerivedBinding=24;
using Backend=tlfea::contact::Q4PlanarIntegrationBackend;

// Completed synthetic observer inputs exercise numerical comparison only.
// Authentication of actual wall meshes and file identities is tested separately.
GuidedStudyData Derived(Backend backend=Backend::ScalarDyadicSquares) {
    auto config=fixture::Config(1,1);config.wall_binding_id=kDerivedBinding;
    config.integration_backend=backend;
    return fixture::Run(config);
}
std::array<double,8> Ratios(const GuidedStudyComparison& r) {
    return {r.displacement_ratio,r.velocity_ratio,r.rotation_ratio,r.force_ratio,
            r.impulse_ratio,r.energy_ratio,r.event_ratio,r.penetration_ratio};
}
GuidedStudyComparison Sentinel() {
    GuidedStudyComparison r;r.passed=true;r.displacement_ratio=11;r.velocity_ratio=12;r.rotation_ratio=13;
    r.force_ratio=14;r.impulse_ratio=15;r.energy_ratio=16;r.event_ratio=17;r.penetration_ratio=18;
    r.energy_envelopes=true;r.events_complete=true;r.deforming_contact_evidence=true;r.diagnostic="Preserved result";return r;
}
void SameResult(const GuidedStudyComparison& a,const GuidedStudyComparison& b) {
    EXPECT_EQ(Ratios(a),Ratios(b));EXPECT_EQ(a.passed,b.passed);EXPECT_EQ(a.energy_envelopes,b.energy_envelopes);
    EXPECT_EQ(a.events_complete,b.events_complete);EXPECT_EQ(a.deforming_contact_evidence,b.deforming_contact_evidence);
    EXPECT_EQ(a.diagnostic,b.diagnostic);
}
void RefreshRotationExtrema(GuidedStudyData& d) {
    d.summary.minimum_rotation=d.summary.maximum_rotation=d.samples[0].world_z_rotation;
    for(const auto& s:d.samples)for(unsigned j=0;j<2;++j) {
        d.summary.minimum_rotation[j]=std::min(d.summary.minimum_rotation[j],s.world_z_rotation[j]);
        d.summary.maximum_rotation[j]=std::max(d.summary.maximum_rotation[j],s.world_z_rotation[j]);
    }
}

TEST(GuidedPlateWallStudy, DistinctWallsAtSameStepPassWithProcessLocalOwnerOne) {
    for(const auto backend:{Backend::ScalarDyadicSquares,Backend::RectangularDyadic}) {
        auto config=fixture::Config(1,1);config.integration_backend=backend;
        const auto derived=Derived(backend),canonical=fixture::Run(config);GuidedStudyComparison out;std::string error;
        ASSERT_EQ(derived.config.owner_id,canonical.config.owner_id);ASSERT_EQ(derived.config.owner_id,1u);
        ASSERT_NE(derived.config.wall_binding_id,canonical.config.wall_binding_id);
        ASSERT_EQ(derived.config.fixed_dt,canonical.config.fixed_dt);
        ASSERT_TRUE(CompareGuidedPlateWallStudies(derived,canonical,out,error))<<error;
        EXPECT_TRUE(error.empty());EXPECT_TRUE(out.passed)<<out.diagnostic;
        EXPECT_TRUE(out.energy_envelopes);EXPECT_TRUE(out.events_complete);EXPECT_TRUE(out.deforming_contact_evidence);
        EXPECT_EQ(out.displacement_ratio,0);EXPECT_EQ(out.velocity_ratio,0);EXPECT_EQ(out.rotation_ratio,0);
        EXPECT_GT(out.force_ratio,0);EXPECT_LE(out.force_ratio,1);
        const auto sentinel=Sentinel();out=sentinel;
        EXPECT_FALSE(CompareGuidedPlateStudies(derived,canonical,out,error));SameResult(out,sentinel);
    }
}

TEST(GuidedPlateWallStudy, CanonicalResponseSuppliesDirectedRelativeScales) {
    auto derived=Derived();const auto canonical=fixture::Run(1,1);GuidedStudyComparison forward,reverse;std::string error;
    derived.samples[0].normal_displacement[0]*=1.051;
    ASSERT_TRUE(CompareGuidedPlateWallStudies(derived,canonical,forward,error))<<error;
    EXPECT_FALSE(forward.passed);EXPECT_GT(forward.displacement_ratio,1);
    // This reversed call is a math-direction check, not a claim that the
    // synthetic derived wall is an authenticated canonical source.
    ASSERT_TRUE(CompareGuidedPlateWallStudies(canonical,derived,reverse,error))<<error;
    EXPECT_TRUE(reverse.passed)<<reverse.diagnostic;EXPECT_LT(reverse.displacement_ratio,1);
    const long double difference=std::abs(static_cast<long double>(derived.samples[0].normal_displacement[0])-
                                        canonical.samples[0].normal_displacement[0]);
    const long double expected=difference/(std::abs(static_cast<long double>(canonical.samples[0].normal_displacement[0]))/20);
    EXPECT_GE(static_cast<long double>(forward.displacement_ratio),expected);
    EXPECT_NEAR(forward.displacement_ratio,static_cast<double>(expected),2e-14);
}

TEST(GuidedPlateWallStudy, ChangedPhysicalExperimentOrSchedulePreservesAllOutput) {
    const auto canonical=fixture::Run(1,1),original=Derived();const auto sentinel=Sentinel();std::string error;
    for(unsigned variant=0;variant<23;++variant) {
        SCOPED_TRACE(variant);auto d=original;
        if(variant==0)d.config.wall_binding_id=canonical.config.wall_binding_id;
        if(variant==1)d.config.wall_binding_id=0;
        if(variant==2)d.config.owner_id=0;
        if(variant==3)d.config.experiment_sha256[0]='b';
        if(variant==4) {
            for(unsigned n=0;n<6;++n)d.config.reference_position[3*n+1]=.01;
            for(auto& p:d.config.contact_reference)for(auto& point:p.reference_projection)point.y=.01;
        }
        if(variant==5)for(unsigned n=0;n<6;++n)for(unsigned j=0;j<4;++j)d.config.reference_rotation[4*n+j]=.5;
        if(variant==6)d.initial_position[0]=std::nextafter(d.initial_position[0],1.);
        if(variant==7)for(unsigned j=0;j<4;++j)d.initial_rotation[j]=-d.initial_rotation[j];
        if(variant==8)++d.config.contact_reference[1].parent.feature_id;
        if(variant==9)++d.config.contact_reference[1].parent.parent_face_id;
        if(variant==10)d.config.contact_reference[0].projected_area=std::nextafter(d.config.contact_reference[0].projected_area,1.);
        if(variant==11) {
            for(auto& p:d.config.contact_reference){p.area_enclosure.lower*=2;p.area_enclosure.upper*=2;}
            d.config.total_reference_area.lower*=2;d.config.total_reference_area.upper*=2;
        }
        if(variant==12) {
            auto config=d.config;config.base_steps=400;config.fixed_dt=.0005;d=fixture::Run(config);
        }
        if(variant==13)d.config.fixed_dt=std::nextafter(d.config.fixed_dt,1.);
        if(variant==14) {
            auto config=d.config;config.refinement=2;config.fixed_dt=.0005;d=fixture::Run(config);
        }
        if(variant==15)d.config.horizon=std::nextafter(d.config.horizon,1.);
        if(variant==16)++d.config.qualification_id;
        if(variant==17)std::swap(d.config.contact_reference[0].parent.nodes[0],d.config.contact_reference[0].parent.nodes[1]);
        if(variant==18)d.config.contact_reference[0].reference_projection[0].y=-0.;
        if(variant==19)d.samples.back().normal_velocity[1]=std::numeric_limits<double>::quiet_NaN();
        if(variant==20)--d.samples.back().epoch;
        if(variant==21) {auto config=d.config;config.integration_backend=Backend::RectangularDyadic;d=fixture::Run(config);}
        if(variant==22)d.config.integration_backend=static_cast<Backend>(2);
        if(variant!=1&&variant!=2&&variant!=19&&variant!=20&&variant!=22)
            ASSERT_TRUE(ValidateGuidedPlateStudy(d,error))<<error;
        auto out=sentinel;EXPECT_FALSE(CompareGuidedPlateWallStudies(d,canonical,out,error));EXPECT_FALSE(error.empty());SameResult(out,sentinel);
    }
}

TEST(GuidedPlateWallStudy, ValidFailedResponsePublishesEveryRatioAndPhysicalOutcome) {
    auto d=Derived();const auto canonical=fixture::Run(1,1);GuidedStudyComparison out=Sentinel();std::string error;
    d.samples[0].normal_displacement[0]=-.003;d.samples[0].normal_velocity[0]=.002;
    d.samples[0].world_z_rotation[0]=.023;RefreshRotationExtrema(d);
    d.samples[100].normal_wall_force=fixture::Cert(2,2,2);d.summary.sampled_peak_normal_force=fixture::Cert(2,2,2);
    d.summary.normal_wall_impulse=fixture::Cert(.12,.119,.121);d.summary.maximum_penetration=.00002;
    d.samples[100].energy[0]=.0001;d.summary.maximum_value_energy_relative_error=.001;
    d.summary.maximum_certified_energy_relative_error=.001;
    d.summary.activation.upper_epoch=60;d.summary.activation.upper_time=.06;
    ASSERT_TRUE(ValidateGuidedPlateStudy(d,error))<<error;
    ASSERT_TRUE(CompareGuidedPlateWallStudies(d,canonical,out,error))<<error;
    EXPECT_FALSE(out.passed);EXPECT_TRUE(error.empty());EXPECT_NE(out.diagnostic,Sentinel().diagnostic);
    for(double ratio:Ratios(out)){EXPECT_TRUE(std::isfinite(ratio));EXPECT_GT(ratio,1);}
    EXPECT_TRUE(out.energy_envelopes);EXPECT_TRUE(out.events_complete);EXPECT_TRUE(out.deforming_contact_evidence);
    const auto prior=out;d.summary.separated_rebounding=false;
    ASSERT_TRUE(CompareGuidedPlateWallStudies(d,canonical,out,error))<<error;
    EXPECT_FALSE(out.passed);EXPECT_FALSE(out.deforming_contact_evidence);EXPECT_EQ(Ratios(out),Ratios(prior));
}

TEST(GuidedPlateWallStudy, SharedDirectedUncertaintyRejectsSubUlpFloorExcess) {
    auto d=Derived(),canonical=fixture::Run(1,1);GuidedStudyComparison out;std::string error;
    constexpr double floor=1e-5;
    const double extra=.25*(std::nextafter(floor,std::numeric_limits<double>::infinity())-floor);
    ASSERT_EQ(floor+extra,floor);d.samples[0].normal_wall_force.error=floor;canonical.samples[0].normal_wall_force.error=extra;
    ASSERT_TRUE(CompareGuidedPlateWallStudies(d,canonical,out,error))<<error;
    EXPECT_FALSE(out.passed);EXPECT_GT(out.force_ratio,1);
    canonical.samples[0].normal_wall_force.error=0;
    ASSERT_TRUE(CompareGuidedPlateWallStudies(d,canonical,out,error))<<error;
    EXPECT_TRUE(out.passed)<<out.diagnostic;EXPECT_EQ(out.force_ratio,1);
}
} // namespace
