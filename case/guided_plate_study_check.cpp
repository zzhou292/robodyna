#include "chrono/core/ChMatrix.h"
#include "GuidedPlateStudy.h"
#include "guided_plate_study_fixture.h"
#include "chrono/core/ChQuaternion.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using namespace crash::case_data;
namespace ct=tlfea::contact;
namespace sh=tl::fea::reissner;
namespace ref=crash::reference;
namespace fixture=crash::case_data::study_test;
using namespace crash::case_data::study_test;

TEST(GuidedPlateStudy, ExactIntegerCommonScheduleHasNoDuplicatesOrChangedFailureOutput) {
    auto c=Config();c.base_steps=19997;
    for(unsigned r:{1u,2u,4u}) {
        c.refinement=r;std::uint64_t previous=0;
        for(std::size_t j=0;j<201;++j) {
            std::uint64_t epoch=999;ASSERT_TRUE(GuidedStudySampleEpoch(c,j,epoch));
            EXPECT_EQ(epoch,((j*19997+199)/200)*r);if(j)EXPECT_GT(epoch,previous);previous=epoch;
        }
    }
    std::uint64_t sentinel=123;EXPECT_FALSE(GuidedStudySampleEpoch(c,201,sentinel));EXPECT_EQ(sentinel,123u);
    c.base_steps=30000;EXPECT_FALSE(GuidedStudySampleEpoch(c,0,sentinel));EXPECT_EQ(sentinel,123u);
}

TEST(GuidedPlateStudy, ConfigurationReusesExactAreaAndSharedPhysicalReferenceFingerprint) {
    const auto c=Config();const auto f=Frame(c,0);ref::ElasticCouponData model;ref::GuidedPlateData guided;
    guided.wall_binding_id=c.wall_binding_id;guided.integration.force_error=1e-6;guided.integration.energy_error=1e-9;
    for(unsigned n=0;n<6;++n) {
        model.reference_configuration.position[n]={c.reference_position[3*n],c.reference_position[3*n+1],c.reference_position[3*n+2]};
        model.nodal_mass[n]={.01,1,.01,.01};
    }
    for(unsigned e=0;e<2;++e) {
        guided.parents[e]=c.contact_reference[e].parent;
        for(unsigned n=0;n<4;++n)model.connectivity[e][n]=guided.parents[e].nodes[n];
    }
    ct::Q4PlanarReferenceView view{c.contact_reference.data(),2,6,c.wall_x,1e-14};
    GuidedStudyConfig out;std::string error;
    ASSERT_TRUE(PrepareGuidedStudyConfig(f.metrics,model,guided,view,1,out,error))<<error;
    EXPECT_EQ(out.total_reference_area.lower,c.total_reference_area.lower);
    EXPECT_EQ(out.contact_reference[1].area_enclosure.upper,c.contact_reference[1].area_enclosure.upper);
    const auto original=out.experiment_sha256;model.nodal_mass[5].artificial_drilling_inertia*=2;
    ASSERT_TRUE(PrepareGuidedStudyConfig(f.metrics,model,guided,view,1,out,error))<<error;
    EXPECT_NE(original,out.experiment_sha256);
    const auto preserved=out.experiment_sha256;view.parent_count=1;
    EXPECT_FALSE(PrepareGuidedStudyConfig(f.metrics,model,guided,view,1,out,error));EXPECT_EQ(out.experiment_sha256,preserved);
}

TEST(GuidedPlateStudy, ActualChronoWorldLogAndNodalProxyMatchAnalyticalTurnAfterCommonPose) {
    auto c=Config();for(unsigned n=0;n<6;++n)for(unsigned j=0;j<4;++j)c.reference_rotation[4*n+j]=.5;
    const auto f=Frame(c,0);GuidedPlateStudy recorder;std::string error;
    ASSERT_TRUE(recorder.Initialize(c,f.metrics,f,error))<<error;
    const auto& sample=recorder.data()->samples[0];
    EXPECT_NEAR(sample.normal_displacement[0],-.002,1e-17);EXPECT_NEAR(sample.normal_displacement[1],-.0007,1e-17);
    EXPECT_NEAR(sample.curvature_proxy,-.06,2e-15);
    EXPECT_NEAR(sample.world_z_rotation[0],-.02,2e-15);EXPECT_NEAR(sample.world_z_rotation[1],-.007,2e-15);
    const long double gap=.05L-.04L;
    // Fixed clamp nodes are closest; compare exact supplied-double subtraction.
    const long double actual=static_cast<long double>(c.wall_x)-c.reference_position[3];
    EXPECT_LE(static_cast<long double>(sample.minimum_signed_gap.lower),actual);
    EXPECT_GE(static_cast<long double>(sample.minimum_signed_gap.upper),actual);EXPECT_NEAR(double(actual),double(gap),1e-17);
}

TEST(GuidedPlateStudy, LateInvalidCaptureStaleOwnerAndMissingSamplePreserveRecordForRetry) {
    const auto c=Config();auto zero=Frame(c,0);GuidedPlateStudy recorder;std::string error;
    ASSERT_TRUE(recorder.Initialize(c,zero.metrics,zero,error));auto first=Frame(c,1,&zero);
    ASSERT_TRUE(recorder.Record(first.metrics,&first,error));const auto before=recorder.data()->summary;
    const auto next=Frame(c,2,&first);
    EXPECT_FALSE(recorder.Record(next.metrics,nullptr,error));
    auto bad=next;bad.metrics.stamp.owner_id=8;EXPECT_FALSE(recorder.Record(bad.metrics,&bad,error));
    bad=next;bad.parent[1].integration.force[3].error=-1;EXPECT_FALSE(recorder.Record(bad.metrics,&bad,error));
    bad=next;bad.rotation[23]=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(recorder.Record(bad.metrics,&bad,error));
    bad=next;bad.metrics.contact.wall_reaction.x=bad.metrics.contact.force_error.x=std::numeric_limits<double>::max();
    EXPECT_FALSE(recorder.Record(bad.metrics,&bad,error));
    EXPECT_FALSE(recorder.Record(first.metrics,&first,error));
    EXPECT_EQ(recorder.data()->summary.accepted_epoch,before.accepted_epoch);
    EXPECT_EQ(recorder.data()->summary.sample_count,before.sample_count);
    EXPECT_EQ(recorder.data()->summary.normal_wall_impulse.value,before.normal_wall_impulse.value);
    ASSERT_TRUE(recorder.Record(next.metrics,&next,error))<<error;EXPECT_EQ(recorder.data()->summary.accepted_epoch,2u);
    GuidedStudyData sentinel;sentinel.config.owner_id=999;EXPECT_FALSE(recorder.Finish(sentinel,error));EXPECT_EQ(sentinel.config.owner_id,999u);
}

TEST(GuidedPlateStudy, RefreshedEndpointAttemptIsIndependentOfConsumedIntervalAttempt) {
    const auto c=Config();const auto zero=Frame(c,0);auto next=Frame(c,1,&zero);GuidedPlateStudy recorder;std::string error;
    ASSERT_TRUE(recorder.Initialize(c,zero.metrics,zero,error));
    next.element_association.phase=sh::ShellBatchPhase::kAcceptedBase;next.contact_association.phase=ct::Q4PlanarContactPhase::AcceptedBase;
    next.element_association.base_epoch=next.contact_association.base_epoch=1;
    next.element_association.attempt=next.contact_association.attempt=99;
    for(auto& p:next.parent) {p.integration.base_epoch=1;p.integration.attempt=99;}
    ASSERT_TRUE(recorder.Record(next.metrics,&next,error))<<error;
    EXPECT_EQ(recorder.data()->summary.last_attempt,next.metrics.shell.attempt);
}

TEST(GuidedPlateStudy, UncertaintyBracketsPressureReleaseAndTrueSeparationRemainDistinct) {
    const auto c=Config();auto before=Frame(c,0);GuidedPlateStudy recorder;std::string error;
    ASSERT_TRUE(recorder.Initialize(c,before.metrics,before,error));
    for(unsigned e=1;e<=8;++e) {
        auto next=Frame(c,e,&before);
        if(e==2||e==3||e==5)SetContact(next,1e-4,1e-5,Cert(1e-8,0,2e-8));
        if(e==4)SetContact(next,1,1e-5,Cert(.001,.00099,.00101));
        if(e>=6) {
            for(unsigned n=0;n<6;++n)next.position[3*n]=e==6?c.wall_x:c.wall_x-.001;
            next.velocity[12]=next.velocity[15]=e==7?.01:-.01;
        }
        ASSERT_TRUE(recorder.Record(next.metrics,&next,error))<<e<<":"<<error;before=next;
        if(e==6||e==7)EXPECT_FALSE(recorder.data()->summary.separated_rebounding);
    }
    const auto& s=recorder.data()->summary;
    ASSERT_TRUE(s.activation.observed);EXPECT_EQ(s.activation.lower_epoch,1u);EXPECT_EQ(s.activation.upper_epoch,4u);
    ASSERT_TRUE(s.pressure_release.observed);EXPECT_EQ(s.pressure_release.lower_epoch,4u);EXPECT_EQ(s.pressure_release.upper_epoch,6u);
    EXPECT_TRUE(s.separated_rebounding);EXPECT_EQ(s.separation_sample_epoch,8u);
}

TEST(GuidedPlateStudy, PeakCertificateTakesAllLowerAndUpperMaximaNotValueArgmax) {
    const auto c=Config();auto before=Frame(c,0);GuidedPlateStudy recorder;std::string error;
    ASSERT_TRUE(recorder.Initialize(c,before.metrics,before,error));
    auto first=Frame(c,1,&before);SetContact(first,10,5,Cert(.001,.001,.001));
    ASSERT_TRUE(recorder.Record(first.metrics,&first,error))<<error;
    auto second=Frame(c,2,&first);SetContact(second,11,.1,Cert(.001,.001,.001));
    ASSERT_TRUE(recorder.Record(second.metrics,&second,error))<<error;
    const auto& peak=recorder.data()->summary.sampled_peak_normal_force;
    EXPECT_DOUBLE_EQ(peak.value,11);EXPECT_NEAR(peak.lower,10.9,1e-13);EXPECT_NEAR(peak.upper,15,1e-13);
    EXPECT_GE(peak.error,4);EXPECT_GT(peak.upper,11.1);
}

TEST(GuidedPlateStudy, BaseImpulseAndEnergyUncertaintyHaveIndependentAnalyticalLedgers) {
    const auto c=Config();const auto zero=Frame(c,0);GuidedPlateStudy recorder;std::string error;
    ASSERT_TRUE(recorder.Initialize(c,zero.metrics,zero,error));
    auto first=Frame(c,1,&zero);SetContact(first,3,.2,Cert(0,0,.005));
    ASSERT_TRUE(recorder.Record(first.metrics,&first,error))<<error;
    auto second=Frame(c,2,&first);SetContact(second,1,.1,Cert(0,0,0));
    second.metrics.shell.elastic_energy=1.001;second.metrics.work.total_energy=1.001;
    second.element_association=second.metrics.shell;
    ASSERT_TRUE(recorder.Record(second.metrics,&second,error))<<error;
    const auto& s=recorder.data()->summary;
    EXPECT_DOUBLE_EQ(s.normal_wall_impulse.value,c.fixed_dt*3);
    EXPECT_LE(static_cast<long double>(s.normal_wall_impulse.lower),static_cast<long double>(c.fixed_dt)*2.8L);
    EXPECT_GE(static_cast<long double>(s.normal_wall_impulse.upper),static_cast<long double>(c.fixed_dt)*3.2L);
    EXPECT_GE(s.normal_wall_impulse.error,c.fixed_dt*.2);
    EXPECT_NEAR(s.maximum_value_energy_relative_error,.001,1e-15);
    EXPECT_GE(s.maximum_certified_energy_relative_error,.005);EXPECT_LT(s.maximum_certified_energy_relative_error,.005000000001);
}

TEST(GuidedPlateStudy, IndependentOwnersCompleteSameCommonTimesAndFrozenComparison) {
    const auto a=fixture::Run(1,7),b=fixture::Run(2,77),c=fixture::Run(4,777);GuidedStudyComparison result;std::string error;
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;EXPECT_TRUE(result.passed)<<result.diagnostic;
    ASSERT_TRUE(CompareGuidedPlateStudies(b,c,result,error))<<error;EXPECT_TRUE(result.passed)<<result.diagnostic;
    EXPECT_TRUE(result.events_complete);EXPECT_TRUE(result.deforming_contact_evidence);EXPECT_TRUE(result.energy_envelopes);
    EXPECT_GT(result.force_ratio,0);EXPECT_LT(result.force_ratio,1);
    EXPECT_EQ(a.summary.sample_count,201u);EXPECT_EQ(c.summary.accepted_epoch,800u);
}

TEST(GuidedPlateStudy, MeasuredGlobalCertificateCanFailForceFloorWithoutMalformedInput) {
    auto a=fixture::Run(1,7);const auto b=fixture::Run(2,77);GuidedStudyComparison result;std::string error;
    a.samples[0].normal_wall_force=Cert(0,0,1.1e-5);
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;
    EXPECT_FALSE(result.passed);EXPECT_GT(result.force_ratio,1);
    a=fixture::Run(1,7);a.summary.separated_rebounding=false;
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;
    EXPECT_FALSE(result.passed);EXPECT_FALSE(result.deforming_contact_evidence);
}

TEST(GuidedPlateStudy, MalformedOrDifferentExperimentPreservesComparisonOutput) {
    auto a=fixture::Run(1,7);const auto b=fixture::Run(2,77);GuidedStudyComparison result;result.force_ratio=123;result.passed=true;std::string error;
    a.config.experiment_sha256[0]='b';EXPECT_FALSE(CompareGuidedPlateStudies(a,b,result,error));
    EXPECT_EQ(result.force_ratio,123);EXPECT_TRUE(result.passed);
    a=fixture::Run(1,7);a.samples.back().epoch--;
    EXPECT_FALSE(CompareGuidedPlateStudies(a,b,result,error));EXPECT_EQ(result.force_ratio,123);
    a=fixture::Run(1,7);a.samples[30].contact_potential={0,0,1,0};
    EXPECT_FALSE(CompareGuidedPlateStudies(a,b,result,error));EXPECT_EQ(result.force_ratio,123);
    a=fixture::Run(1,7);ASSERT_TRUE(ValidateGuidedPlateStudy(a,error));a.config.contact_reference[0].reference_projection[0].y=1;
    EXPECT_FALSE(ValidateGuidedPlateStudy(a,error));
    a=fixture::Run(1,7);a.config.contact_reference[0].projected_area=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(ValidateGuidedPlateStudy(a,error));
    a=fixture::Run(1,7);a.summary.maximum_curvature*=2;EXPECT_FALSE(ValidateGuidedPlateStudy(a,error));
    a=fixture::Run(1,7);a.samples[100].world_z_rotation[0]=.1;EXPECT_FALSE(ValidateGuidedPlateStudy(a,error));
    a=fixture::Run(1,7);a.summary.maximum_certified_energy_relative_error=0;EXPECT_FALSE(ValidateGuidedPlateStudy(a,error));
    a=fixture::Run(1,7);a.summary.maximum_penetration=0;EXPECT_FALSE(ValidateGuidedPlateStudy(a,error));
}
} // namespace
