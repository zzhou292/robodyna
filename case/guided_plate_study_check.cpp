#include "chrono/core/ChMatrix.h"
#include "GuidedPlateStudy.h"
#include "guided_plate_study_fixture.h"
#include "chrono/core/ChQuaternion.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
using namespace crash::case_data;
namespace ct=tlfea::contact;
namespace sh=tl::fea::reissner;
namespace ref=crash::reference;
namespace fixture=crash::case_data::study_test;
using namespace crash::case_data::study_test;
using Backend=ct::Q4PlanarIntegrationBackend;
template<class T> std::array<unsigned char,sizeof(T)> ObjectBytes(const T& value) {
    std::array<unsigned char,sizeof(T)> bytes{};std::memcpy(bytes.data(),&value,sizeof(T));return bytes;
}

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
    guided.wall_binding_id=c.wall_binding_id;guided.integration.force_error=ref::kGuidedOriginalForceError;
    guided.integration.energy_error=ref::kGuidedOriginalEnergyError;
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
    const auto original=out.experiment_sha256;
    auto revised_guided=guided;revised_guided.experiment=ref::GuidedPlateExperiment::PenaltyMarginV1;
    revised_guided.qualification_id=ref::kGuidedPenaltyMarginV1Experiment.qualification_id;
    revised_guided.stiffness_per_area=ref::kGuidedPenaltyMarginV1Experiment.stiffness_per_area;
    auto revised_metrics=f.metrics;revised_metrics.shell.configuration_id=revised_metrics.contact.configuration_id=revised_guided.qualification_id;
    GuidedStudyConfig revised;
    ASSERT_TRUE(PrepareGuidedStudyConfig(revised_metrics,model,revised_guided,view,1,revised,error))<<error;
    EXPECT_EQ(revised.experiment,revised_guided.experiment);EXPECT_NE(revised.experiment_sha256,original);
    for(unsigned fault=0;fault<6;++fault) {
        auto bad=revised_guided;
        if(fault==0)bad.qualification_id=kGuidedPlateQualification;
        if(fault==1)bad.stiffness_per_area*=2;
        if(fault==2)bad.integration.force_error*=2;
        if(fault==3)bad.integration.energy_error*=2;
        if(fault==4)++bad.integration.max_leaves;
        if(fault==5)bad.target_penetration=bad.maximum_penetration;
        const auto preserved=revised.experiment_sha256;
        EXPECT_FALSE(PrepareGuidedStudyConfig(revised_metrics,model,bad,view,1,revised,error));EXPECT_EQ(revised.experiment_sha256,preserved);
    }
    EXPECT_EQ(out.integration_backend,Backend::ScalarDyadicSquares);
    auto rectangular=f.metrics;rectangular.contact.integration_backend=Backend::RectangularDyadic;
    ASSERT_TRUE(PrepareGuidedStudyConfig(rectangular,model,guided,view,1,out,error))<<error;
    EXPECT_EQ(out.integration_backend,Backend::RectangularDyadic);EXPECT_EQ(out.experiment_sha256,original);
    const auto retained=ObjectBytes(out);rectangular.contact.integration_backend=static_cast<Backend>(2);
    EXPECT_FALSE(PrepareGuidedStudyConfig(rectangular,model,guided,view,1,out,error));
    EXPECT_EQ(ObjectBytes(out),retained);EXPECT_EQ(out.experiment_sha256,original);
    model.nodal_mass[5].artificial_drilling_inertia*=2;
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
    bad=next;bad.metrics.stamp.temporal_scheme=tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
    EXPECT_FALSE(recorder.Record(bad.metrics,&bad,error));
    bad=next;bad.stamp.velocity_phase=tl::fea::NodalVelocityPhase::PreviousMidpoint;
    EXPECT_FALSE(recorder.Record(bad.metrics,&bad,error));
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

TEST(GuidedPlateStudy, SubUlpUncertaintyCannotRoundAnExceededForceFloorIntoPassing) {
    auto a=fixture::Run(1,7),b=fixture::Run(2,77);GuidedStudyComparison result;std::string error;
    constexpr double floor=1e-5;
    const double extra=.25*(std::nextafter(floor,std::numeric_limits<double>::infinity())-floor);
    ASSERT_EQ(floor+extra,floor); // The previously rounded uncertainty sum lost this term.
    ASSERT_GT(static_cast<long double>(floor)+extra,static_cast<long double>(floor));
    a.samples[0].normal_wall_force.error=floor;b.samples[0].normal_wall_force.error=extra;
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;
    EXPECT_FALSE(result.passed);EXPECT_GT(result.force_ratio,1);EXPECT_TRUE(std::isfinite(result.force_ratio));
    b.samples[0].normal_wall_force.error=0;
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;
    EXPECT_TRUE(result.passed)<<result.diagnostic;EXPECT_EQ(result.force_ratio,1);
    a.samples[0].normal_wall_force.error=b.samples[0].normal_wall_force.error=std::numeric_limits<double>::max();
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;
    EXPECT_FALSE(result.passed);EXPECT_EQ(result.force_ratio,std::numeric_limits<double>::max());
}

TEST(GuidedPlateStudy, EventWidthAtStepFloorPassesButNextRepresentableWidthFails) {
    auto a=fixture::Run(1,7),b=fixture::Run(2,77);GuidedStudyComparison result;std::string error;
    // Synthetic observer summaries isolate the comparison boundary; this is
    // not a claim that these earlier activation times came from the fixture.
    const double floor=2*a.config.fixed_dt;
    a.summary.activation={true,0,2,0,floor};b.summary.activation={true,0,4,0,floor};
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;
    EXPECT_TRUE(result.passed)<<result.diagnostic;EXPECT_EQ(result.event_ratio,1);
    a.summary.activation.upper_time=std::nextafter(floor,std::numeric_limits<double>::infinity());
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;
    EXPECT_FALSE(result.passed);EXPECT_GT(result.event_ratio,1);
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

TEST(GuidedPlateStudy, BackendAndPartitionMismatchRejectBeforeInitializationAndPermitRetry) {
    for(const auto backend:{Backend::ScalarDyadicSquares,Backend::RectangularDyadic}) {
        auto c=Config();c.integration_backend=backend;const auto zero=Frame(c,0);
        for(unsigned variant=0;variant<6;++variant) {
            SCOPED_TRACE(variant);auto bad_config=c;auto bad=zero;GuidedPlateStudy recorder;std::string error;
            if(variant==0)bad_config.integration_backend=static_cast<Backend>(2);
            if(variant==1)bad.metrics.contact.integration_backend=backend==Backend::ScalarDyadicSquares ? Backend::RectangularDyadic : Backend::ScalarDyadicSquares;
            if(variant==2)bad.contact_association.integration_backend=static_cast<Backend>(2);
            if(variant==3)bad.parent[1].integration_backend=static_cast<Backend>(2);
            if(variant==4)bad.metrics.contact.deepest_u=ct::MaxQ4IntegrationDepth+1;
            if(variant==5)bad.parent[1].integration.leaf_count=0;
            EXPECT_FALSE(recorder.Initialize(bad_config,bad.metrics,bad,error));EXPECT_EQ(recorder.data(),nullptr);
            ASSERT_TRUE(recorder.Initialize(c,zero.metrics,zero,error))<<error;
            EXPECT_EQ(recorder.data()->config.integration_backend,backend);EXPECT_EQ(recorder.data()->summary.accepted_epoch,0u);
        }
    }
}

TEST(GuidedPlateStudy, RejectedBackendDepthAndCaptureMetadataPreserveEntireAcceptedHistory) {
    for(const auto backend:{Backend::ScalarDyadicSquares,Backend::RectangularDyadic}) {
        auto c=Config();c.integration_backend=backend;const auto zero=Frame(c,0);const auto first=Frame(c,1,&zero);
        GuidedPlateStudy recorder;std::string error;ASSERT_TRUE(recorder.Initialize(c,zero.metrics,zero,error));
        ASSERT_TRUE(recorder.Record(first.metrics,&first,error));const auto before=ObjectBytes(*recorder.data());
        const auto physical_hash=recorder.data()->config.experiment_sha256;const auto next=Frame(c,2,&first);
        const auto other=backend==Backend::ScalarDyadicSquares ? Backend::RectangularDyadic : Backend::ScalarDyadicSquares;
        for(unsigned variant=0;variant<18;++variant) {
            SCOPED_TRACE(variant);auto metrics=next.metrics;auto frame=next;
            if(variant==0)metrics.contact.integration_backend=other;
            if(variant==1)metrics.applied_contact.integration_backend=other;
            if(variant==2)frame.contact_association.integration_backend=other;
            if(variant==3)frame.parent[1].integration_backend=other;
            if(variant==4)frame.metrics.contact.integration_backend=other;
            if(variant==5)frame.metrics.applied_contact.integration_backend=other;
            if(variant==6)metrics.contact.deepest_u=17;
            if(variant==7)metrics.applied_contact.deepest_u=17;
            if(variant==8)frame.contact_association.deepest_v=17;
            if(variant==9)frame.parent[1].deepest_v=17;
            if(variant==10)metrics.contact.deepest_leaf=1;
            if(variant==11)metrics.applied_contact.deepest_leaf=1;
            if(variant==12)frame.parent[1].integration.deepest_leaf=1;
            if(variant==13)frame.contact_association.visited=2*ct::MaxQ4IntegrationVisits+1;
            if(variant==14)frame.parent[1].integration.leaf_count=0;
            if(variant==15) {
                auto& p=frame.parent[1];p.deepest_u=1;p.integration.deepest_leaf=1;
                p.integration.leaf_count=2;p.integration.visited=3;
            }
            if(variant==16)metrics.contact.integration_backend=static_cast<Backend>(2);
            if(variant==17)frame.parent[1].integration_backend=static_cast<Backend>(2);
            EXPECT_FALSE(recorder.Record(metrics,&frame,error));EXPECT_FALSE(error.empty());
            EXPECT_EQ(ObjectBytes(*recorder.data()),before);EXPECT_EQ(recorder.data()->config.experiment_sha256,physical_hash);
        }
        ASSERT_TRUE(recorder.Record(next.metrics,&next,error))<<error;
        EXPECT_EQ(recorder.data()->summary.accepted_epoch,2u);EXPECT_EQ(recorder.data()->summary.sample_count,3u);
        EXPECT_EQ(recorder.data()->samples[2].epoch,2u);
    }
}

TEST(GuidedPlateStudy, RectangularReportsCompleteAndRefineOnlyWithinTheSameBackend) {
    auto c=Config(1,1);c.integration_backend=Backend::RectangularDyadic;
    const auto a=fixture::Run(c);c.refinement=2;c.fixed_dt*=.5;const auto b=fixture::Run(c);
    GuidedStudyComparison result;std::string error;
    ASSERT_TRUE(ValidateGuidedPlateStudy(a,error))<<error;
    ASSERT_TRUE(CompareGuidedPlateStudies(a,b,result,error))<<error;EXPECT_TRUE(result.passed)<<result.diagnostic;
    EXPECT_EQ(a.config.integration_backend,Backend::RectangularDyadic);
    const auto retained=ObjectBytes(result);const auto diagnostic=result.diagnostic;
    c.integration_backend=Backend::ScalarDyadicSquares;const auto scalar=fixture::Run(c);
    EXPECT_EQ(scalar.config.experiment_sha256,b.config.experiment_sha256);
    EXPECT_FALSE(CompareGuidedPlateStudies(a,scalar,result,error));
    EXPECT_EQ(ObjectBytes(result),retained);EXPECT_EQ(result.diagnostic,diagnostic);
    auto malformed=b;malformed.config.integration_backend=static_cast<Backend>(2);
    EXPECT_FALSE(ValidateGuidedPlateStudy(malformed,error));
    EXPECT_FALSE(CompareGuidedPlateStudies(a,malformed,result,error));
    EXPECT_EQ(ObjectBytes(result),retained);EXPECT_EQ(result.diagnostic,diagnostic);
    // A rectangular-only asymmetric partition is valid execution metadata.
    const auto zero=Frame(a.config,0);auto first=Frame(a.config,1,&zero);GuidedPlateStudy observer;
    for(auto& p:first.parent) {p.deepest_u=1;p.integration.deepest_leaf=1;p.integration.leaf_count=2;p.integration.visited=3;}
    first.contact_association.deepest_u=1;first.contact_association.deepest_leaf=1;
    first.contact_association.leaves=4;first.contact_association.visited=6;
    ASSERT_TRUE(observer.Initialize(a.config,zero.metrics,zero,error));
    ASSERT_TRUE(observer.Record(first.metrics,&first,error))<<error;
}
TEST(GuidedPlateStudy, KnownExperimentMismatchCannotPassEitherComparisonWithSameClaimedHash) {
    const auto original=fixture::Run(Config());auto fine_config=Config(2);
    fine_config.experiment=ref::GuidedPlateExperiment::PenaltyMarginV1;
    fine_config.qualification_id=ref::kGuidedPenaltyMarginV1Experiment.qualification_id;
    const auto fine=fixture::Run(fine_config);GuidedStudyComparison sentinel;sentinel.diagnostic="preserved";sentinel.force_ratio=17;
    auto out=sentinel;std::string error;EXPECT_FALSE(CompareGuidedPlateStudies(original,fine,out,error));
    EXPECT_EQ(out.diagnostic,sentinel.diagnostic);EXPECT_EQ(out.force_ratio,sentinel.force_ratio);
    auto wall_config=fine_config;wall_config.refinement=1;wall_config.fixed_dt=.001;wall_config.wall_binding_id=24;
    const auto wall=fixture::Run(wall_config);EXPECT_FALSE(CompareGuidedPlateWallStudies(wall,original,out,error));
    EXPECT_EQ(out.diagnostic,sentinel.diagnostic);EXPECT_EQ(out.force_ratio,sentinel.force_ratio);
    wall_config.qualification_id=kGuidedPlateQualification;EXPECT_THROW(fixture::Run(wall_config),std::runtime_error);
}
} // namespace
