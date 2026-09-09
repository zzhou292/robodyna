#include "GuidedPlatePenaltyAudit.h"
#include "lib_utest/q4_planar_geometry_fixture.h"
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace crash::reference {
namespace {
using Status=ElasticCouponStatus;
using Experiment=GuidedPlateExperiment;
namespace sc=tlfea::contact;
constexpr std::uint64_t Binding=0x50454e414c545931ULL;
std::string Precise(double value) { std::ostringstream out; out.precision(17); out<<value; return out.str(); }
void SameConfiguration(const ElasticCouponConfiguration& a,const ElasticCouponConfiguration& b) {
    for (unsigned n=0;n<kCouponNodes;++n) {
        EXPECT_EQ(a.position[n].x,b.position[n].x); EXPECT_EQ(a.position[n].y,b.position[n].y); EXPECT_EQ(a.position[n].z,b.position[n].z);
        EXPECT_EQ(a.rotation[n].w,b.rotation[n].w); EXPECT_EQ(a.rotation[n].x,b.rotation[n].x);
        EXPECT_EQ(a.rotation[n].y,b.rotation[n].y); EXPECT_EQ(a.rotation[n].z,b.rotation[n].z);
    }
}
class GuidedPlatePenalty : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
        const auto wall=q4_planar_test::Square();
        original=std::make_unique<GuidedPlateModel>(wall.view(),Binding);
        revised=std::make_unique<GuidedPlateModel>(wall.view(),Binding,Experiment::PenaltyMarginV1);
        std::string error;
        ASSERT_EQ(AuditGuidedPlate(*original,original_modal,error),Status::kSuccess)<<error;
        ASSERT_EQ(AuditGuidedPlate(*revised,revised_modal,error),Status::kSuccess)<<error;
    }
    static void TearDownTestSuite() { original.reset(); revised.reset(); }
    static inline std::unique_ptr<GuidedPlateModel> original,revised;
    static inline GuidedPlateModalReport original_modal,revised_modal;
};

TEST_F(GuidedPlatePenalty, NamedPresetsKeepOriginalAbsoluteBudgetsAndRejectUnknownBeforeSetup) {
    const auto* old=FindGuidedPlateExperiment(Experiment::Original);
    const auto* next=FindGuidedPlateExperiment(Experiment::PenaltyMarginV1);
    ASSERT_NE(old,nullptr); ASSERT_NE(next,nullptr);
    EXPECT_STREQ(old->name,"original"); EXPECT_STREQ(next->name,"penalty-margin-v1");
    EXPECT_EQ(old->qualification_id,0x4432475549444531ULL); EXPECT_EQ(next->qualification_id,0x4432475549444532ULL);
    EXPECT_EQ(old->stiffness_per_area,1e5); EXPECT_EQ(next->stiffness_per_area,4e5);
    const double area=.5*ElasticCouponData::length*ElasticCouponData::width;
    const double force=1e-6*1e5*area*GuidedPlateData::maximum_penetration;
    const double energy=5e-9*1e5*area*GuidedPlateData::maximum_penetration*GuidedPlateData::maximum_penetration;
    for (const auto* spec:{old,next}) {
        EXPECT_EQ(std::memcmp(&force,&spec->force_error,sizeof(double)),0);
        EXPECT_EQ(std::memcmp(&energy,&spec->energy_error,sizeof(double)),0);
        EXPECT_EQ(spec->maximum_penetration,.0005); EXPECT_EQ(spec->target_penetration,.000375);
    }
    const auto invalid=static_cast<Experiment>(999);
    EXPECT_EQ(FindGuidedPlateExperiment(invalid),nullptr);
    EXPECT_THROW(GuidedPlateModel({},Binding,invalid),std::invalid_argument);
}

TEST_F(GuidedPlatePenalty, ActualReferencesAndInitialModeStayIdenticalButC3RateAndTimeAreFresh) {
    const auto& a=original->shell().data(); const auto& b=revised->shell().data();
    SameConfiguration(a.reference_configuration,b.reference_configuration);
    SameConfiguration(original_modal.initial_configuration,revised_modal.initial_configuration);
    EXPECT_EQ(original_modal.initial_mode_increment,revised_modal.initial_mode_increment);
    EXPECT_EQ(original_modal.squared_frequency,revised_modal.squared_frequency);
    EXPECT_EQ(original_modal.initial_elastic_energy,revised_modal.initial_elastic_energy);
    EXPECT_NE(original_modal.qualification_id,revised_modal.qualification_id);
    for (unsigned n=0;n<kCouponNodes;++n) {
        EXPECT_EQ(a.nodal_mass[n].mass,b.nodal_mass[n].mass);
        EXPECT_EQ(a.nodal_mass[n].physical_tangential_inertia,b.nodal_mass[n].physical_tangential_inertia);
        EXPECT_EQ(a.nodal_mass[n].artificial_drilling_inertia,b.nodal_mass[n].artificial_drilling_inertia);
        EXPECT_EQ(a.inverse_mass[n],b.inverse_mass[n]); EXPECT_EQ(a.inverse_isotropic_inertia[n],b.inverse_isotropic_inertia[n]);
    }
    const auto& first=original->contact_stiffness(); const auto& second=revised->contact_stiffness();
    ASSERT_EQ(first.count,second.count);
    for (unsigned i=0;i<first.count;++i) for (unsigned j=0;j<first.count;++j) {
        EXPECT_EQ(4*first.entry[i][j].lower,second.entry[i][j].lower);
        EXPECT_EQ(4*first.entry[i][j].upper,second.entry[i][j].upper);
    }
    EXPECT_EQ(4*first.rate_bound,second.rate_bound);
    EXPECT_EQ(revised_modal.contact_rate_bound,second.rate_bound);
    EXPECT_EQ(original_modal.monitored_structural_norm_limit,revised_modal.monitored_structural_norm_limit);
    EXPECT_LT(revised_modal.proposed_step_limit,original_modal.proposed_step_limit);
    EXPECT_LE(revised_modal.time_step,.1/std::sqrt(revised_modal.combined_rate_envelope));
    EXPECT_GE(revised_modal.step_count,original_modal.step_count);
    RecordProperty("revised_contact_rate",Precise(second.rate_bound));
    RecordProperty("revised_proposed_h",Precise(revised_modal.proposed_step_limit));
    RecordProperty("revised_base_steps",std::to_string(revised_modal.step_count));
}

TEST_F(GuidedPlatePenalty, NonlinearCertifiedCapacityRejectsOriginalDesignTargetAndScreensRevision) {
    GuidedPlatePenaltyReport old,next; std::string error;
    ASSERT_EQ(AuditGuidedPlatePenalty(*original,original_modal,old,error),Status::kSuccess)<<error;
    ASSERT_EQ(AuditGuidedPlatePenalty(*revised,revised_modal,next,error),Status::kSuccess)<<error;
    EXPECT_FALSE(old.enforced); EXPECT_FALSE(old.target_sufficient);
    EXPECT_TRUE(next.enforced); EXPECT_TRUE(next.target_sufficient);
    EXPECT_LT(old.sample[1].total_potential_lower,old.admitted_energy_upper);
    EXPECT_GT(next.sample[0].total_potential_lower,next.admitted_energy_upper);
    for (const auto* report:{&old,&next}) for (unsigned i=0;i<2;++i) {
        const auto& s=report->sample[i];
        EXPECT_LE(s.maximum_depth.upper,s.requested_penetration);
        EXPECT_GE(s.actual_penetration,s.maximum_depth.lower); EXPECT_LE(s.actual_penetration,s.maximum_depth.upper);
        EXPECT_LT(s.requested_penetration-s.maximum_depth.lower,1e-12);
        EXPECT_LE(s.inward_scale_adjustments,64u);
        EXPECT_LE(std::abs(s.shell_energy_tl-s.shell_energy_chrono),s.shell_energy_allowance);
        EXPECT_LE(std::abs(s.strip_potential-s.contact_potential.value),s.strip_energy_allowance+1e-20);
        EXPECT_LE(std::abs(s.strip_resultant-s.contact_resultant.value),s.strip_force_allowance+1e-18);
        EXPECT_LE(s.contact_potential.error,2*kGuidedOriginalEnergyError);
        EXPECT_LE(s.contact_resultant.error,2*kGuidedOriginalForceError);
    }
    RecordProperty("initial_energy_J",Precise(next.initial_energy));
    RecordProperty("admitted_energy_upper_J",Precise(next.admitted_energy_upper));
    RecordProperty("original_cap_lower_J",Precise(old.sample[1].total_potential_lower));
    RecordProperty("revised_target_lower_J",Precise(next.sample[0].total_potential_lower));
    RecordProperty("revised_cap_lower_J",Precise(next.sample[1].total_potential_lower));
    RecordProperty("revised_target_actual_depth_m",Precise(next.sample[0].actual_penetration));
    RecordProperty("revised_cap_actual_depth_m",Precise(next.sample[1].actual_penetration));
}

TEST_F(GuidedPlatePenalty, ActualForcePotentialDifferencesStayInsideContactDomainWithoutSnappingMode) {
    const auto initial=revised_modal.initial_configuration;
    GuidedPlatePenaltyReport report; std::string error;
    ASSERT_EQ(AuditGuidedPlatePenalty(*revised,revised_modal,report,error),Status::kSuccess)<<error;
    for (unsigned sample=0;sample<2;++sample) for (unsigned level=0;level<2;++level) {
        const auto& d=report.sample[sample].derivative[level];
        EXPECT_EQ(d.backward,sample==1); EXPECT_EQ(d.step_m,patch_audit::TranslationDifference/(1u<<level));
        EXPECT_GT(d.derivative_N,0); EXPECT_LT(d.restoring_force_N,0);
        EXPECT_GE(d.contact_uncertainty_N,0); EXPECT_LE(d.error_N,d.allowance_N);
        RecordProperty("derivative_error_"+std::to_string(sample)+"_"+std::to_string(level),Precise(d.error_N));
        RecordProperty("derivative_allowance_"+std::to_string(sample)+"_"+std::to_string(level),Precise(d.allowance_N));
    }
    SameConfiguration(initial,revised_modal.initial_configuration);
    EXPECT_NE(revised_modal.initial_mode_increment[8],revised_modal.initial_mode_increment[12]);
    // Reapply the reported global scale: actual unequal tip coordinates survive.
    ElasticCouponConfiguration sample;
    ASSERT_EQ(ApplyGuidedPlateIncrement(revised->shell().data().reference_configuration,revised_modal.initial_mode_increment,
        report.sample[1].modal_scale,sample,error),Status::kSuccess)<<error;
    EXPECT_NE(sample.position[4].x,sample.position[5].x);
}

TEST_F(GuidedPlatePenalty, ForeignAndLateInvalidModalDataPreserveEntireReportAndPermitCleanRetry) {
    GuidedPlatePenaltyReport report; std::string error;
    ASSERT_EQ(AuditGuidedPlatePenalty(*revised,revised_modal,report,error),Status::kSuccess)<<error;
    const auto saved=report;
    EXPECT_EQ(AuditGuidedPlatePenalty(*revised,original_modal,report,error),Status::kAuditRejected);
    EXPECT_EQ(std::memcmp(&saved,&report,sizeof(report)),0); EXPECT_FALSE(error.empty());
    auto invalid=revised_modal; invalid.initial_mode_increment.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(AuditGuidedPlatePenalty(*revised,invalid,report,error),Status::kAuditRejected);
    EXPECT_EQ(std::memcmp(&saved,&report,sizeof(report)),0);
    invalid=revised_modal; invalid.initial_elastic_energy*=2;
    EXPECT_EQ(AuditGuidedPlatePenalty(*revised,invalid,report,error),Status::kAuditRejected);
    EXPECT_EQ(std::memcmp(&saved,&report,sizeof(report)),0);
    ASSERT_EQ(AuditGuidedPlatePenalty(*revised,revised_modal,report,error),Status::kSuccess)<<error;
    EXPECT_EQ(report.sample[0].contact_potential.value,saved.sample[0].contact_potential.value);
    EXPECT_EQ(report.sample[1].total_potential_lower,saved.sample[1].total_potential_lower);
}
} // namespace
} // namespace crash::reference
