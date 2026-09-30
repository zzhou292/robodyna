#include "GuidedPlateAdmission.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
namespace app=crash::case_data;
namespace ref=crash::reference;
namespace shell=tl::fea::reissner;
namespace contact=tlfea::contact;
struct Fixture {
    shell::ShellBatchDiagnostics shell;
    contact::Q4PlanarContactDiagnostics contact;
    ref::GuidedPlateModalReport modal;
    static constexpr double initial_energy=.001;
    Fixture() {
        shell.owner_id=contact.owner_id=9; shell.base_epoch=contact.base_epoch=3;
        shell.attempt=contact.attempt=7;
        shell.configuration_id=contact.configuration_id=modal.qualification_id=ref::kGuidedOriginalExperiment.qualification_id;
        contact.wall_binding_id=13; shell.valid=contact.valid=true;
        shell.phase=shell::ShellBatchPhase::kPreparedCandidate;
        contact.phase=contact::Q4PlanarContactPhase::PreparedCandidate;
        shell.elastic_energy=.0007; shell.kinetic_translation=.0002;
        shell.minimum_signed_area_ratio=shell.minimum_area_norm_ratio=shell.maximum_area_norm_ratio=1;
        shell.minimum_display_triangle_area_ratio=1;
        contact.parent_count=contact.covered_count=2;
        contact.potential.value=contact.potential.lower=contact.potential.upper=.0001;
        contact.active_area={.003,.003}; contact.maximum_penetration=.0002;
        contact.stiffness_rate_bound=modal.contact_rate_bound=1000;
        modal.monitored_structural_norm_limit=2000; modal.combined_rate_envelope=3000;
        shell.base_elastic_energy=.000699; shell.base_kinetic_energy=.000197;
        shell.elastic_energy_increment=1e-6; shell.kinetic_energy_increment=3e-6;
        shell.kinetic_midpoint_work=1e-6; shell.kinetic_work_residual=2e-6;
        shell.force_coordinate_work=-.999e-6; shell.conservative_force_coordinate_defect=1e-9;
        shell.mass_weighted_increment_squared=1e-11;
        contact.base_potential=.000104; contact.potential_increment=-4e-6;
        contact.kinetic_midpoint_work=2e-6; contact.force_coordinate_work=4.004e-6;
        contact.conservative_force_coordinate_defect=4e-9; contact.quadratic_work_upper=5e-9;
        contact.continuum_work_uncertainty=1e-13; contact.kinetic_midpoint_roundoff=1e-20;
    }
    bool Check(app::GuidedPlateWorkReport& out,std::string& why,bool candidate=true) const {
        return app::CheckGuidedPlateEnvelope(shell,contact,modal,initial_energy,.004,candidate,out,why);
    }
};

TEST(GuidedPlateAdmission, ContactWorkCompletesTheDiscreteKineticIdentity) {
    Fixture f; app::GuidedPlateWorkReport out; std::string why;
    ASSERT_TRUE(f.Check(out,why))<<why;
    EXPECT_GT(std::abs(f.shell.kinetic_work_residual),out.kinetic_arithmetic_budget);
    EXPECT_LE(std::abs(out.combined_kinetic_residual),out.kinetic_arithmetic_budget);
    EXPECT_DOUBLE_EQ(out.total_energy,Fixture::initial_energy);
    EXPECT_EQ(out.contact_defect_lower_limit,-f.contact.continuum_work_uncertainty);
    EXPECT_TRUE(why.empty());
}

TEST(GuidedPlateAdmission, ContinuumUncertaintyCannotHideAnOmittedKineticContribution) {
    Fixture f; app::GuidedPlateWorkReport out; out.total_energy=17; const auto before=out;
    std::string why; f.contact.kinetic_midpoint_work=0; f.contact.continuum_work_uncertainty=1.;
    EXPECT_FALSE(f.Check(out,why)); EXPECT_NE(why.find("kinetic identity"),std::string::npos);
    EXPECT_EQ(std::memcmp(&out,&before,sizeof(out)),0);
    f.contact.kinetic_midpoint_work=2e-6; f.contact.continuum_work_uncertainty=1e-13;
    EXPECT_TRUE(f.Check(out,why))<<why;
}

TEST(GuidedPlateAdmission, ContactConvexityRequiresBothBoundsAndPreservesOutputOnFailure) {
    for(double defect:{-1e-8,2e-8}) {
        Fixture f; f.contact.conservative_force_coordinate_defect=defect;
        app::GuidedPlateWorkReport out; out.contact_defect_upper_limit=19; const auto before=out;
        std::string why; EXPECT_FALSE(f.Check(out,why));
        EXPECT_NE(why.find("convexity"),std::string::npos);
        EXPECT_EQ(std::memcmp(&out,&before,sizeof(out)),0);
    }
}

TEST(GuidedPlateAdmission, EnergyIncludesContactAndShellGeometryUsesTheSharedLimits) {
    Fixture f; app::GuidedPlateWorkReport out; std::string why;
    f.contact.potential.value=f.contact.potential.lower=f.contact.potential.upper=.0002;
    EXPECT_FALSE(f.Check(out,why)); EXPECT_NE(why.find("total energy"),std::string::npos);
    f=Fixture{}; f.shell.maximum_displacement=.00401;
    EXPECT_FALSE(f.Check(out,why)); EXPECT_NE(why.find("displacement"),std::string::npos);
    f=Fixture{}; f.shell.maximum_pair_angle=.25;
    EXPECT_FALSE(f.Check(out,why)); EXPECT_NE(why.find("director"),std::string::npos);
    f=Fixture{}; f.shell.conservative_force_coordinate_defect=1e-4;
    EXPECT_FALSE(f.Check(out,why)); EXPECT_NE(why.find("shell force-potential"),std::string::npos);
}

TEST(GuidedPlateAdmission, StaleIdentitiesNonfiniteBoundsAndFalseCertificatesReject) {
    for(unsigned fault=0;fault<7;++fault) {
        Fixture f; app::GuidedPlateWorkReport out; std::string why;
        if(fault==0) ++f.contact.attempt;
        if(fault==1) ++f.contact.configuration_id;
        if(fault==2) f.contact.wall_binding_id=0;
        if(fault==3) f.contact.potential.upper+=1e-8; // Error no longer encloses truth.
        if(fault==4) f.modal.combined_rate_envelope=2999;
        if(fault==5) f.contact.force_error.x=-1;
        if(fault==6) f.contact.quadratic_work_upper=std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(f.Check(out,why)); EXPECT_FALSE(why.empty());
    }
}

TEST(GuidedPlateAdmission, InitialBaseRequiresZeroContactUnderTheDeclaredGap) {
    Fixture f; app::GuidedPlateWorkReport out; std::string why;
    f.shell.phase=shell::ShellBatchPhase::kAcceptedBase;
    f.contact.phase=contact::Q4PlanarContactPhase::AcceptedBase;
    EXPECT_FALSE(f.Check(out,why,false));
    f.contact.potential={}; f.contact.active_area={}; f.contact.maximum_penetration=0;
    f.shell.elastic_energy+=.0001;
    EXPECT_TRUE(f.Check(out,why,false))<<why;
}

TEST(GuidedPlateAdmission, CombinedRateMustEncloseTheExactSumRatherThanItsRoundedDownValue) {
    Fixture f; app::GuidedPlateWorkReport out; std::string why;
    f.modal.monitored_structural_norm_limit=1;
    f.modal.contact_rate_bound=f.contact.stiffness_rate_bound=std::ldexp(1.,-53);
    f.modal.combined_rate_envelope=1;
    f.shell.conservative_force_coordinate_defect=0;
    ASSERT_EQ(f.modal.monitored_structural_norm_limit+f.modal.contact_rate_bound,1);
    EXPECT_FALSE(f.Check(out,why)); EXPECT_NE(why.find("rate envelope"),std::string::npos);
    f.modal.combined_rate_envelope=std::nextafter(1.,2.);
    EXPECT_TRUE(f.Check(out,why))<<why;
}
TEST(GuidedPlateAdmission, CrossExperimentModalCannotAuthorizeMatchingForeignContributors) {
    Fixture f;app::GuidedPlateWorkReport out;out.total_energy=17;const auto before=out;std::string error;
    f.modal.experiment=ref::GuidedPlateExperiment::PenaltyMarginV1;
    f.modal.qualification_id=ref::kGuidedPenaltyMarginV1Experiment.qualification_id;
    EXPECT_FALSE(f.Check(out,error));EXPECT_EQ(std::memcmp(&out,&before,sizeof(out)),0);
    // Same numerical rate envelope is insufficient without the named identity.
    f.shell.configuration_id=f.contact.configuration_id=f.modal.qualification_id;
    EXPECT_TRUE(f.Check(out,error))<<error;
    f.modal.experiment=ref::GuidedPlateExperiment::Original;
    EXPECT_FALSE(f.Check(out,error));
}
}  // namespace
