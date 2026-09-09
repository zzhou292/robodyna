#include "GuidedPlateModal.h"
#include "ElasticCouponModal.h"
#include "ReissnerShellHostFixture.h"
#include "lib_utest/q4_planar_geometry_fixture.h"
#include "math/Quaternion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace crash::qualification {
namespace {
using namespace reference;
namespace tlr=tl::fea::reissner;
namespace sc=tlfea::contact;
namespace fixture=q4_planar_test;
using Status=ElasticCouponStatus;
constexpr std::uint64_t WallBinding=0x443157414c4cULL;

tlr::Vec3 Cyclic(tlr::Vec3 value) { return {value.z,value.x,value.y}; }
void SameConfiguration(const ElasticCouponConfiguration& a,const ElasticCouponConfiguration& b) {
    for (std::size_t n=0;n<kCouponNodes;++n) {
        EXPECT_EQ(a.position[n].x,b.position[n].x); EXPECT_EQ(a.position[n].y,b.position[n].y); EXPECT_EQ(a.position[n].z,b.position[n].z);
        EXPECT_EQ(a.rotation[n].w,b.rotation[n].w); EXPECT_EQ(a.rotation[n].x,b.rotation[n].x);
        EXPECT_EQ(a.rotation[n].y,b.rotation[n].y); EXPECT_EQ(a.rotation[n].z,b.rotation[n].z);
    }
}
void NearVector(tlr::Vec3 a,tlr::Vec3 b,double tolerance=1e-9) {
    EXPECT_NEAR(a.x,b.x,tolerance); EXPECT_NEAR(a.y,b.y,tolerance); EXPECT_NEAR(a.z,b.z,tolerance);
}
std::string PreciseGuided(double value) {
    std::ostringstream text; text.precision(17); text<<value; return text.str();
}

TEST(GuidedPlateReference, DefaultPoseAndExactCyclicPosePreserveReferenceForceContract) {
    ElasticCouponModel original,identity(ElasticCouponPose{});
    SameConfiguration(original.data().reference_configuration,identity.data().reference_configuration);
    const auto wall=fixture::Square(); GuidedPlateModel guided(wall.view(),WallBinding);
    const auto& data=guided.shell().data();
    for (std::size_t n=0;n<kCouponNodes;++n) {
        NearVector(data.reference_configuration.position[n],
                   tlr::detail::Add(Cyclic(original.data().reference_configuration.position[n]),guided.data().pose.translation),0);
        EXPECT_EQ(data.reference_configuration.rotation[n].w,.5); EXPECT_EQ(data.reference_configuration.rotation[n].x,.5);
        EXPECT_EQ(data.reference_configuration.rotation[n].y,.5); EXPECT_EQ(data.reference_configuration.rotation[n].z,.5);
    }
    std::array<double,kCouponFreeDofs> delta{};
    for (std::size_t i=0;i<kCouponFreeNodes.size();++i) {
        delta[6*i]=1e-5*(i+1); delta[6*i+2]=2e-4*(i+1);
        delta[6*i+3]=.001; delta[6*i+4]=-.003*(i+1); delta[6*i+5]=.0002;
    }
    ElasticCouponConfiguration changed,posed;
    std::string diagnostic;
    ASSERT_EQ(ApplyElasticCouponIncrement(original.data().reference_configuration,delta,1,changed,diagnostic),Status::kSuccess);
    posed=changed;
    for (std::size_t n=0;n<kCouponNodes;++n) {
        posed.position[n]=tlr::detail::Add(Cyclic(changed.position[n]),guided.data().pose.translation);
        posed.rotation[n]=tl::math::Product(guided.data().pose.rotation,changed.rotation[n]);
    }
    ElasticCouponEvaluation a,b,c,default_copy;
    ASSERT_EQ(original.EvaluateTL(changed,a,diagnostic),Status::kSuccess) << diagnostic;
    ASSERT_EQ(identity.EvaluateTL(changed,default_copy,diagnostic),Status::kSuccess) << diagnostic;
    ASSERT_EQ(guided.shell().EvaluateTL(posed,b,diagnostic),Status::kSuccess) << diagnostic;
    ASSERT_EQ(guided.shell().EvaluateChrono(posed,c,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_EQ(a.energy,default_copy.energy); EXPECT_NEAR(a.energy,b.energy,1e-11);
    for (std::size_t n=0;n<kCouponNodes;++n) {
        NearVector(b.force[n],Cyclic(a.force[n])); NearVector(b.couple[n],Cyclic(a.couple[n]));
    }
    for (std::size_t e=0;e<kCouponElements;++e) {
        ExpectShellAgreement(b.element[e],ToEvaluation(c.element[e]));
        ExpectShellAgreement(default_copy.element[e],ToEvaluation(a.element[e]));
    }
}

TEST(GuidedPlateReference, PhysicalAndArtificialInertiaEnergiesAreCovariantAndCountSharedNodesOnce) {
    ElasticCouponModel original; const auto wall=fixture::Square(); GuidedPlateModel guided(wall.view(),WallBinding);
    const auto& source=original.data(); const auto& posed=guided.shell().data();
    double mass=0,physical=0,artificial=0;
    for (std::size_t n=0;n<kCouponNodes;++n) {
        EXPECT_NEAR(posed.nodal_mass[n].mass,source.nodal_mass[n].mass,1e-15);
        EXPECT_NEAR(posed.nodal_mass[n].physical_tangential_inertia,source.nodal_mass[n].physical_tangential_inertia,1e-18);
        EXPECT_EQ(posed.nodal_mass[n].physical_tangential_inertia,posed.nodal_mass[n].artificial_drilling_inertia);
        mass+=posed.nodal_mass[n].mass; physical+=posed.nodal_mass[n].physical_tangential_inertia;
        artificial+=posed.nodal_mass[n].artificial_drilling_inertia;
    }
    EXPECT_NEAR(mass,.4,1e-15); EXPECT_NEAR(physical,.4*.02*.02/12,1e-18); EXPECT_EQ(physical,artificial);
    for (std::size_t e=0;e<kCouponElements;++e) {
        tlr::Vec3 d[4],v[4],w[4],dp[4],vp[4],wp[4];
        for (unsigned n=0;n<4;++n) {
            const auto global=source.connectivity[e][n];
            d[n]={0,0,1}; v[n]={.2*(global+1),-.1,.3}; w[n]={-.4,.2*(global+1),.1};
            const auto q=tlr::detail::Product(posed.reference[e].initial_rotation[n],posed.reference[e].node_frame_offset[n]);
            dp[n]=tlr::detail::Product(tlr::detail::Rotation(q),tlr::Vec3{0,0,1});
            NearVector(dp[n],{1,0,0},1e-14); vp[n]=Cyclic(v[n]); wp[n]=Cyclic(w[n]);
        }
        tlr::ShellKineticEnergy before,after;
        ASSERT_EQ(tlr::ComputeShellKineticEnergy(source.element_mass[e],d,v,w,before),tlr::ShellMassStatus::kSuccess);
        ASSERT_EQ(tlr::ComputeShellKineticEnergy(posed.element_mass[e],dp,vp,wp,after),tlr::ShellMassStatus::kSuccess);
        EXPECT_NEAR(after.translation,before.translation,1e-15);
        EXPECT_NEAR(after.physical_rotation,before.physical_rotation,1e-18);
        EXPECT_NEAR(after.artificial_drilling,before.artificial_drilling,1e-18);
    }
}

TEST(GuidedPlateReference, ActualFiniteWallCoverageAndNaturalOrderingAreRequiredWithoutRepositioning) {
    auto wall=fixture::Square(2); GuidedPlateModel model(wall.view(),WallBinding);
    const auto& data=model.shell().data(); const auto reference=model.contact_geometry().view();
    EXPECT_EQ(model.data().wall_binding_id,WallBinding); EXPECT_EQ(reference.parent_count,2);
    for (std::size_t e=0;e<kCouponElements;++e) {
        EXPECT_TRUE(reference.parents[e].covered);
        const auto* p=reference.parents[e].reference_projection;
        EXPECT_GT(p[0].y,p[1].y); EXPECT_GT(p[0].z,p[3].z);
        EXPECT_EQ(p[0].y,p[3].y); EXPECT_EQ(p[0].z,p[1].z);
        EXPECT_NEAR(reference.parents[e].projected_area,.01,1e-17);
    }
    for (std::size_t n=0;n<kCouponNodes;++n) {
        EXPECT_EQ(model.data().translation_fixed_bits[n],data.fixed[n] ? 7 : 6);
        EXPECT_EQ(model.data().rotation_fixed[n],data.fixed[n] ? 1 : 0);
        EXPECT_EQ(data.reference_configuration.position[n].x,-.001);
    }
    const double stored=model.wall().faces()[0].geometry.vertices[0].y;
    wall.vertices[0].position.y=100;
    EXPECT_EQ(model.wall().faces()[0].geometry.vertices[0].y,stored);
    const auto hole=fixture::Ring();
    EXPECT_THROW(GuidedPlateModel(hole.view(),WallBinding),std::invalid_argument);
    const auto too_small=fixture::GridWall({-.09,.09});
    EXPECT_THROW(GuidedPlateModel(too_small.view(),WallBinding),std::invalid_argument);
    const auto valid=fixture::Square();
    EXPECT_THROW(GuidedPlateModel(valid.view(),0),std::invalid_argument);
}

TEST(GuidedPlateReference, SixteenCoordinateIncrementUsesWorldSpinAndStagesLateFailure) {
    const auto wall=fixture::Square(); GuidedPlateModel model(wall.view(),WallBinding);
    const auto base=model.shell().data().reference_configuration;
    const auto layout=GuidedPlateLayout(); ASSERT_EQ(layout.size(),16);
    std::array<double,kGuidedPlateDofs> increment{};
    for (std::size_t free=0;free<kCouponFreeNodes.size();++free) {
        increment[4*free]=-.001*(free+1); increment[4*free+1]=.01; increment[4*free+2]=-.02; increment[4*free+3]=.03;
    }
    ElasticCouponConfiguration output; std::string diagnostic;
    ASSERT_EQ(ApplyGuidedPlateIncrement(base,increment,1,output,diagnostic),Status::kSuccess);
    chrono::ChQuaterniond spin; spin.SetFromRotVec(chrono::ChVector3d(.01,-.02,.03));
    const auto expected=spin*chrono::ChQuaterniond(.5,.5,.5,.5);
    for (std::size_t n=0;n<kCouponNodes;++n) {
        EXPECT_EQ(output.position[n].y,base.position[n].y); EXPECT_EQ(output.position[n].z,base.position[n].z);
        if (model.shell().data().fixed[n]) {
            EXPECT_EQ(output.position[n].x,base.position[n].x); EXPECT_EQ(output.rotation[n].w,.5);
        } else {
            EXPECT_NEAR(output.rotation[n].w,expected.e0(),1e-15); EXPECT_NEAR(output.rotation[n].x,expected.e1(),1e-15);
            EXPECT_NEAR(output.rotation[n].y,expected.e2(),1e-15); EXPECT_NEAR(output.rotation[n].z,expected.e3(),1e-15);
        }
    }
    const auto saved=output;
    increment.back()=std::numeric_limits<double>::infinity();
    EXPECT_EQ(ApplyGuidedPlateIncrement(base,increment,1,output,diagnostic),Status::kInvalidConfiguration);
    SameConfiguration(output,saved);
    auto invalid=layout; invalid.back()=invalid.front(); increment.back()=0;
    EXPECT_EQ(patch_audit::ApplyIncrement(base,invalid.data(),invalid.size(),increment.data(),1,output,diagnostic),Status::kInvalidConfiguration);
    SameConfiguration(output,saved);
    patch_audit::ReferenceSpectrum<kGuidedPlateDofs> spectrum; spectrum.symmetry_error=123;
    EXPECT_EQ(patch_audit::AuditReference<kGuidedPlateDofs>(model.shell(),invalid,spectrum,diagnostic),Status::kInvalidConfiguration);
    EXPECT_EQ(spectrum.symmetry_error,123);
}

TEST(GuidedPlateReference, GuidedSpectrumAndPrescribedContactSweepAdmitOnlyTheDeclaredEnvelope) {
    const auto wall=fixture::Square(); GuidedPlateModel model(wall.view(),WallBinding);
    GuidedPlateModalReport report; std::string diagnostic;
    ASSERT_EQ(AuditGuidedPlate(model,report,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_EQ(report.selected_mode,0); EXPECT_GT(report.squared_frequency.front(),0);
    EXPECT_TRUE(std::is_sorted(report.squared_frequency.begin(),report.squared_frequency.end()));
    EXPECT_GE(report.bending_mass_fraction,.9); EXPECT_GE(report.normal_translation_mass_fraction,.5);
    EXPECT_LE(report.reference_symmetry_error,patch_audit::DerivativeTolerance);
    EXPECT_LE(report.mass_scaled_derivative_refinement_error,patch_audit::DerivativeTolerance);
    EXPECT_LE(report.tl_derivative_relative_error,patch_audit::DerivativeTolerance);
    EXPECT_LE(report.tl_directional_relative_error,patch_audit::DerivativeTolerance);
    EXPECT_LE(report.maximum_frequency_refinement_error,patch_audit::FrequencyRefinementTolerance);
    EXPECT_LE(report.eigen_residual,1e-10);
    const auto& reference=model.shell().data().reference_configuration;
    EXPECT_NEAR(.5*(report.initial_configuration.position[4].x+report.initial_configuration.position[5].x)-reference.position[4].x,-.002,1e-15);
    EXPECT_NEAR(report.initial_configuration.position[4].x,report.initial_configuration.position[5].x,1e-10);
    EXPECT_GT(report.initial_elastic_energy,0); EXPECT_EQ(report.monitored_structural_norm_limit,2*report.sampled_structural_norm_maximum);
    EXPECT_GE(report.combined_rate_envelope,report.monitored_structural_norm_limit+report.contact_rate_bound);
    EXPECT_GT(report.contact_rate_bound,0); EXPECT_LE(report.time_step,.1/std::sqrt(report.combined_rate_envelope));
    EXPECT_LE(report.time_step,report.wave_step_limit); EXPECT_LE(report.time_step,report.rotary_step_limit);
    EXPECT_EQ(report.horizon,report.time_step*report.step_count); EXPECT_NEAR(report.horizon,.2,1e-16);
    EXPECT_GT(report.step_count,100); EXPECT_LE(report.step_count,1000000);
    double current_norm=0;
    ASSERT_EQ(MeasureGuidedPlateStructuralNorm(model,report.initial_configuration,current_norm,diagnostic),Status::kSuccess);
    EXPECT_EQ(current_norm,report.sampled_structural_operator_norm[4]);
    fixture::Scratch scratch;
    bool nonuniform_contact=false;
    for (double scale:{1.,0.,-.5,-.7}) {
        ElasticCouponConfiguration configuration;
        ASSERT_EQ(ApplyGuidedPlateIncrement(reference,report.initial_mode_increment,scale,configuration,diagnostic),Status::kSuccess);
        std::array<double,3*kCouponNodes> x{},v{};
        for (std::size_t n=0;n<kCouponNodes;++n) {
            x[3*n]=configuration.position[n].x; x[3*n+1]=configuration.position[n].y; x[3*n+2]=configuration.position[n].z;
        }
        const sc::Q4SurfaceView surface{{x.data(),kCouponNodes,3,1},{v.data(),kCouponNodes,3,1},model.data().parents.data(),kCouponElements};
        const sc::Q4FixedYZMassView mass{model.shell().data().inverse_mass.data(),model.data().translation_fixed_bits.data(),kCouponNodes,0};
        double potential=0,force=0;
        for (unsigned p=0;p<kCouponElements;++p) {
            sc::Q4PreparedIntegration input; sc::Q4IntegrationResult result;
            ASSERT_EQ(sc::PrepareQ4PlanarIntegration(model.contact_geometry().view(),surface,mass,p,model.data().stiffness_per_area,
                      model.data().maximum_penetration,1,&input),sc::PlanarContactStatus::Ok);
            ASSERT_TRUE(input.covered);
            const auto integral=sc::IntegrateQ4NormalContact(input.input,model.data().integration,scratch.view(),&result);
            ASSERT_EQ(integral.status,sc::Q4IntegrationStatus::Ok) << "scale="<<scale<<", parent="<<p;
            potential+=result.potential.value; force+=result.resultant.value;
            if (result.resultant.value>0)
                nonuniform_contact|=std::abs(result.force[0].value-result.force[1].value)>result.force[0].error+result.force[1].error;
        }
        if (scale>=0) { EXPECT_EQ(potential,0); EXPECT_EQ(force,0); }
        if (scale==-.7) { EXPECT_GT(potential,0); EXPECT_GT(force,0); }
    }
    EXPECT_TRUE(nonuniform_contact);
    RecordProperty("first_mode_angular_frequency",PreciseGuided(report.first_mode_angular_frequency));
    RecordProperty("structural_norm_limit",PreciseGuided(report.monitored_structural_norm_limit));
    RecordProperty("contact_rate_bound",PreciseGuided(report.contact_rate_bound));
    RecordProperty("combined_rate_envelope",PreciseGuided(report.combined_rate_envelope));
    RecordProperty("time_step",PreciseGuided(report.time_step)); RecordProperty("step_count",std::to_string(report.step_count));
    RecordProperty("initial_elastic_energy",PreciseGuided(report.initial_elastic_energy));
}

TEST(GuidedPlateReference, InvalidPoseAndNormOutputRejectWithoutChangingPublishedValues) {
    ElasticCouponPose pose; pose.rotation={0,0,0,0};
    EXPECT_THROW(ElasticCouponModel{pose},std::invalid_argument);
    pose={}; pose.translation.y=std::numeric_limits<double>::infinity();
    EXPECT_THROW(ElasticCouponModel{pose},std::invalid_argument);
    const auto wall=fixture::Square(); GuidedPlateModel model(wall.view(),WallBinding);
    auto invalid=model.shell().data().reference_configuration; invalid.rotation[5]={0,0,0,0};
    double output=123; std::string diagnostic;
    EXPECT_EQ(MeasureGuidedPlateStructuralNorm(model,invalid,output,diagnostic),Status::kInvalidConfiguration);
    EXPECT_EQ(output,123); EXPECT_FALSE(diagnostic.empty());
}
}  // namespace
}  // namespace crash::qualification
