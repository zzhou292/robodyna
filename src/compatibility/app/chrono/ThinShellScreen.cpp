#include "ThinShellScreenInternal.h"

#include <cmath>
#include <new>
#include <stdexcept>

namespace crash::reference {
namespace {
using Status=ElasticCouponStatus;
namespace audit=patch_audit;
namespace detail=thin_shell_detail;

void Measure(const ElasticCouponModel& model,const audit::PatchDifferenceSteps& steps,
             std::array<ThinShellMatrixMeasurement,3>& result) {
    const auto layout=audit::FullCouponLayout();
    for (std::size_t i=0;i<result.size();++i) {
        auto& measurement=result[i];
        measurement.status=audit::DifferenceJacobian<kCouponFreeDofs>(model,model.data().reference_configuration,layout,
            i==2 ? audit::ForceSource::TL : audit::ForceSource::Chrono,steps,i==0 ? 1 : .5,
            measurement.stiffness,measurement.diagnostic);
        measurement.available=measurement.status==Status::kSuccess;
    }
}

void Policy(const ElasticCouponModel& model,const ThinShellFixtureDiagnostic& fixture,
            ThinShellInertiaPolicy kind,ThinShellPolicyDiagnostic& result) {
    result.policy=kind;
    result.mass=kind==ThinShellInertiaPolicy::PhysicalThickness ? audit::DefaultPatchNodalMass(model.data()) :
                                                               fixture.inertia.counterfactual;
    const auto layout=audit::FullCouponLayout();
    result.inverse_root_mass=audit::InverseRootMass<kCouponFreeDofs>(result.mass,layout);
    for (std::size_t level=0;level<2;++level) if (fixture.measurement[level].available) {
        auto& spectrum=result.spectrum[level];
        std::string diagnostic;
        if (!detail::InspectSpectrum(fixture.measurement[level].stiffness,result.inverse_root_mass,spectrum,diagnostic))
            spectrum.diagnostic=diagnostic;
        detail::DescribeModeEnergies(model,fixture.inertia,kind,result.inverse_root_mass,level==1,spectrum);
    }
    // This call deliberately follows the retained, independently inspectable
    // raw measurements. Its existing positivity/uncertainty gates are intact.
    audit::ReferenceSpectrum<kCouponFreeDofs> admitted;
    result.reference_attempted=true;
    result.reference_status=audit::AuditReference<kCouponFreeDofs>(model,layout,result.mass,fixture.difference_steps,
                                                                admitted,result.reference_diagnostic);
    result.reference_passed=result.reference_status==Status::kSuccess;
    if (result.reference_passed) {
        result.reference_symmetry_error=admitted.symmetry_error;
        result.derivative_refinement_error=admitted.derivative_refinement_error;
        result.tl_derivative_error=admitted.tl_derivative_error;
        result.maximum_frequency_refinement_error=admitted.maximum_frequency_refinement_error;
        result.reference_eigen_residual=admitted.eigen_residual;
    }
    if (fixture.measurement[1].available) {
        result.directional_attempted=true;
        result.directional_status=audit::DirectionalCrossCheck<kCouponFreeDofs>(model,model.data().reference_configuration,
            layout,fixture.measurement[1].stiffness,result.inverse_root_mass,fixture.difference_steps,
            result.directional_error,result.directional_diagnostic);
        detail::EstimateSteps(model.parameters(),fixture.edge_m,kind,fixture.measurement[1].stiffness,
                              result.inverse_root_mass,result.step);
    }
}
} // namespace

ElasticCouponParameters ThinShellFixtureParameters(std::size_t index) {
    if (index>=kThinShellFixtureCount) throw std::out_of_range("Thin-shell fixture index must be in [0,5]");
    constexpr double edges[]{.01,.02},multipliers[]{.5,1,2};
    ElasticCouponParameters result;
    result.length=2*edges[index/3]; result.width=edges[index/3];
    result.thickness=multipliers[index%3]*kThinShellSourceThickness;
    result.young_modulus=200e9; result.density=7890; result.poisson_ratio=.3;
    result.shear_factor=5.0/6.0; result.torque_factor=.01;
    return result;
}

ThinShellFixtureDiagnostic ScreenThinShellFixture(std::size_t index) {
    ThinShellFixtureDiagnostic result;
    result.fixture_index=index; result.parameters=ThinShellFixtureParameters(index);
    result.edge_m=result.parameters.width;
    constexpr double multipliers[]{.5,1,2}; result.thickness_multiplier=multipliers[index%3];
    result.difference_steps={1e-6*result.edge_m,1e-6};
    result.policies[1].policy=ThinShellInertiaPolicy::AreaCounterfactual;
    try {
        ElasticCouponModel model(result.parameters);
        result.reference_configuration=model.data().reference_configuration;
        result.connectivity=model.data().connectivity;
        const auto status=AssembleShellPatchInertia(model.data().element_mass,model.data().connectivity,
                                                  result.inertia,result.diagnostic);
        if (status!=Status::kSuccess) return result;
        result.setup_available=true;
        Measure(model,result.difference_steps,result.measurement);
        Policy(model,result,ThinShellInertiaPolicy::PhysicalThickness,result.policies[0]);
        Policy(model,result,ThinShellInertiaPolicy::AreaCounterfactual,result.policies[1]);
        detail::CompleteScreen(result);
    } catch (const std::bad_alloc&) { throw; }
      catch (const std::exception& error) { result.diagnostic=error.what(); }
    return result;
}

ThinShellScreenDiagnostic ScreenThinShellFixtures() {
    ThinShellScreenDiagnostic result;
    result.screen_passed=true;
    for (std::size_t i=0;i<kThinShellFixtureCount;++i) {
        result.fixtures[i]=ScreenThinShellFixture(i);
        result.screen_passed=result.screen_passed&&result.fixtures[i].screen_passed;
    }
    return result;
}

namespace thin_shell_detail {
void CompleteScreen(ThinShellFixtureDiagnostic& result) {
    result.screen_passed=false; result.simulation_ready=false;
    if (!result.setup_available) { result.diagnostic="Reference setup is unavailable"; return; }
    auto& physical=result.policies[0]; auto& changed=result.policies[1];
    const auto& physical_fine=physical.spectrum[1];
    const auto& physical_coarse=physical.spectrum[0];
    // Never skip a lower nonbending baseline cluster, at either FD level.
    if (!physical_fine.available||!physical_coarse.available||!physical_fine.cluster_count||
        !physical_coarse.cluster_count||!physical_fine.clusters[0].bending_eligible||
        !physical_coarse.clusters[0].bending_eligible) {
        result.diagnostic="Lowest physical baseline cluster is unresolved or fails bending identity"; return;
    }
    MatchCluster(physical_fine,physical.inverse_root_mass,0,physical_coarse,physical.inverse_root_mass,
                 physical.mass,true,physical.coarse_fine_match);
    if (physical.coarse_fine_match.matched&&physical.coarse_fine_match.comparison.clusters[0].candidate_cluster!=0) {
        physical.coarse_fine_match.matched=false;
        physical.coarse_fine_match.status=ShellModeComparisonStatus::kUnmatched;
        physical.coarse_fine_match.diagnostic="Fine lowest branch does not match the coarse lowest baseline cluster";
    }
    MatchCluster(physical_fine,physical.inverse_root_mass,0,changed.spectrum[1],changed.inverse_root_mass,
                 physical.mass,false,result.hybrid_match);
    if (result.hybrid_match.matched) {
        const auto selected=result.hybrid_match.comparison.clusters[0].candidate_cluster;
        MatchCluster(changed.spectrum[1],changed.inverse_root_mass,selected,changed.spectrum[0],changed.inverse_root_mass,
                     physical.mass,true,changed.coarse_fine_match);
    }
    for (const auto& raw:result.measurement) if (!raw.available) {
        result.diagnostic="At least one raw force-difference measurement failed"; return;
    }
    if (!result.hybrid_match.matched) {
        result.diagnostic="Area-inertia hybrid failed unique bending identity or the 5% frequency screen"; return;
    }
    for (const auto& policy:result.policies) {
        if (!policy.reference_passed||policy.directional_status!=Status::kSuccess) {
            result.diagnostic="Existing reference or directional audit rejected a measured policy"; return;
        }
        if (!policy.coarse_fine_match.matched||!policy.coarse_fine_match.frequency_refined) {
            result.diagnostic="Tracked coarse/fine cluster identity or 0.5% frequency refinement failed"; return;
        }
        for (const auto& spectrum:policy.spectrum) for (const auto& mode:spectrum.modes)
            if (!mode.kinetic_available) {
                result.diagnostic="A modal physical/artificial kinetic partition is unresolved"; return;
            }
    }
    result.screen_passed=true; result.diagnostic.clear();
}
} // namespace thin_shell_detail
} // namespace crash::reference
