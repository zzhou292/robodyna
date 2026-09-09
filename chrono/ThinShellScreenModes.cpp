#include "ThinShellScreenInternal.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace crash::reference::thin_shell_detail {
namespace {
namespace tlr=tl::fea::reissner;
using Status=ElasticCouponStatus;
using Nodes=std::array<tlr::Vec3,kCouponNodes>;

Nodes Directors(const ElasticCouponData& data) {
    Nodes directors{};
    for (std::size_t e=0;e<kCouponElements;++e) for (std::size_t i=0;i<4;++i) {
        const auto n=data.connectivity[e][i];
        const auto rotation=tlr::detail::Product(tlr::detail::Rotation(data.reference_configuration.rotation[n]),
                                               tlr::detail::Rotation(data.reference[e].node_frame_offset[i]));
        directors[n]=tlr::detail::Product(rotation,tlr::Vec3{0,0,1});
    }
    return directors;
}

void FiniteAmplitude(const ElasticCouponModel& model,const ThinShellVector& mode,ThinShellModeDiagnostic& output) {
    const auto& p=model.parameters();
    const double edge=std::min(p.length/2,p.width);
    double norm=0;
    for (std::size_t n=0;n<kCouponFreeNodes.size();++n) {
        norm=std::max(norm,mode.template segment<3>(6*n).norm());
        norm=std::max(norm,edge*mode.template segment<3>(6*n+3).norm());
    }
    if (!std::isfinite(norm)||norm<=0) {
        output.finite_amplitude_diagnostic="Unrepresentable generalized modal amplitude"; return;
    }
    std::array<double,kCouponFreeDofs> increment{};
    for (std::size_t c=0;c<kCouponFreeDofs;++c) increment[c]=mode(c)/norm;
    const double amplitude=kThinShellShearAmplitudeFraction*std::min(edge,p.thickness);
    for (std::size_t level=0;level<2;++level) {
        output.amplitude_m[level]=level==0 ? amplitude : .5*amplitude;
        ElasticCouponConfiguration configuration;
        auto status=ApplyElasticCouponIncrement(model.data().reference_configuration,increment,
                                                output.amplitude_m[level],configuration,output.finite_amplitude_diagnostic);
        if (status!=Status::kSuccess) { output.finite_amplitude_status=status; return; }
        ElasticCouponEvaluation force;
        status=model.EvaluateTL(configuration,force,output.finite_amplitude_diagnostic);
        if (status!=Status::kSuccess) { output.finite_amplitude_status=status; return; }
        double shear=0;
        // Owning TL resultants/strains and actual copied Chrono quadrature.
        // This centered isotropic section has uncoupled transverse shear rows
        // eps1_z/eps2_z (2/5). No extra constitutive evaluation is introduced.
        for (std::size_t e=0;e<kCouponElements;++e) for (std::size_t point=0;point<4;++point)
            for (std::size_t row:{2u,5u})
                shear+=.5*model.data().reference[e].gauss[point].area_weight*
                    force.element[e].strain[point][row]*force.element[e].resultant[point][row];
        if (!std::isfinite(shear)||shear<0||!std::isfinite(force.energy)||force.energy<=0) {
            output.finite_amplitude_status=Status::kNonfiniteResult;
            output.finite_amplitude_diagnostic="Finite-amplitude shear/total energy is not positive and finite"; return;
        }
        output.total_energy[level]=force.energy; output.bending_energy[level]=force.bending_energy;
        output.shear_energy[level]=shear; output.shear_fraction[level]=shear/force.energy;
        if (!std::isfinite(output.shear_fraction[level])) {
            output.finite_amplitude_status=Status::kNonfiniteResult;
            output.finite_amplitude_diagnostic="Finite-amplitude shear fraction is unrepresentable"; return;
        }
    }
    output.finite_amplitude_status=Status::kSuccess; output.finite_amplitude_available=true;
    output.finite_amplitude_diagnostic.clear();
}
} // namespace

void DescribeModeEnergies(const ElasticCouponModel& model,const ShellPatchInertia& inertia,ThinShellInertiaPolicy policy,
                         const ThinShellVector& root,bool finite_amplitude,ThinShellSpectrumDiagnostic& spectrum) {
    if (!spectrum.available) return;
    const auto director=Directors(model.data());
    for (std::size_t m=0;m<kCouponFreeDofs;++m) {
        auto& diagnostic=spectrum.modes[m];
        const ThinShellVector mode=root.asDiagonal()*spectrum.mass_modes.col(m);
        Nodes velocity{},omega{};
        for (std::size_t free=0;free<kCouponFreeNodes.size();++free) {
            velocity[kCouponFreeNodes[free]]={mode(6*free),mode(6*free+1),mode(6*free+2)};
            omega[kCouponFreeNodes[free]]={mode(6*free+3),mode(6*free+4),mode(6*free+5)};
        }
        diagnostic.kinetic_status=ComputeShellPatchKineticEnergy(inertia,director,velocity,omega,
                                                                diagnostic.kinetic,diagnostic.kinetic_diagnostic);
        diagnostic.kinetic_available=diagnostic.kinetic_status==Status::kSuccess;
        if (policy==ThinShellInertiaPolicy::PhysicalThickness) {
            diagnostic.kinetic.added_tangential=0; diagnostic.kinetic.added_drilling=0;
        }
        if (finite_amplitude) FiniteAmplitude(model,mode,diagnostic);
    }
}

void EstimateSteps(const ElasticCouponParameters& p,double edge,ThinShellInertiaPolicy policy,
                   const ThinShellMatrix& fine,const ThinShellVector& root,ThinShellStepDiagnostic& output) {
    ThinShellStepDiagnostic result;
    const ThinShellMatrix scaled=patch_audit::MassScale<kCouponFreeDofs>(fine,root);
    if (!scaled.allFinite()) {
        output=ThinShellStepDiagnostic{}; output.diagnostic="Neutral mass-scaled matrix is unrepresentable"; return;
    }
    result.neutral_operator_norm=patch_audit::OperatorNorm<kCouponFreeDofs>(scaled);
    result.membrane_wave_speed=std::sqrt(p.young_modulus/(p.density*(1-p.poisson_ratio*p.poisson_ratio)));
    const double shear=p.young_modulus/(2*(1+p.poisson_ratio));
    result.neutral_spectral_limit=.1/std::sqrt(2*result.neutral_operator_norm);
    result.neutral_spectral_steps_200ms=std::ceil(kThinShellForecastHorizon/result.neutral_spectral_limit);
    result.wave_limit=.1*edge/result.membrane_wave_speed;
    result.physical_rotary_limit=.1*p.thickness/std::sqrt(12*shear/p.density);
    for (double value:{result.neutral_operator_norm,result.membrane_wave_speed,result.neutral_spectral_limit,
                      result.neutral_spectral_steps_200ms,result.wave_limit,result.physical_rotary_limit})
        if (!std::isfinite(value)||value<=0) {
            output=ThinShellStepDiagnostic{};
            output.diagnostic="Neutral-only timestep estimate is unrepresentable"; return;
        }
    if (policy==ThinShellInertiaPolicy::PhysicalThickness) {
        result.original_policy_limit=std::min({result.neutral_spectral_limit,result.wave_limit,result.physical_rotary_limit});
        result.original_policy_steps_200ms=std::ceil(kThinShellForecastHorizon/result.original_policy_limit);
        result.original_policy_estimate_available=true;
    }
    result.available=true; output=std::move(result);
}
} // namespace crash::reference::thin_shell_detail
