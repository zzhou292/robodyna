#include "GuidedPlateModal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::reference {
namespace {
namespace audit=patch_audit;
namespace tlr=tl::fea::reissner;
using Status=ElasticCouponStatus;
using Matrix=audit::Matrix<kGuidedPlateDofs>;
using Vector=audit::Vector<kGuidedPlateDofs>;

Status SelectMode(const GuidedPlateModel& model,const audit::ReferenceSpectrum<kGuidedPlateDofs>& spectrum,
                  GuidedPlateModalReport& report,std::string& diagnostic) {
    const Vector mass_mode=spectrum.mass_modes.col(0);
    const Vector mode=spectrum.inverse_root_mass.asDiagonal()*mass_mode;
    for (std::size_t free=0;free<kCouponFreeNodes.size();++free) {
        report.normal_translation_mass_fraction+=std::pow(mass_mode(4*free),2);
        for (const auto c:{0,2,3}) report.bending_mass_fraction+=std::pow(mass_mode(4*free+c),2);
    }
    if (report.bending_mass_fraction<.9)
        return audit::Reject("Guided lowest mode is not predominantly normal bending",report.bending_mass_fraction,.9,diagnostic);
    if (report.normal_translation_mass_fraction<.5)
        return audit::Reject("Guided lowest mode lacks normal translational participation",
                             report.normal_translation_mass_fraction,.5,diagnostic);
    const double tip_a=mode(8),tip_b=mode(12),mean_tip=.5*(tip_a+tip_b);
    if (!std::isfinite(mean_tip) || std::abs(mean_tip)<=.5*std::max(std::abs(tip_a),std::abs(tip_b)))
        return audit::Reject("Guided lowest mode lacks coherent nonzero normal tip motion",mean_tip,0,diagnostic);
    const double scale=GuidedPlateData::initial_tip_displacement/mean_tip;
    for (std::size_t c=0;c<kGuidedPlateDofs;++c) report.initial_mode_increment[c]=scale*mode(c);
    return ApplyGuidedPlateIncrement(model.shell().data().reference_configuration,report.initial_mode_increment,1,
                                     report.initial_configuration,diagnostic);
}
}

ElasticCouponStatus MeasureGuidedPlateStructuralNorm(const GuidedPlateModel& model,
                                                     const ElasticCouponConfiguration& configuration,
                                                     double& output,std::string& diagnostic) {
    Matrix stiffness; const auto layout=GuidedPlateLayout();
    const auto status=audit::DifferenceJacobian<kGuidedPlateDofs>(model.shell(),configuration,layout,
        audit::ForceSource::Chrono,.5,stiffness,diagnostic);
    if (status!=Status::kSuccess) return status;
    const double norm=audit::OperatorNorm<kGuidedPlateDofs>(audit::MassScale<kGuidedPlateDofs>(stiffness,
        audit::InverseRootMass<kGuidedPlateDofs>(model.shell().data(),layout)));
    if (!std::isfinite(norm) || norm<=0) return audit::Reject("Invalid guided structural operator norm",norm,0,diagnostic);
    output=norm; diagnostic.clear(); return Status::kSuccess;
}

ElasticCouponStatus AuditGuidedPlate(const GuidedPlateModel& model,GuidedPlateModalReport& output,std::string& diagnostic) {
    const auto& shell=model.shell(); const auto& data=shell.data();
    const auto& neutral=data.reference_configuration; const auto layout=GuidedPlateLayout();
    audit::ReferenceSpectrum<kGuidedPlateDofs> spectrum;
    auto status=audit::AuditReference<kGuidedPlateDofs>(shell,layout,spectrum,diagnostic);
    if (status!=Status::kSuccess) return status;
    GuidedPlateModalReport report;
    report.experiment=model.data().experiment;
    report.qualification_id=model.data().qualification_id;
    report.reference_symmetry_error=spectrum.symmetry_error;
    report.mass_scaled_derivative_refinement_error=spectrum.derivative_refinement_error;
    report.tl_derivative_relative_error=spectrum.tl_derivative_error;
    report.maximum_frequency_refinement_error=spectrum.maximum_frequency_refinement_error;
    report.eigen_residual=spectrum.eigen_residual;
    for (std::size_t c=0;c<kGuidedPlateDofs;++c) report.squared_frequency[c]=spectrum.squared_frequency(c);
    status=SelectMode(model,spectrum,report,diagnostic); if (status!=Status::kSuccess) return status;
    for (std::size_t sample=0;sample<report.sampled_amplitude.size();++sample) {
        ElasticCouponConfiguration configuration;
        status=ApplyGuidedPlateIncrement(neutral,report.initial_mode_increment,report.sampled_amplitude[sample],configuration,diagnostic);
        if (status!=Status::kSuccess) return status;
        Matrix stiffness;
        if (report.sampled_amplitude[sample]==0) stiffness=spectrum.stiffness;
        else {
            status=audit::DifferenceJacobian<kGuidedPlateDofs>(shell,configuration,layout,audit::ForceSource::Chrono,.5,stiffness,diagnostic);
            if (status!=Status::kSuccess) return status;
        }
        const double norm=audit::OperatorNorm<kGuidedPlateDofs>(audit::MassScale<kGuidedPlateDofs>(stiffness,spectrum.inverse_root_mass));
        if (!std::isfinite(norm) || norm<=0) return audit::Reject("Sampled guided structural norm",norm,0,diagnostic);
        report.sampled_structural_operator_norm[sample]=norm;
        report.sampled_structural_norm_maximum=std::max(report.sampled_structural_norm_maximum,norm);
        status=audit::DirectionalCrossCheck<kGuidedPlateDofs>(shell,configuration,layout,stiffness,spectrum.inverse_root_mass,
                                                            report.tl_directional_relative_error,diagnostic);
        if (status!=Status::kSuccess) return status;
    }
    report.monitored_structural_norm_limit=2*report.sampled_structural_norm_maximum;
    report.contact_rate_bound=model.contact_stiffness().rate_bound;
    if (!tlfea::contact::q4_bounds::AddScalar(report.monitored_structural_norm_limit,report.contact_rate_bound,true,
                                             &report.combined_rate_envelope))
        return audit::Reject("Unrepresentable guided combined rate",report.contact_rate_bound,0,diagnostic);
    report.membrane_wave_speed=std::sqrt(data.young_modulus/(data.density*(1-data.poisson_ratio*data.poisson_ratio)));
    report.shortest_reference_edge=std::numeric_limits<double>::infinity();
    for (const auto& connectivity:data.connectivity) for (std::size_t n=0;n<4;++n) {
        const auto edge=tlr::detail::Subtract(neutral.position[connectivity[n]],neutral.position[connectivity[(n+1)%4]]);
        report.shortest_reference_edge=std::min(report.shortest_reference_edge,std::sqrt(tlr::detail::Dot(edge,edge)));
    }
    const double shear=data.young_modulus/(2*(1+data.poisson_ratio));
    report.spectral_step_limit=.1/std::sqrt(report.combined_rate_envelope);
    report.wave_step_limit=.1*report.shortest_reference_edge/report.membrane_wave_speed;
    report.rotary_step_limit=.1*data.thickness/std::sqrt(12*shear/data.density);
    report.proposed_step_limit=std::min({report.spectral_step_limit,report.wave_step_limit,report.rotary_step_limit});
    report.first_mode_angular_frequency=std::sqrt(report.squared_frequency.front());
    const double steps=std::ceil(GuidedPlateData::requested_horizon/report.proposed_step_limit);
    if (!std::isfinite(steps) || steps<1 || steps>1000000)
        return audit::Reject("Guided horizon exceeds one-million-step startup cap",steps,1000000,diagnostic);
    report.step_count=static_cast<std::uint64_t>(steps);
    report.time_step=GuidedPlateData::requested_horizon/report.step_count;
    if (report.time_step>report.proposed_step_limit) {
        if (report.step_count==1000000)
            return audit::Reject("Guided rounded step exceeds one-million-step startup cap",report.time_step,report.proposed_step_limit,diagnostic);
        report.time_step=GuidedPlateData::requested_horizon/++report.step_count;
    }
    report.horizon=report.step_count*report.time_step;
    ElasticCouponEvaluation initial;
    status=shell.EvaluateTL(report.initial_configuration,initial,diagnostic);
    if (status!=Status::kSuccess) return status;
    if (!std::isfinite(initial.energy) || initial.energy<=0)
        return audit::Reject("Guided initial elastic energy must be positive",initial.energy,0,diagnostic);
    for (const auto& point:report.initial_configuration.position)
        if (!(point.x<model.wall().wall_x()))
            return audit::Reject("Guided initial bending mode must remain separated from the wall",point.x,model.wall().wall_x(),diagnostic);
    report.initial_elastic_energy=initial.energy;
    output=report; diagnostic.clear(); return Status::kSuccess;
}
}  // namespace crash::reference
