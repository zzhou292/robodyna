#include "GuidedPlateAdmission.h"
#include "GuidedPlateExperimentProtocol.h"
#include "lib_src/collision/Q4ContactBounds.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::case_data {
namespace shell=tl::fea::reissner;
namespace sc=tlfea::contact;
namespace {
bool Nonnegative(double value) { return std::isfinite(value) && value>=0; }
bool Certified(const sc::Q4CertifiedIntegral& value) {
    sc::Q4CertifiedIntegral checked;
    return Nonnegative(value.error) &&
           sc::q4_bounds::Certify(value.value,{value.lower,value.upper},&checked) && checked.error<=value.error;
}
}

bool CheckGuidedPlateEnvelope(const shell::ShellBatchDiagnostics& d,
                              const sc::Q4PlanarContactDiagnostics& c,
                              const reference::GuidedPlateModalReport& modal,
                              double initial_energy,double maximum_displacement,bool candidate,
                              GuidedPlateWorkReport& output,std::string& error) {
    auto reject=[&](const std::string& message) { error=message; return false; };
    if(!std::isfinite(initial_energy) || initial_energy<=0 || !c.valid || !d.valid ||
       !d.owner_id || !d.configuration_id || !d.attempt || !c.wall_binding_id ||
       d.owner_id!=c.owner_id || d.base_epoch!=c.base_epoch || d.attempt!=c.attempt ||
       d.configuration_id!=c.configuration_id||!GuidedExperimentIdentity(modal.experiment,modal.qualification_id)||
       d.configuration_id!=modal.qualification_id)
        return reject("Guided shell/contact diagnostics have invalid or mismatched identities");
    if(!CheckElasticShellGeometry(d,maximum_displacement,error)) return false;
    double required_rate=0;
    if(c.parent_count!=reference::kCouponElements || c.covered_count!=c.parent_count ||
       !Certified(c.potential) || !sc::q4_bounds::Nonnegative(c.active_area) ||
       !Nonnegative(c.maximum_penetration) || c.maximum_penetration>reference::GuidedPlateData::maximum_penetration ||
       !std::isfinite(c.stiffness_rate_bound) || c.stiffness_rate_bound<=0 ||
       c.stiffness_rate_bound!=modal.contact_rate_bound ||
       !std::isfinite(modal.monitored_structural_norm_limit) || modal.monitored_structural_norm_limit<=0 ||
       !std::isfinite(modal.combined_rate_envelope) ||
       !sc::q4_bounds::AddScalar(modal.monitored_structural_norm_limit,modal.contact_rate_bound,true,&required_rate) ||
       modal.combined_rate_envelope<required_rate)
        return reject("Guided contact geometry, potential certificate or rate envelope is invalid");
    for(const auto value:{c.force_on_surface,c.wall_reaction,c.wall_moment,c.force_error,c.wall_moment_error})
        if(!sc::IsFinite(value)) return reject("Nonfinite guided contact force/moment diagnostic");
    if(!Nonnegative(c.force_error.x) || !Nonnegative(c.force_error.y) || !Nonnegative(c.force_error.z) ||
       !Nonnegative(c.wall_moment_error.x) || !Nonnegative(c.wall_moment_error.y) || !Nonnegative(c.wall_moment_error.z) ||
       !std::isfinite(c.surface_power) || !Nonnegative(c.surface_power_error))
        return reject("Invalid guided contact force/work uncertainty");

    GuidedPlateWorkReport next;
    next.kinetic_energy=ShellKineticEnergy(d);
    next.total_energy=next.kinetic_energy+d.elastic_energy+c.potential.value;
    next.relative_energy_error=std::abs(next.total_energy-initial_energy)/initial_energy;
    // Initial contact is identically inactive by this case's declared gap.
    // Include the current numerical contact certificate in the energy gate.
    if(!std::isfinite(next.total_energy) ||
       next.relative_energy_error+c.potential.error/initial_energy>ElasticShellLimits::energy_fraction)
        return reject(ShellLimitDiagnostic("Guided total energy envelope exceeded",
                      next.relative_energy_error+c.potential.error/initial_energy,ElasticShellLimits::energy_fraction));
    if(candidate) {
        if(d.phase!=shell::ShellBatchPhase::kPreparedCandidate ||
           c.phase!=sc::Q4PlanarContactPhase::PreparedCandidate)
            return reject("Guided interval diagnostics are not a matching prepared candidate");
        for(double value:{d.base_kinetic_energy,d.base_elastic_energy,d.mass_weighted_increment_squared,
                          c.base_potential,c.base_potential_error,c.kinetic_midpoint_roundoff,
                          c.force_coordinate_roundoff,c.continuum_work_uncertainty,c.quadratic_work_upper})
            if(!Nonnegative(value)) return reject("Invalid guided candidate energy/work bound");
        for(double value:{d.elastic_energy_increment,d.kinetic_energy_increment,d.kinetic_midpoint_work,
                          d.force_coordinate_work,d.kinetic_work_residual,d.conservative_force_coordinate_defect,
                          c.potential_increment,c.kinetic_midpoint_work,c.force_coordinate_work,
                          c.conservative_force_coordinate_defect})
            if(!std::isfinite(value)) return reject("Nonfinite guided candidate interval diagnostic");
        constexpr double eps=std::numeric_limits<double>::epsilon();
        next.combined_kinetic_residual=d.kinetic_energy_increment-d.kinetic_midpoint_work-c.kinetic_midpoint_work;
        next.kinetic_arithmetic_budget=128*eps*std::max({initial_energy,next.kinetic_energy,d.base_kinetic_energy})+
                                      c.kinetic_midpoint_roundoff;
        if(!std::isfinite(next.combined_kinetic_residual) || !std::isfinite(next.kinetic_arithmetic_budget) ||
           std::abs(next.combined_kinetic_residual)>next.kinetic_arithmetic_budget)
            return reject(ShellLimitDiagnostic("Guided combined discrete kinetic identity failed",
                          std::abs(next.combined_kinetic_residual),next.kinetic_arithmetic_budget));
        const double elastic_floor=128*eps*reference::ElasticCouponData::young_modulus*
            reference::ElasticCouponData::thickness*reference::ElasticCouponData::length*reference::ElasticCouponData::width;
        next.shell_coordinate_work_budget=.75*modal.monitored_structural_norm_limit*d.mass_weighted_increment_squared+elastic_floor;
        if(!std::isfinite(next.shell_coordinate_work_budget) ||
           std::abs(d.conservative_force_coordinate_defect)>next.shell_coordinate_work_budget)
            return reject(ShellLimitDiagnostic("Guided shell force-potential increment failed",
                          std::abs(d.conservative_force_coordinate_defect),next.shell_coordinate_work_budget));
        next.contact_defect_lower_limit=-c.continuum_work_uncertainty;
        next.contact_defect_upper_limit=c.quadratic_work_upper+c.continuum_work_uncertainty;
        if(!std::isfinite(next.contact_defect_upper_limit) ||
           c.conservative_force_coordinate_defect<next.contact_defect_lower_limit ||
           c.conservative_force_coordinate_defect>next.contact_defect_upper_limit)
            return reject("Guided contact force-potential defect is outside its certified convexity bounds");
    } else if(d.phase!=shell::ShellBatchPhase::kAcceptedBase || c.phase!=sc::Q4PlanarContactPhase::AcceptedBase ||
              c.potential.upper!=0 || c.active_area.upper!=0 || c.maximum_penetration!=0) {
        return reject("Guided initial diagnostics must describe an inactive-contact accepted base");
    }
    output=next; error.clear(); return true;
}
}  // namespace crash::case_data
