#include "ElasticCouponAdmission.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::case_data {
namespace shell = tl::fea::reissner;
std::string CouponLimitDiagnostic(const char* quantity,double measured,double limit) {
    return ShellLimitDiagnostic(quantity,measured,limit);
}
double CouponKineticEnergy(const shell::ShellBatchDiagnostics& d) {
    return ShellKineticEnergy(d);
}
bool CheckElasticCouponEnvelope(const shell::ShellBatchDiagnostics& d,
                                const reference::ElasticCouponModalReport& modal, double initial_energy,
                                double maximum_displacement, bool candidate, std::string& error) {
    auto reject=[&](const std::string& message) { error=message; return false; };
    if(!d.valid || !std::isfinite(initial_energy) || initial_energy<=0 ||
       !std::isfinite(maximum_displacement) || maximum_displacement<=0 ||
       maximum_displacement>ElasticCouponLimits::displacement)
        return reject("Invalid coupon envelope request or unqualified diagnostics");
    if(!CheckElasticShellGeometry(d,maximum_displacement,error)) return false;
    const double kinetic=CouponKineticEnergy(d),total=kinetic+d.elastic_energy;
    if(!std::isfinite(total) || std::abs(total-initial_energy)>ElasticCouponLimits::energy_fraction*initial_energy)
        return reject(CouponLimitDiagnostic("Coupon relative total energy envelope exceeded",
                                            std::abs(total-initial_energy)/initial_energy,ElasticCouponLimits::energy_fraction));
    if(candidate) {
        if(d.phase!=shell::ShellBatchPhase::kPreparedCandidate || !std::isfinite(d.base_kinetic_energy) ||
           d.base_kinetic_energy<0 || !std::isfinite(d.base_elastic_energy) || d.base_elastic_energy<0 ||
           !std::isfinite(d.elastic_energy_increment) || !std::isfinite(d.kinetic_energy_increment) ||
           !std::isfinite(d.kinetic_midpoint_work) || !std::isfinite(d.force_coordinate_work) ||
           !std::isfinite(d.kinetic_work_residual) || !std::isfinite(d.conservative_force_coordinate_defect) ||
           !std::isfinite(d.mass_weighted_increment_squared) || d.mass_weighted_increment_squared<0 ||
           !std::isfinite(modal.monitored_norm_limit) || modal.monitored_norm_limit<=0)
            return reject("Invalid candidate interval diagnostic");
        constexpr double eps=std::numeric_limits<double>::epsilon();
        const double kinetic_budget=128*eps*std::max({initial_energy,kinetic,d.base_kinetic_energy});
        if(std::abs(d.kinetic_work_residual)>kinetic_budget)
            return reject(CouponLimitDiagnostic("Coupon discrete kinetic work identity failed",std::abs(d.kinetic_work_residual),kinetic_budget));
        const double elastic_floor=128*eps*reference::ElasticCouponData::young_modulus*
            reference::ElasticCouponData::thickness*reference::ElasticCouponData::length*reference::ElasticCouponData::width;
        const double work_budget=.75*modal.monitored_norm_limit*d.mass_weighted_increment_squared+elastic_floor;
        if(!std::isfinite(work_budget) || std::abs(d.conservative_force_coordinate_defect)>work_budget)
            return reject(CouponLimitDiagnostic("Coupon force-potential increment budget exceeded",
                                                std::abs(d.conservative_force_coordinate_defect),work_budget));
    } else if(d.phase!=shell::ShellBatchPhase::kAcceptedBase) {
        return reject("Initial diagnostics do not describe the accepted base");
    }
    error.clear(); return true;
}
}  // namespace crash::case_data
