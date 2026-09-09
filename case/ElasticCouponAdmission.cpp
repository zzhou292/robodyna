#include "ElasticCouponAdmission.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>

namespace crash::case_data {
namespace shell = tl::fea::reissner;
std::string CouponLimitDiagnostic(const char* quantity,double measured,double limit) {
    std::ostringstream message;
    message<<quantity<<": measured="<<std::setprecision(17)<<measured<<", limit="<<limit;
    return message.str();
}
double CouponKineticEnergy(const shell::ShellBatchDiagnostics& d) {
    return d.kinetic_translation+d.kinetic_physical_rotation+d.kinetic_artificial_drilling;
}
bool CheckElasticCouponEnvelope(const shell::ShellBatchDiagnostics& d,
                                const reference::ElasticCouponModalReport& modal, double initial_energy,
                                double maximum_displacement, bool candidate, std::string& error) {
    auto reject=[&](const std::string& message) { error=message; return false; };
    if(!d.valid || !std::isfinite(initial_energy) || initial_energy<=0 ||
       !std::isfinite(maximum_displacement) || maximum_displacement<=0 ||
       maximum_displacement>ElasticCouponLimits::displacement)
        return reject("Invalid coupon envelope request or unqualified diagnostics");
    for(double value:{d.elastic_energy,d.bending_energy,d.kinetic_translation,d.kinetic_physical_rotation,
                      d.kinetic_artificial_drilling,d.maximum_displacement,d.maximum_director_departure,
                      d.maximum_pair_angle,d.maximum_membrane_strain,d.maximum_thickness_curvature})
        if(!std::isfinite(value) || value<0) return reject("Nonfinite or negative coupon diagnostic");
    if(d.maximum_displacement>maximum_displacement)
        return reject(CouponLimitDiagnostic("Coupon displacement envelope exceeded",d.maximum_displacement,maximum_displacement));
    if(d.maximum_director_departure>ElasticCouponLimits::director_departure ||
       d.maximum_pair_angle>=ElasticCouponLimits::pair_angle)
        return reject("Coupon director envelope exceeded");
    if(d.maximum_membrane_strain>ElasticCouponLimits::strain ||
       d.maximum_thickness_curvature>ElasticCouponLimits::thickness_curvature)
        return reject("Coupon strain envelope exceeded");
    if(!std::isfinite(d.minimum_signed_area_ratio) || !std::isfinite(d.minimum_area_norm_ratio) ||
       !std::isfinite(d.maximum_area_norm_ratio) || !std::isfinite(d.minimum_display_triangle_area_ratio) ||
       d.minimum_signed_area_ratio<ElasticCouponLimits::minimum_area_ratio ||
       d.minimum_area_norm_ratio<ElasticCouponLimits::minimum_area_ratio ||
       d.maximum_area_norm_ratio>ElasticCouponLimits::maximum_area_ratio ||
       d.minimum_display_triangle_area_ratio<=1e-12)
        return reject("Coupon area/orientation envelope exceeded");
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
