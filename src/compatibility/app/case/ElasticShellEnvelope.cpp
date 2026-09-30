#include "ElasticShellEnvelope.h"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace crash::case_data {
namespace shell=tl::fea::reissner;
std::string ShellLimitDiagnostic(const char* quantity,double measured,double limit) {
    std::ostringstream message;
    message<<quantity<<": measured="<<std::setprecision(17)<<measured<<", limit="<<limit;
    return message.str();
}
double ShellKineticEnergy(const shell::ShellBatchDiagnostics& d) {
    return d.kinetic_translation+d.kinetic_physical_rotation+d.kinetic_artificial_drilling;
}
bool CheckElasticShellGeometry(const shell::ShellBatchDiagnostics& d,double maximum_displacement,
                               std::string& error) {
    auto reject=[&](const std::string& message) { error=message; return false; };
    if(!d.valid || !std::isfinite(maximum_displacement) || maximum_displacement<=0 ||
       maximum_displacement>ElasticShellLimits::displacement)
        return reject("Invalid elastic shell envelope request or unqualified diagnostics");
    for(double value:{d.elastic_energy,d.bending_energy,d.kinetic_translation,d.kinetic_physical_rotation,
                      d.kinetic_artificial_drilling,d.maximum_displacement,d.maximum_director_departure,
                      d.maximum_pair_angle,d.maximum_membrane_strain,d.maximum_thickness_curvature})
        if(!std::isfinite(value) || value<0) return reject("Nonfinite or negative elastic shell diagnostic");
    if(d.maximum_displacement>maximum_displacement)
        return reject(ShellLimitDiagnostic("Shell displacement envelope exceeded",d.maximum_displacement,maximum_displacement));
    if(d.maximum_director_departure>ElasticShellLimits::director_departure ||
       d.maximum_pair_angle>=ElasticShellLimits::pair_angle)
        return reject("Shell director envelope exceeded");
    if(d.maximum_membrane_strain>ElasticShellLimits::strain ||
       d.maximum_thickness_curvature>ElasticShellLimits::thickness_curvature)
        return reject("Shell strain envelope exceeded");
    if(!std::isfinite(d.minimum_signed_area_ratio) || !std::isfinite(d.minimum_area_norm_ratio) ||
       !std::isfinite(d.maximum_area_norm_ratio) || !std::isfinite(d.minimum_display_triangle_area_ratio) ||
       d.minimum_signed_area_ratio<ElasticShellLimits::minimum_area_ratio ||
       d.minimum_area_norm_ratio<ElasticShellLimits::minimum_area_ratio ||
       d.maximum_area_norm_ratio>ElasticShellLimits::maximum_area_ratio ||
       d.minimum_display_triangle_area_ratio<=1e-12)
        return reject("Shell area/orientation envelope exceeded");
    error.clear(); return true;
}
}  // namespace crash::case_data
