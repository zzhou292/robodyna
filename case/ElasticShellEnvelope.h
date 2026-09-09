#pragma once

#include "lib_src/elements/ReissnerShellBatch.h"
#include <string>

namespace crash::case_data {
// Shared geometric envelope of the declared elastic coupon and guided plate.
// Energy/work admission belongs to each case's complete set of contributors.
struct ElasticShellLimits {
    static constexpr double displacement = .004;
    static constexpr double director_departure = .1;
    static constexpr double pair_angle = .25;
    static constexpr double rotation_increment = .01;
    static constexpr double strain = .01;
    static constexpr double thickness_curvature = .05;
    static constexpr double minimum_area_ratio = .8;
    static constexpr double maximum_area_ratio = 1.2;
    static constexpr double energy_fraction = .01;
};
double ShellKineticEnergy(const tl::fea::reissner::ShellBatchDiagnostics&);
std::string ShellLimitDiagnostic(const char* quantity,double measured,double limit);
bool CheckElasticShellGeometry(const tl::fea::reissner::ShellBatchDiagnostics&,
                               double maximum_displacement,std::string& diagnostic);
}  // namespace crash::case_data
