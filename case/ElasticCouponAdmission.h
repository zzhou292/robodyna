#pragma once
#include "chrono/ElasticCouponModal.h"
#include "lib_src/elements/ReissnerShellBatch.h"
#include <string>

namespace crash::case_data {
// Fixed numerical envelope of the synthetic two-Q4 experiment. Changing these
// values changes the declared experiment and requires fresh qualification.
struct ElasticCouponLimits {
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
double CouponKineticEnergy(const tl::fea::reissner::ShellBatchDiagnostics&);
std::string CouponLimitDiagnostic(const char* quantity,double measured,double limit);
bool CheckElasticCouponEnvelope(const tl::fea::reissner::ShellBatchDiagnostics&,
                                const reference::ElasticCouponModalReport&, double initial_energy,
                                double maximum_displacement, bool candidate, std::string& diagnostic);
}  // namespace crash::case_data
