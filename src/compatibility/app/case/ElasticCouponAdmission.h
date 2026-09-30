#pragma once
#include "ElasticShellEnvelope.h"
#include "chrono/ElasticCouponModal.h"
#include "lib_src/elements/ReissnerShellBatch.h"
#include <string>

namespace crash::case_data {
// Fixed numerical envelope of the synthetic two-Q4 experiment. Changing these
// values changes the declared experiment and requires fresh qualification.
struct ElasticCouponLimits : ElasticShellLimits {};
double CouponKineticEnergy(const tl::fea::reissner::ShellBatchDiagnostics&);
std::string CouponLimitDiagnostic(const char* quantity,double measured,double limit);
bool CheckElasticCouponEnvelope(const tl::fea::reissner::ShellBatchDiagnostics&,
                                const reference::ElasticCouponModalReport&, double initial_energy,
                                double maximum_displacement, bool candidate, std::string& diagnostic);
}  // namespace crash::case_data
