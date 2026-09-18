#pragma once

#include "../Run.h"
#include <functional>

namespace crash::cases::vehicle_run::test {

inline constexpr double FixedStepS = 2e-7;
Config CombinedConfig();
using TwoIntervalExecution = std::function<Result(
    const PreparedRun&, const std::filesystem::path&, const Control&)>;

// One acceptance contract for ordinary and observed native execution. The
// operation must return the actual controller result without changing it.
void CheckTwoCommittedV5Intervals(const TwoIntervalExecution&);

} // namespace crash::cases::vehicle_run::test
