#include "QephSpinTrace.h"
namespace crash::output::assembly {
std::size_t PlanQephSpinTrace(std::uint64_t steps,std::uint64_t cadence) {
    Require(steps&&steps<=1u<<20&&cadence&&cadence<=steps,"Spin trace requires bounded positive steps/cadence");
    // Header/footer + first stage + cadence rows + a possible unsaved terminal.
    const auto rows=4+steps/cadence;
    Require(rows<=SpinTraceByteCap/SpinTraceRowCap,"Requested spin trace exceeds its 256 MiB forecast");
    return static_cast<std::size_t>(rows)*SpinTraceRowCap;
}
}
