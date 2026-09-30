#include "Replay.h"
#include "ReplayBudget.h"
#include "IntervalIO.h"
namespace crash::output::physical_run {
std::size_t Replay::Preflight(const records::Context& context,const Configuration& config,ReplayLimits limits) {
    replay_detail::CheckLimits(limits);
    const auto memory=replay_detail::Budget(context,config,limits.source.host_bytes,limits.host_bytes);
    CheckIntervalReadWorkspace(memory.interval_workspace,128u<<20);
    return memory.peak_host_bytes;
}
} // namespace crash::output::physical_run
