#include "ReplayBudget.h"
#include "IntervalIO.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
namespace crash::output::physical_run::replay_detail {
Memory Budget(const records::Context& context,const Configuration& config,
              std::size_t source_bytes,std::size_t host_cap) {
    Require(host_cap && host_cap<=512u<<20 && source_bytes<=host_cap &&
        !(config.wall&&config.environment),"Invalid physical replay memory envelope");
    Memory out;
    out.interval_workspace=IntervalReadStagingBytes(config.profile,
        config.request.intervals,config.request.file_byte_cap);
    out.frame_workspace=3*sizeof(double)*(3*context.nodes()+context.points());
    out.environment_workspace=config.wall?WallWorkspaceBytes:
        config.environment?EnvironmentWorkspaceBytes:0;
    out.sequential_workspace=std::max({out.interval_workspace,out.frame_workspace,
        out.environment_workspace});
    tl::util::BoundedArenaLayout budget(host_cap);tl::util::ArenaRegion region;
    Require(budget.Append<std::byte>(source_bytes,region) &&
        budget.Append<std::byte>(context.retained_payload_bytes(),region) &&
        budget.Append<std::byte>(32*MetadataCap,region) &&
        budget.Append<std::byte>(out.sequential_workspace,region) &&
        budget.Append<std::byte>(config.wall?WallMeshRetainedBytes:
            config.environment?EnvironmentRetainedBytes:0,region),
        "Physical replay retained/peak buffers exceed host cap");
    out.peak_host_bytes=budget.bytes();return out;
}
}
