#include "Internal.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>

namespace crash::output::recovered_frames {
std::size_t Budget(const records::Context& context,const run::Configuration& config,Limits limits) {
    Require(limits.host_bytes && limits.host_bytes<=512u<<20 && limits.source.host_bytes<=limits.host_bytes &&
        (128u<<20)<=limits.host_bytes-limits.source.host_bytes,"Recovery source/record workspace exceeds host cap");
    const auto plan=records::activity::PlanWithActivity(context,config.request,"parent-activity.json");
    Require(plan.archive.forecast_bytes<=config.request.total_byte_cap,"Original archive forecast exceeds its cap");
    tl::util::BoundedArenaLayout budget(limits.host_bytes),sample(limits.host_bytes);
    tl::util::ArenaRegion region;
    for(unsigned copy=0;copy<3;++copy) {
        Require(sample.Append<double>(3*context.nodes(),region) && sample.Append<double>(context.points(),region),
            "Recovery frame staging exceeds cap");
    }
    Require(budget.Append<std::byte>(limits.source.host_bytes,region) &&
        budget.Append<std::byte>(context.retained_payload_bytes(),region) &&
        budget.Append<std::byte>(32*run::MetadataCap,region) &&
        budget.Append<std::byte>(std::max<std::size_t>({sample.bytes(),kArtifactFileCap,
            config.wall?run::WallWorkspaceBytes:config.environment?run::EnvironmentWorkspaceBytes:0}),region) &&
        budget.Append<std::byte>(config.wall?run::WallMeshRetainedBytes:config.environment?run::EnvironmentRetainedBytes:0,region),
        "Recovery retained/source/sample/copy budget exceeds cap");
    return budget.bytes();
}
} // namespace crash::output::recovered_frames
