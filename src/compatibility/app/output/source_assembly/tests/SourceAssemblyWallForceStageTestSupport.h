#pragma once
#include "SourceAssemblyWallFieldTestSupport.h"

namespace crash::output::assembly::test {
using cases::source_assembly_observation::ForceStageSummary;
inline ForceStageSummary SyntheticStage(const WallFields& f) {
    ForceStageSummary s;s.owner_id=f.stamp.owner_id;s.source=f.stamp.rigid_groups;
    s.base_epoch=f.stamp.reaction_base_epoch;s.enclosing_epoch=f.stamp.epoch;s.attempt=f.diagnostics.shells.qeph.attempt;
    s.enclosing_time=f.stamp.time;s.phase={f.stamp.reaction_time,f.diagnostics.shells.qeph.base_velocity_time,
        f.diagnostics.motion.before.phase.frame_time,{f.stamp.epoch==1?0:f.stamp.fixed_dt,f.stamp.reaction_kick_dt,f.stamp.fixed_dt}};
    // Formatting-only channel markers, never claimed as source dynamics.
    s.ordinary={2,3,1,2,5,0};s.grouped_members={7,11,4,7,18,0};
    s.groups={13,17,30,12,1,5,7,3,4,2,1,2,0,0};s.native_total=23;s.effective_total=35;s.replacement=12;return s;
}
} // namespace crash::output::assembly::test
