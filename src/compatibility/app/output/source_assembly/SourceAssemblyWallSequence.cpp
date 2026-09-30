#include "SourceAssemblyWallSequence.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace crash::output::assembly {
void WallArchiveSequence::CheckInterval(const tl::fea::NodalStamp& base,const tl::fea::NodalStamp& next) const {
    Require(request.frame_every&&frame_count&&!prefix_frame&&tl::fea::trial_identity::SameStamp(base,last)&&
        next.epoch==base.epoch+1&&next.epoch<=request.steps&&next.owner_id==base.owner_id&&
        next.node_count==base.node_count&&next.fixed_dt==base.fixed_dt&&next.has_rotations==base.has_rotations&&
        next.temporal_scheme==base.temporal_scheme&&tl::fea::SameRigidGroupInfo(next.rigid_groups,base.rigid_groups)&&
        next.reaction_base_epoch==base.epoch&&next.reaction_time==base.time&&next.time==base.time+base.fixed_dt&&
        (base.epoch%request.frame_every||last_frame==base.epoch),
        "Assembly interval skipped an indexed endpoint or changed its exact base");
}
bool WallArchiveSequence::CheckFrame(const tl::fea::NodalStamp& stamp) const {
    Require(request.frame_every&&!prefix_frame&&frame_count<frame_capacity&&tl::fea::trial_identity::SameStamp(stamp,last)&&
        (!frame_count?stamp.epoch==0:stamp.epoch>last_frame),"Assembly frame is duplicate, unrecorded or exceeds its capacity");
    return stamp.epoch&&stamp.epoch<request.steps&&stamp.epoch%request.frame_every;
}
void WallArchiveSequence::CheckClose(const tl::fea::NodalStamp& stamp,bool prefix) const {
    Require(stamp.epoch&&tl::fea::trial_identity::SameStamp(stamp,last)&&frame_count&&last_frame==stamp.epoch&&
        (prefix?stamp.epoch<request.steps:stamp.epoch==request.steps),"Assembly archive cannot close an incomplete accepted endpoint or wrong horizon");
}
} // namespace crash::output::assembly
