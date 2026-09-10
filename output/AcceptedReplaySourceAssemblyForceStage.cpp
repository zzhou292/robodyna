#include "AcceptedReplaySourceAssembly.h"
#include "source_assembly/SourceAssemblyKineticSchema.h"

namespace crash::output::replay_detail {
void CheckAssemblyForceStage(const Bundle& b,const Entry& e,const Value& frame) {
    const auto& a=*b.assembly;
    if(!a.observe_force_stage) {
        Require(!frame.HasMember("force_stage_kinetic"),"Undeclared assembly force-stage observation");return;
    }
    const auto& f=Member(frame,"force_stage_kinetic");
    if(!e.epoch) {Require(f.IsNull(),"Initial accepted frame cannot contain an executed force stage");return;}
    Require(Text(f,"kind")==assembly::ForceStageKind&&Unsigned(f,"owner_id")==e.owner&&
        Unsigned(f,"base_epoch")==e.epoch-1&&Unsigned(f,"enclosing_epoch")==e.epoch&&
        Unsigned(f,"attempt")==e.interval_attempt,"Assembly force-stage accepted interval identity changed");
    AssemblyEqual(Real(f,"enclosing_time_s"),e.time);
    const auto& source=Member(f,"source");
    Require(Unsigned(source,"source_instance_id")==a.instance&&Unsigned(source,"group_count")==a.group_count&&
        Unsigned(source,"member_count")==a.member_count,"Assembly force-stage source association changed");
    const auto& p=Member(f,"phase");const bool first=e.epoch==1;
    AssemblyEqual(Real(p,"force_time_s"),e.interval_base_time);
    AssemblyEqual(Real(p,"input_velocity_time_s"),e.interval_base_velocity_time);
    AssemblyEqual(Real(p,"previous_frame_time_s"),first?0:e.interval_previous_base_time);
    AssemblyEqual(Real(p,"previous_drift_dt_s"),first?0:b.fixed_dt);
    AssemblyEqual(Real(p,"kick_dt_s"),first?.5*b.fixed_dt:b.fixed_dt);
    AssemblyEqual(Real(p,"drift_dt_s"),b.fixed_dt);
    // These are persisted force-stage partitions, not reconstructed from the
    // differently phased accepted nodal velocities and not a global balance.
    const auto channels=CheckAssemblyKineticChannels(b,f);
    const long double aggregate=channels.groups[2],members=channels.members[4];
    AssemblyReduction(Real(f,"replacement_J"),aggregate-members,aggregate+members,a.group_count);
}
} // namespace crash::output::replay_detail
