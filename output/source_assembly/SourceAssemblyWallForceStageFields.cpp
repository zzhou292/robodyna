#include "SourceAssemblyWallFields.h"
#include "SourceAssemblyWallKineticChannels.h"
#include "lib_src/constraints/NodalRigidForceStageChecks.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace crash::output::assembly::wall_fields {
namespace fe=tl::fea;
void CheckForceStageFrame(const FrameView& v) {
    const auto* f=v.force_stage;
    if(!v.observe_force_stage||!v.stamp->epoch) {
        Require(!f,"Disabled or initial assembly output cannot carry a force-stage sample");return;
    }
    Require(f,"Enabled accepted assembly output is missing its force-stage sample");
    const auto& s=*v.stamp;const auto& q=v.diagnostics->shells.qeph;
    const auto& p=f->phase;const auto& dt=p.durations;
    Require(f->owner_id==s.owner_id&&fe::SameRigidGroupInfo(f->source,s.rigid_groups)&&
        f->base_epoch==s.reaction_base_epoch&&f->enclosing_epoch==s.epoch&&f->attempt==q.attempt&&f->attempt&&
        f->enclosing_time==s.time&&p.force_time==s.reaction_time&&p.input_velocity_time==q.base_velocity_time&&
        p.previous_frame_time==v.diagnostics->motion.before.phase.frame_time&&
        dt.previous_drift_dt==(s.epoch==1?0:s.fixed_dt)&&dt.kick_dt==s.reaction_kick_dt&&dt.drift_dt==s.fixed_dt&&
        fe::rigid::force_stage_detail::Phase(p),"Force-stage output does not belong to this accepted interval");
}
Document ForceStageDocument(const cases::source_assembly_observation::ForceStageSummary& f,bool has_connectors) {
    Document d;d.SetObject();String(d,"kind",ForceStageKind);
    Integer(d,"owner_id",f.owner_id);Integer(d,"base_epoch",f.base_epoch);Integer(d,"attempt",f.attempt);
    Integer(d,"enclosing_epoch",f.enclosing_epoch);Number(d,"enclosing_time_s",f.enclosing_time);
    Document source;source.SetObject();Integer(source,"source_instance_id",f.source.source_instance_id);
    Integer(source,"group_count",f.source.group_count);Integer(source,"member_count",f.source.member_count);Child(d,"source",source);
    Document p;p.SetObject();Number(p,"force_time_s",f.phase.force_time);
    Number(p,"input_velocity_time_s",f.phase.input_velocity_time);Number(p,"previous_frame_time_s",f.phase.previous_frame_time);
    Number(p,"previous_drift_dt_s",f.phase.durations.previous_drift_dt);Number(p,"kick_dt_s",f.phase.durations.kick_dt);
    Number(p,"drift_dt_s",f.phase.durations.drift_dt);Child(d,"phase",p);
    KineticChannels(d,f.ordinary,f.grouped_members,f.groups,f.native_total,f.effective_total);
    if(has_connectors)Put(d,"connector_kinetic_J",Values(d,{f.connector.translation,f.connector.rotation}));
    Number(d,"replacement_J",f.replacement);return d;
}
} // namespace crash::output::assembly::wall_fields
