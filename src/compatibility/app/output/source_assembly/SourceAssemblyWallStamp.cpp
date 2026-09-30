#include "SourceAssemblyWallFields.h"
#include "WallFieldValues.h"

namespace crash::output::assembly::wall_fields {
Document StampDocument(const tl::fea::NodalStamp& s) {
    Document d;d.SetObject();Integer(d,"owner_id",s.owner_id);Integer(d,"epoch",s.epoch);
    Integer(d,"node_count",s.node_count);Number(d,"time",s.time);Number(d,"fixed_dt",s.fixed_dt);
    Boolean(d,"has_rotations",s.has_rotations);Boolean(d,"reactions_valid",s.reactions_valid);
    Integer(d,"reaction_base_epoch",s.reaction_base_epoch);Number(d,"reaction_time",s.reaction_time);
    String(d,"temporal_scheme","staggered_half_kick_start");
    String(d,"velocity_phase",s.epoch?"previous_midpoint":"collocated");
    Number(d,"velocity_time",s.velocity_time);Number(d,"reaction_kick_dt",s.reaction_kick_dt);
    Document groups;groups.SetObject();Integer(groups,"source_instance_id",s.rigid_groups.source_instance_id);
    Integer(groups,"group_count",s.rigid_groups.group_count);Integer(groups,"member_count",s.rigid_groups.member_count);
    Child(d,"rigid_groups",groups);return d;
}
} // namespace crash::output::assembly::wall_fields
