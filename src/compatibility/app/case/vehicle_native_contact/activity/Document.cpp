#include "Declaration.h"
namespace crash::cases::vehicle_native_contact::activity {
output::Document Document(const Declaration& declaration) {
    output::Document d;d.SetObject();output::String(d,"schema","robo_dyna.native_v6_activity_source.v1");
    output::String(d,"scope","Executed selected V6 model; original omitted elements are not silently counted as active support");
    const auto& c=declaration.coverage();output::String(d,"executed_profile",c.executed_profile);
    output::String(d,"canonical_sha256",c.canonical_sha256);output::String(d,"scope_sha256",c.scope_sha256);
    output::String(d,"source_member_sha256",c.source_member_sha256);
    output::Integer(d,"physical_nodes",c.physical_nodes);output::Integer(d,"wall_shells",c.wall_shells);
    output::Integer(d,"type13",c.type13);output::Integer(d,"beam18",c.beam18);output::Integer(d,"welds",c.welds);output::Integer(d,"joints",c.joints);
    const auto population=[&](const char* name,const Population& value) {
        output::Document row;row.SetObject();output::Integer(row,"original",value.original);output::Integer(row,"executed",value.executed);
        output::Integer(row,"excluded",value.excluded);output::Integer(row,"excluded_touching_owner",value.excluded_touching_owner);
        output::String(row,"executed_ids_sha256",value.executed_ids_sha256);output::String(row,"excluded_ids_sha256",value.excluded_ids_sha256);
        output::Value copy;copy.CopyFrom(row,d.GetAllocator());d.AddMember(output::Value(name,d.GetAllocator()),copy,d.GetAllocator());
    };
    population("shells",c.shells);population("beams",c.beams);population("solids",c.solids);
    output::Integer(d,"pre_shell_internal_count",c.pre_shell_internal);
    output::String(d,"incoming_solid_erosion",c.incoming_solid_erosion==tlfea::contact::radioss_type25::startup::SolidErosion::Enabled?"enabled":"disabled");
    output::String(d,"final_solid_erosion",c.final_solid_erosion==tlfea::contact::radioss_type25::startup::SolidErosion::Enabled?"enabled":"disabled");
    output::Integer(d,"self_idel",static_cast<unsigned>(declaration.self().deletion));
    output::Integer(d,"wall_idel",static_cast<unsigned>(declaration.wall().deletion));
    output::Boolean(d,"self_keep_disconnected_nodes",declaration.self().keep_disconnected_nodes);
    output::String(d,"mass_policy","No mass deletion; bare point mass and rigid/CIN membership are not element support");
    output::Integer(d,"metadata_bytes",declaration.forecast().owned_metadata_bytes);
    output::Integer(d,"construction_workspace_bytes",declaration.forecast().construction_workspace_bytes);return d;
}
}
