#include "Internal.h"
#include "JointRows.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
void CheckJoints(const VehiclePhysicalAttachments& source, const joints::VehicleJointModel& joints) {
    const auto& physical = source.physical();
    const auto& model = joints.model();
    const auto& original = joints.source().data();
    Require(joints.physical().SharesStorage(physical) && model.prepared() && model.domain() &&
        model.domain()->SharesStorage(physical.source_domain().domain()) && model.rigid_binding() &&
        model.rigid_binding()->groups().data() == physical.rigid_assembly().groups().data() &&
        model.rigid_binding()->members().data() == physical.rigid_assembly().members().data(),
        "Connectivity joints do not retain the exact physical/domain/rigid authority");
    Require(model.joints().size() == original.required && joints.source_rows().size() == original.required &&
        original.required <= original.rows.size() && original.boundaries == original.rows.size()-original.required,
        "Connectivity joint source coverage differs");
    std::size_t admitted = 0;
    for (std::size_t row = 0; row < original.rows.size(); ++row) {
        const auto& input = original.rows[row];
        if (input.disposition == modelio::type45::Disposition::OmittedAssemblyBoundary) continue;
        Require(input.disposition == modelio::type45::Disposition::Required && admitted < model.joints().size() &&
            joints.source_rows()[admitted] == row && input.property_index < original.properties.size(),
            "Connectivity joint required/source-row association differs");
        const auto& joint = model.joints()[admitted++];
        Require(joint.geometry.source_joint_id == input.source_id &&
            tl::fea::type45::detail::Same(joint.property,original.properties[input.property_index].value),
            "Connectivity joint source/property identity differs");
        JointKind(joint.property.kind);
        for (unsigned end = 0; end < 2; ++end) {
            Require(joint.domain_nodes[end] == input.nodes[end].domain_index &&
                joint.geometry.source_node_id[end] == input.nodes[end].source_id &&
                joint.body_groups[end] < physical.rigid_assembly().groups().size(),
                "Connectivity joint source endpoint/body association differs");
            const auto& body = physical.rigid_assembly().groups()[joint.body_groups[end]];
            const bool part = input.nodes[end].body.kind == modelio::type45::BodyKind::PartRoot;
            Require((part || input.nodes[end].body.kind == modelio::type45::BodyKind::PlainGroup) &&
                input.nodes[end].body.retained && body.source_id == input.nodes[end].body.source_id &&
                body.source_kind == (part ? tl::fea::RigidBindingSourceKind::Part :
                    tl::fea::RigidBindingSourceKind::NodalGroup),
                "Connectivity joint body source identity differs");
        }
    }
    Require(admitted == original.required, "Connectivity omitted a required joint operator");
}
void VisitJoints(const joints::VehicleJointModel& joints, Relations& output) {
    const auto rows = joints.model().joints();
    const auto domain = joints.model().domain()->nodes();
    for (std::size_t row = 0; row < rows.size(); ++row)
        AppendJoint(output,rows[row],joints.source_rows()[row],domain);
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
