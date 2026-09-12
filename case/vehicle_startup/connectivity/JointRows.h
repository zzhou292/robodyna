#pragma once
#include "Relations.h"
#include "lib_src/elements/type45/Model.h"
#include "lib_src/elements/type45/Type45Checks.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
inline Kind JointKind(tl::fea::type45::Kind kind) {
    using Native = tl::fea::type45::Kind;
    switch (kind) {
        case Native::Spherical: return Kind::SphericalJoint;
        case Native::Revolute: return Kind::RevoluteJoint;
        case Native::Cylindrical: return Kind::CylindricalJoint;
    }
    throw std::runtime_error("Connectivity joint kind is unavailable");
}
inline void AppendJoint(Relations& output, const tl::fea::type45::Joint& joint,
                        std::size_t source_row,
                        tl::util::ConstView<tl::fea::NodalDomainNode> domain) {
    const auto kind = JointKind(joint.property.kind);
    namespace native = tl::fea::type45::detail;
    for (unsigned dof = 0; dof < 6; ++dof) {
        output::Require(native::Get(joint.property.free_stiffness,dof) == 0 &&
            native::Get(joint.property.free_viscosity,dof) == 0 &&
            bool(ReleasedDofs(kind) & (1u << dof)) == !native::Blocked(joint.property.kind,dof),
            "Connectivity released joint DOFs differ from the admitted native profile");
    }
    for (unsigned end = 0; end < 2; ++end)
        output::Require(joint.domain_nodes[end] < domain.size() &&
            domain[joint.domain_nodes[end]].source_id == joint.geometry.source_node_id[end],
            "Connectivity joint endpoint source/domain identity differs");
    output::Require(joint.domain_nodes[0] != joint.domain_nodes[1],
                    "Connectivity joint endpoints coincide in the source domain");
    output.Append(kind,Role::Constraint,joint.geometry.source_joint_id,0,source_row,joint.domain_nodes,2);
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
