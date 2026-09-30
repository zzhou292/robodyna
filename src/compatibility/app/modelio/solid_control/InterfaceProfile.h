#pragma once
#include "Internal.h"
#include <set>

namespace crash::modelio::solid_control::detail {
// Explicit source-generator closure of the admitted ordinary original import.
// This is a supported feature census, never a PID/NID or observed interface map.
// Unknown generators reject instead of being presumed interface-neutral.
inline void CheckInterfaceKeyword(const std::string& keyword, InterfaceCensus& census,
    const std::string& member, std::size_t line) {
    ++census.checked_source_blocks;
    if (keyword == "*CONTACT_AUTOMATIC_SINGLE_SURFACE") { ++census.type25_sources; return; }
    if (keyword == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE") { ++census.type2_sources; return; }
    if (keyword == "*CONTACT_INTERIOR") { ++census.interior_sources; return; }
    if (keyword == "*RIGIDWALL_PLANAR" || keyword == "*RIGIDWALL_PLANAR_FINITE_ID" ||
        keyword == "*RIGIDWALL_PLANAR_FINITE_FORCES_ID") { ++census.rigid_wall_sources; return; }
    static const std::set<std::string> neutral{
        "*AIRBAG_SIMPLE_AIRBAG_MODEL_ID",
        "*CONSTRAINED_EXTRA_NODES_SET", "*CONSTRAINED_JOINT_CYLINDRICAL_ID",
        "*CONSTRAINED_JOINT_REVOLUTE_ID", "*CONSTRAINED_JOINT_SPHERICAL_ID",
        "*CONSTRAINED_NODAL_RIGID_BODY", "*CONSTRAINED_RIGID_BODIES", "*CONSTRAINED_SPOTWELD_ID",
        "*CONTROL_ACCURACY", "*CONTROL_CONTACT", "*CONTROL_CPU", "*CONTROL_ENERGY",
        "*CONTROL_HOURGLASS", "*CONTROL_OUTPUT", "*CONTROL_SHELL", "*CONTROL_SOLID",
        "*CONTROL_TERMINATION", "*CONTROL_TIMESTEP",
        "*DATABASE_ABSTAT", "*DATABASE_BINARY_D3PLOT", "*DATABASE_BINARY_D3THDT",
        "*DATABASE_BINARY_INTFOR", "*DATABASE_DEFORC", "*DATABASE_ELOUT", "*DATABASE_EXTENT_BINARY",
        "*DATABASE_GLSTAT", "*DATABASE_HISTORY_NODE_ID", "*DATABASE_HISTORY_NODE_SET_LOCAL",
        "*DATABASE_JNTFORC", "*DATABASE_MATSUM", "*DATABASE_NODOUT", "*DATABASE_RCFORC",
        "*DATABASE_RWFORC", "*DATABASE_SECFORC", "*DATABASE_SLEOUT",
        "*DEFINE_COORDINATE_NODES", "*DEFINE_CURVE", "*DEFINE_TRANSFORMATION",
        "*ELEMENT_BEAM", "*ELEMENT_DISCRETE", "*ELEMENT_MASS", "*ELEMENT_MASS_PART",
        "*ELEMENT_SEATBELT_ACCELEROMETER", "*ELEMENT_SHELL", "*ELEMENT_SOLID",
        "*END", "*HOURGLASS", "*INCLUDE", "*INCLUDE_TRANSFORM", "*INITIAL_VELOCITY_GENERATION",
        "*KEYWORD", "*LOAD_BODY_Z", "*MAT_BLATZ-KO_RUBBER", "*MAT_DAMPER_VISCOUS",
        "*MAT_ELASTIC", "*MAT_LOW_DENSITY_FOAM", "*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY",
        "*MAT_PIECEWISE_LINEAR_PLASTICITY", "*MAT_RIGID", "*MAT_SPOTWELD",
        "*MAT_SPRING_ELASTIC", "*MAT_SPRING_NONLINEAR_ELASTIC", "*NODE", "*PARAMETER",
        "*PARAMETER_EXPRESSION", "*PART", "*SECTION_BEAM", "*SECTION_DISCRETE",
        "*SECTION_SHELL", "*SECTION_SOLID", "*SET_NODE_ADD", "*SET_NODE_GENERAL",
        "*SET_NODE_LIST", "*SET_NODE_LIST_TITLE", "*SET_PART_ADD", "*SET_PART_LIST_TITLE",
        "*SET_SEGMENT", "*TITLE"
    };
    if (!neutral.count(keyword))
        Reject(Status::UnsupportedSource, "Unreviewed source generator prevents complete TYPE24 disposition", member, line);
}
} // namespace crash::modelio::solid_control::detail
