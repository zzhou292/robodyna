#pragma once
#include "../EnvelopePhysicalSource.h"
#include "case/vehicle_startup/physical_model/Components.h"
#include "case/vehicle_startup/shell_binding/Internal.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_wall::native::physical_detail {
namespace fe = tl::fea;
namespace model = vehicle_startup::physical_model;
namespace shell = vehicle_startup::shell_binding_detail;
using output::Require;
inline tl::util::ConstView<fe::NodalDomainNode> Suffix(const WallSource& wall) {
    const auto first = wall.vehicle_prefix().nodes;
    return {wall.domain().nodes().data() + first, wall.domain().node_count() - first};
}
void Check(const WallSource&, const vehicle_startup::VehicleShellReferences&, EnvelopePhysicalLimits);
fe::ShellQephBindingInput EnvironmentInput(const fe::qeph::ReferenceInput&,
    std::uint64_t source_parent, std::size_t original_shell_nodes);
shell::Inputs Pack(const WallSource&, const vehicle_startup::VehicleShellReferences&);
}
