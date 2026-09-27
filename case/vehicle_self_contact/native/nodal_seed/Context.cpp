#include "Internal.h"
#include "modelio/solid_control/Members.h"
#include "modelio/self_contact/OriginalSelection.h"
#include <map>
namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
InteriorPrepared ReadInterior(const PhysicalModel& model, const ids::ImportMembers& members, Limits limits) {
    const auto& canonical = model.shell_source().references().source().canonical();
    const auto combine = modelio::solid_control::detail::ReadMember(members, "combine.key", 64u << 10);
    const auto auxiliary = modelio::solid_control::detail::ReadMember(members, "set-yaris-coarse-v1l.key", 1u << 20);
    const auto selected = modelio::self_contact::OriginalSelection::Prepare(canonical, auxiliary, combine);
    Require(selected.data().source_fields.soft && *selected.data().source_fields.soft == 1.,
        "Contact seed requires its authenticated original selected contact profile");
    auto declarations = modelio::solid_control::DirectSource::Prepare(canonical, members,
        {limits.metadata_bytes, modelio::solid_control::DirectLimits{}.retained_bytes});
    InteriorDisposition result;
    result.combine_sha256 = declarations.data().combine_sha256;
    result.auxiliary_sha256 = declarations.data().auxiliary_sha256;
    std::map<std::uint64_t, std::size_t> retained;
    for (const auto& row : model.source_domain().source().solid_source().data().rows) ++retained[row.part_id];
    result.parts.reserve(declarations.data().parts.size());
    for (const auto& part : declarations.data().parts) {
        result.parts.push_back({part.part_id, part.section_id, part.material_id, retained[part.part_id], part.original_solids});
        result.original_solids += part.original_solids;
        result.retained_solids += retained[part.part_id];
    }
    return {std::move(declarations), std::move(result)};
}
}
