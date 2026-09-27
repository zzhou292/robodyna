#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
std::vector<PartControl> ResolveSharing(const std::vector<Part>& parts,
    const std::vector<Section>& sections, const std::vector<std::uint64_t>& requested) {
    try { return controls::detail::ResolveSharing(parts, sections, requested); }
    catch (const controls::detail::Failure& failure) { RejectControl(failure.report); }
}
}
