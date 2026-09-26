#include "Internal.h"
#include "lib_src/collision/radioss_type25/UnitConversions.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::mixed_starter::detail {
const tl::fea::NodalNodeDomain& OriginalDomain(const MainSource& source) noexcept {
    return source.mixed().initial().context().pre_correction().physical().source_domain().domain();
}
void CheckEmbedding(const MainSource& source, const DomainEmbedding& embedding) {
    const auto& physical = source.mixed().initial().context().pre_correction().physical();
    const auto& canonical = physical.shell_source().references().source().canonical().data();
    const auto input = source.startup_input();
    if (&canonical != &embedding.source().tied_source().canonical().data() ||
        !embedding.original().Matches(OriginalDomain(source)) ||
        embedding.original().node_count() != input.node_count ||
        embedding.domain().node_count() <= input.node_count ||
        embedding.domain().node_count()-input.node_count != embedding.suffix().size())
        Reject(Status::InvalidInput, "Combined Starter domain is not the authenticated original source prefix");
    if (input.coordinates != s::Coordinates::Native)
        Reject(Status::UnsupportedSource, "Combined Starter source requires explicit native working coordinates");
    tlfea::contact::radioss_type25::units_detail::Factors units;
    if (!tlfea::contact::radioss_type25::units_detail::Make(input.units, units))
        Reject(Status::InvalidInput, "Combined Starter native unit context is invalid");
}
std::size_t CombinedInput::capacity_bytes() const noexcept {
    return ids.capacity()*sizeof(std::uint64_t) + positions.capacity()*sizeof(double);
}
s::Input CombinedInput::Input(const MainSource& source) const noexcept {
    auto input = source.startup_input();
    input.node_source_ids = ids.data();
    input.node_count = ids.size();
    input.positions = {positions.data(), std::uint32_t(ids.size()), 3, 1};
    return input;
}
CombinedInput PackCombined(const MainSource& source, const DomainEmbedding& embedding, std::size_t capacity) {
    CheckEmbedding(source, embedding);
    const auto original = source.startup_input();
    const auto count = embedding.domain().node_count();
    CombinedInput result;
    result.ids.resize(count);
    result.positions.resize(3*count);
    if (result.capacity_bytes() > capacity)
        Reject(Status::ResourceLimit, "Actual combined Starter input capacity exceeds forecast");
    for (std::size_t i = 0; i < original.node_count; ++i) {
        const auto point = original.positions.at(std::uint32_t(i));
        result.ids[i] = original.node_source_ids[i];
        result.positions[3*i] = point.x;
        result.positions[3*i+1] = point.y;
        result.positions[3*i+2] = point.z;
        if (result.ids[i] != embedding.original().nodes()[i].source_id)
            Reject(Status::InvalidInput, "Native prefix node identity differs from actual original domain");
    }
    for (std::size_t i = original.node_count; i < count; ++i) {
        const auto& node = embedding.domain().nodes()[i];
        result.ids[i] = node.source_id;
        result.positions[3*i] = node.position.x/original.units.length_m;
        result.positions[3*i+1] = node.position.y/original.units.length_m;
        result.positions[3*i+2] = node.position.z/original.units.length_m;
        const double original_si[]{node.position.x, node.position.y, node.position.z};
        for (unsigned k = 0; k < 3; ++k)
            if (!std::isfinite(result.positions[3*i+k]) || (original_si[k] != 0 && result.positions[3*i+k] == 0))
                Reject(Status::InvalidInput, "Declared suffix is not representable in native working length");
    }
    return result;
}
s::NodePrefixExtension Prefix(const MainSource& source) noexcept {
    const auto input = source.startup_input();
    s::NodePrefixExtension prefix;
    prefix.node_source_ids = input.node_source_ids;
    prefix.positions = input.positions;
    prefix.node_count = input.node_count;
    prefix.coordinates = input.coordinates;
    prefix.units = input.units;
    return prefix;
}
}
