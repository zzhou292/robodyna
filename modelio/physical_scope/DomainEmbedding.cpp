#include "DomainEmbedding.h"
#include "output/ArtifactIO.h"
#include "output/BoundedArrayIO.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <cstdint>
#include <functional>

namespace crash::modelio::physical_scope {
using output::Require;
struct DomainEmbedding::Data {
    Data(const PhysicalScope& s, const tl::fea::NodalNodeDomain& a,
         const tl::fea::NodalNodeDomain& b, DomainEmbeddingForecast f)
        : source(s), original(a), domain(b), forecast(f) {}
    PhysicalScope source;
    tl::fea::NodalNodeDomain original, domain;
    DomainEmbeddingForecast forecast;
};
namespace {
void Shape(const tl::fea::NodalNodeDomain& original, const tl::fea::NodalNodeDomain& complete,
           tl::util::ConstView<tl::fea::NodalDomainNode> suffix, DomainEmbeddingLimits limits) {
    const DomainEmbeddingLimits hard;
    const auto nodes = tl::fea::NodalDomainLimits::Vehicle().max_nodes;
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
        limits.added_nodes && limits.added_nodes <= hard.added_nodes,
        "Invalid preserved-domain embedding limits");
    Require(original.prepared() && complete.prepared() && original.node_count() &&
        original.node_count() <= nodes && suffix.size() && suffix.size() <= limits.added_nodes &&
        suffix.size() <= nodes - original.node_count() &&
        complete.node_count() == original.node_count() + suffix.size() &&
        original.source_instance_id() == complete.source_instance_id(),
        "Embedding must preserve one original domain and an explicit bounded suffix");
    const auto address = reinterpret_cast<std::uintptr_t>(suffix.data());
    Require(address && address % alignof(tl::fea::NodalDomainNode) == 0 &&
        suffix.size() <= (UINTPTR_MAX - address) / sizeof(tl::fea::NodalDomainNode),
        "Invalid declared domain-suffix span");
}
bool Same(const tl::fea::NodalDomainNode& a, const tl::fea::NodalDomainNode& b) {
    return a.source_id == b.source_id && output::Bits(a.position.x) == output::Bits(b.position.x) &&
        output::Bits(a.position.y) == output::Bits(b.position.y) &&
        output::Bits(a.position.z) == output::Bits(b.position.z);
}
}
void detail::CheckEmbedding(const std::vector<std::uint64_t>& canonical_ids,
        const tl::fea::NodalNodeDomain& original, const tl::fea::NodalNodeDomain& complete,
        tl::util::ConstView<tl::fea::NodalDomainNode> suffix) {
    Shape(original, complete, suffix, {});
    Require(!canonical_ids.empty() && canonical_ids.front() &&
        std::adjacent_find(canonical_ids.begin(), canonical_ids.end(),
            std::greater_equal<std::uint64_t>()) == canonical_ids.end(),
        "Embedding requires complete unique canonical NODE identities");
    for (std::size_t i = 0; i < original.node_count(); ++i)
        Require(Same(original.nodes()[i], complete.nodes()[i]),
            "Combined domain changes an original NID, order or coordinate bit");
    for (std::size_t i = 0; i < suffix.size(); ++i) {
        Require(Same(suffix[i], complete.nodes()[original.node_count() + i]),
            "Combined domain differs from the explicit new source suffix");
        Require(!std::binary_search(canonical_ids.begin(), canonical_ids.end(), suffix[i].source_id),
            "New source suffix collides with an original canonical NODE identity");
    }
    // NodalNodeDomain::Initialize has already proved global source-ID uniqueness;
    // the exact suffix check above binds that proof to the declared input rows.
}
DomainEmbeddingForecast DomainEmbedding::Preflight(const PhysicalScope& source,
        const tl::fea::NodalNodeDomain& original, const tl::fea::NodalNodeDomain& complete,
        tl::util::ConstView<tl::fea::NodalDomainNode> suffix, DomainEmbeddingLimits limits) {
    Shape(original, complete, suffix, limits);
    const auto& canonical = source.tied_source().canonical().data();
    DomainEmbeddingForecast f;
    f.source = source.forecast().total_bytes;
    f.original_domain = original.owned_payload_bytes();
    f.complete_domain = complete.owned_payload_bytes();
    f.object = sizeof(DomainEmbedding) + sizeof(Data) + 64;
    tl::util::BoundedArenaLayout decode(limits.host_bytes), all(limits.host_bytes);
    tl::util::ArenaRegion ignored;
    Require(decode.Append<std::uint64_t>(canonical.canonical_nodes, ignored) &&
        decode.Append<std::uint64_t>(canonical.canonical_nodes, ignored) &&
        decode.Append<std::array<double, 3>>(canonical.canonical_nodes, ignored) &&
        decode.Append<std::array<double, 3>>(canonical.canonical_nodes, ignored),
        "Embedding canonical validation scratch exceeds cap");
    f.decode = decode.bytes();
    for (const auto bytes : {f.source, f.original_domain, f.complete_domain, f.object, f.decode})
        Require(all.Append<std::byte>(bytes, ignored), "Complete domain-embedding forecast exceeds cap");
    f.peak_bytes = all.bytes();
    return f;
}
DomainEmbedding DomainEmbedding::Prepare(const PhysicalScope& source,
        const tl::fea::NodalNodeDomain& original, const tl::fea::NodalNodeDomain& complete,
        tl::util::ConstView<tl::fea::NodalDomainNode> suffix, DomainEmbeddingLimits limits) {
    const auto forecast = Preflight(source, original, complete, suffix, limits);
    const auto& canonical = source.tied_source().canonical().data();
    const auto& id_array = physical_scope::source::FindArray(canonical, "node_ids");
    const auto& position_array = physical_scope::source::FindArray(canonical, "node_positions");
    const auto ids = output::arrays::Decode<std::uint64_t>(id_array.descriptor, id_array.bytes);
    const auto positions = output::arrays::Decode<double>(position_array.descriptor, position_array.bytes);
    detail::CheckDomain(ids, positions, source.data().node_roles, original,
        source.point_mass_source().rigid_source().topology().source_instance_id(),
        source.data().counts.with_type25_nodes);
    detail::CheckEmbedding(ids, original, complete, suffix);
    return DomainEmbedding(std::make_shared<Data>(source, original, complete, forecast));
}
const PhysicalScope& DomainEmbedding::source() const noexcept { return data_->source; }
const tl::fea::NodalNodeDomain& DomainEmbedding::original() const noexcept { return data_->original; }
const tl::fea::NodalNodeDomain& DomainEmbedding::domain() const noexcept { return data_->domain; }
tl::util::ConstView<tl::fea::NodalDomainNode> DomainEmbedding::suffix() const noexcept {
    const auto first = data_->original.node_count();
    return {data_->domain.nodes().data() + first, data_->domain.node_count() - first};
}
const DomainEmbeddingForecast& DomainEmbedding::forecast() const noexcept { return data_->forecast; }
std::size_t DomainEmbedding::incremental_backing_bytes() const noexcept {
    return data_->forecast.object + data_->forecast.original_domain;
}
} // namespace crash::modelio::physical_scope
