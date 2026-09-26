#include "Internal.h"
#include <algorithm>

namespace crash::modelio::type25::detail {
void Add(std::size_t& bytes, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && bytes <= cap && count <= (cap - bytes) / width, "Vehicle TYPE25 source byte cap exceeded");
    bytes += count * width;
}
}
namespace crash::modelio::type25 {
Forecast VehicleType25Source::Preflight(const physical_scope::PhysicalScope& source,
    const tl::fea::NodalNodeDomain& domain, Declaration declaration, Limits limits) {
    using namespace detail;
    const Limits hard;
    const auto& model_limits = limits.model;
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
        model_limits.profile == native::CapacityProfile::Vehicle && model_limits.max_connections &&
        model_limits.max_connections <= hard.model.max_connections && model_limits.max_properties &&
        model_limits.max_properties <= hard.model.max_properties && model_limits.max_nodes &&
        model_limits.max_nodes <= hard.model.max_nodes && model_limits.max_host_bytes &&
        model_limits.max_host_bytes <= hard.model.max_host_bytes,
        "Invalid vehicle TYPE25 source or native model limits");
    const auto& canonical = source.tied_source().canonical().data();
    const auto& data = source.data();
    Require(declaration.policy == Policy::OriginalDefaultSpotweldsV1 && declaration.generated_property_id &&
        data.spotwelds.size() == 2828 && data.counts.optional_spotwelds == 0,
        "Vehicle TYPE25 requires all original default-only spotweld cards");
    for (const auto& part : canonical.parts)
        Require(part.section != declaration.generated_property_id,
                "Generated TYPE25 property overlaps an original section identity");
    Require(domain.prepared() && domain.source_instance_id() ==
        source.point_mass_source().rigid_source().topology().source_instance_id() &&
        domain.node_count() >= data.counts.with_type25_nodes && domain.node_count() <= model_limits.max_nodes &&
        data.spotwelds.size() <= model_limits.max_connections,
        "Vehicle TYPE25 common-domain identity or native capacity changed");
    Forecast result;
    result.previous_phase = source.forecast().total_bytes;
    result.retained_source_bound = source.forecast().source_reservation;
    Add(result.retained_source_bound, source.forecast().additional_retained, 1, limits.host_bytes);
    Add(result.retained_source_bound, data.owned_payload_bytes, 1, limits.host_bytes);
    result.domain_payload = domain.owned_payload_bytes();
    // ID and position decodes coexist. Each decoder's native bytes are a
    // transient copy; two complete arrays conservatively bound the peak.
    Add(result.decode_bytes, canonical.canonical_nodes, 2 * (sizeof(std::uint64_t) + 3 * sizeof(double)), limits.host_bytes);
    Add(result.connection_bytes, data.spotwelds.size(), sizeof(native::ConnectionInput), limits.host_bytes);
    result.native_reservation = model_limits.max_host_bytes;
    result.current_phase = sizeof(VehicleType25Source) + sizeof(native::Model) + 1024;
    for (auto bytes : {result.retained_source_bound, result.domain_payload, result.decode_bytes,
                      result.connection_bytes, result.native_reservation})
        Add(result.current_phase, bytes, 1, limits.host_bytes);
    result.total_bytes = std::max(result.previous_phase, result.current_phase);
    Require(result.total_bytes <= limits.host_bytes, "Complete vehicle TYPE25 source phase exceeds cap");
    return result;
}
Forecast VehicleType25Source::PreflightEmbedded(const physical_scope::DomainEmbedding& embedding,
    Declaration declaration, Limits limits) {
    return PreflightEmbedded(embedding.source(),embedding.domain(),embedding.forecast(),declaration,limits);
}
Forecast VehicleType25Source::PreflightEmbedded(const physical_scope::PhysicalScope& source,
    const tl::fea::NodalNodeDomain& domain,const physical_scope::DomainEmbeddingForecast& embedding,
    Declaration declaration,Limits limits) {
    auto f = Preflight(source, domain, declaration, limits);
    detail::Require(embedding.object>=sizeof(physical_scope::DomainEmbedding)&&embedding.original_domain&&
        embedding.source==source.forecast().total_bytes&&embedding.complete_domain==domain.owned_payload_bytes(),
        "Embedded TYPE25 forecast differs from its actual input shape");
    detail::Add(f.current_phase, embedding.object, 1, limits.host_bytes);
    detail::Add(f.current_phase, embedding.original_domain, 1, limits.host_bytes);
    // Current retained backing remains inside the existing local cap; the
    // separately admitted prior embedding peak is reported, not reallocated.
    f.previous_phase = std::max(f.previous_phase, embedding.peak_bytes);
    f.total_bytes = std::max(f.previous_phase, f.current_phase);
    return f;
}
} // namespace crash::modelio::type25
