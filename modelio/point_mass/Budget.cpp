#include "VehiclePointMassSource.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <algorithm>

namespace crash::modelio::point_mass {
Forecast VehiclePointMassSource::Preflight(const physical_scope::PhysicalScope& source,
    const tl::fea::NodalNodeDomain& domain, Limits limits) {
    const Limits hard;
    const auto& native = limits.native;
    output::Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes && native.max_records &&
        native.max_records <= hard.native.max_records && native.max_nodes && native.max_nodes <= hard.native.max_nodes &&
        native.max_host_bytes && native.max_host_bytes <= hard.native.max_host_bytes,
        "Invalid declared-domain point-mass limits");
    const auto& rows = source.point_mass_source().data().records;
    output::Require(rows.size() == 155 && rows.size() <= native.max_records && domain.prepared() &&
        domain.source_instance_id() == source.point_mass_source().rigid_source().topology().source_instance_id() &&
        domain.node_count() >= source.data().counts.with_type25_nodes && domain.node_count() <= native.max_nodes,
        "Point-mass original source extent or physical domain changed");
    Forecast result;
    result.previous_phase = source.forecast().total_bytes;
    tl::util::BoundedArenaLayout retained(limits.host_bytes), decode(limits.host_bytes), phase(limits.host_bytes);
    tl::util::ArenaRegion ignored;
    output::Require(retained.Append<unsigned char>(source.forecast().source_reservation, ignored) &&
        retained.Append<unsigned char>(source.forecast().additional_retained, ignored) &&
        retained.Append<unsigned char>(source.data().owned_payload_bytes, ignored),
        "Retained point-mass source exceeds cap");
    result.retained_source = retained.bytes();
    // Both decoded arrays and their decoder scratch coexist conservatively.
    const auto count = source.tied_source().canonical().data().canonical_nodes;
    output::Require(decode.Append<std::uint64_t>(count, ignored) && decode.Append<std::uint64_t>(count, ignored) &&
        decode.Append<std::array<double, 3>>(count, ignored) && decode.Append<std::array<double, 3>>(count, ignored),
        "Point-mass canonical coordinate scratch exceeds cap");
    result.decode_bytes = decode.bytes();
    result.native_reservation = native.max_host_bytes;
    output::Require(phase.Append<unsigned char>(sizeof(VehiclePointMassSource) + 1024, ignored) &&
        phase.Append<unsigned char>(result.retained_source, ignored) &&
        phase.Append<unsigned char>(domain.owned_payload_bytes(), ignored) &&
        phase.Append<unsigned char>(result.decode_bytes, ignored) &&
        phase.Append<unsigned char>(result.native_reservation, ignored) &&
        phase.Append<Disposition>(rows.size(), ignored) &&
        phase.Append<tl::fea::ElementMassSource>(rows.size(), ignored),
        "Complete declared-domain point-mass phase exceeds cap");
    result.current_phase = phase.bytes();
    result.total_bytes = std::max(result.previous_phase, result.current_phase);
    output::Require(result.total_bytes <= limits.host_bytes, "Earlier physical-source phase exceeds point-mass cap");
    return result;
}
Forecast VehiclePointMassSource::PreflightEmbedded(const physical_scope::DomainEmbedding& embedding,
    Limits limits) {
    return PreflightEmbedded(embedding.source(),embedding.domain(),embedding.forecast(),limits);
}
Forecast VehiclePointMassSource::PreflightEmbedded(const physical_scope::PhysicalScope& source,
    const tl::fea::NodalNodeDomain& domain,const physical_scope::DomainEmbeddingForecast& embedding,
    Limits limits) {
    auto f = Preflight(source, domain, limits);
    output::Require(embedding.object>=sizeof(physical_scope::DomainEmbedding)&&embedding.original_domain&&
        embedding.source==source.forecast().total_bytes&&embedding.complete_domain==domain.owned_payload_bytes(),
        "Embedded point-mass forecast differs from its actual input shape");
    output::Require(embedding.object<=SIZE_MAX-embedding.original_domain,"Embedded source forecast overflow");
    const auto extra = embedding.object+embedding.original_domain;
    output::Require(f.current_phase <= limits.host_bytes && extra <= limits.host_bytes - f.current_phase,
        "Embedded point-mass source and original-domain authority exceed cap");
    f.current_phase += extra;
    // The prepared embedding owns its separately admitted earlier construction
    // peak. Its retired decode scratch is not charged to this current local cap.
    // Retained original-domain/certificate backing IS charged above. The outer
    // composition admits the reported whole-chain maximum before either stage.
    f.previous_phase = std::max(f.previous_phase, embedding.peak_bytes);
    f.total_bytes = std::max(f.previous_phase, f.current_phase);
    return f;
}
} // namespace crash::modelio::point_mass
