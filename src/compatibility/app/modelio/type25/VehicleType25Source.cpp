#include "Internal.h"
#include <optional>
#include "output/BoundedArrayIO.h"

namespace crash::modelio::type25 {
struct VehicleType25Source::Storage {
    Storage(const physical_scope::PhysicalScope& source, const tl::fea::NodalNodeDomain& domain,
            Declaration declaration) : source(source), domain(domain), declaration(declaration) {}
    physical_scope::PhysicalScope source;
    std::optional<physical_scope::DomainEmbedding> embedding;
    tl::fea::NodalNodeDomain domain;
    Declaration declaration;
    native::Model model;
    Forecast forecast;
};
VehicleType25Source VehicleType25Source::Prepare(const physical_scope::PhysicalScope& source,
    const tl::fea::NodalNodeDomain& domain, Declaration declaration, Limits limits) {
    const auto forecast = Preflight(source, domain, declaration, limits);
    return PrepareChecked(source, domain, declaration, limits, forecast, nullptr);
}
VehicleType25Source VehicleType25Source::PrepareEmbedded(const physical_scope::DomainEmbedding& embedding,
    Declaration declaration, Limits limits) {
    const auto forecast = PreflightEmbedded(embedding, declaration, limits);
    return PrepareChecked(embedding.source(), embedding.domain(), declaration, limits, forecast, &embedding);
}
VehicleType25Source VehicleType25Source::PrepareChecked(const physical_scope::PhysicalScope& source,
    const tl::fea::NodalNodeDomain& domain, Declaration declaration, Limits limits,
    Forecast forecast, const physical_scope::DomainEmbedding* embedding) {
    auto next = std::make_shared<Storage>(source, domain, declaration);
    if (embedding) next->embedding.emplace(*embedding);
    else physical_scope::ValidateDeclaredDomain(source, domain);
    const auto connections = detail::Pack(source.data().spotwelds, domain);
    const native::PropertyInput property{declaration.generated_property_id, assembly::ResolvedSpotweldProperty()};
    native::ModelInput input;
    input.source_instance_id = domain.source_instance_id();
    input.global_node_count = domain.node_count();
    input.properties = &property;
    input.property_count = 1;
    input.connections = connections.data();
    input.connection_count = connections.size();
    input.source_units = assembly::SpotweldSourceUnits;
    input.limits = limits.model;
    const auto report = next->model.Initialize(input);
    if (!report) {
        const auto id = report.connection < connections.size() ? connections[report.connection].source_element_id : 0;
        throw std::runtime_error("Original TYPE25 startup failed at WID " + std::to_string(id) + ": " + report.message);
    }
    detail::Require(next->model.startup_payload_bytes() <= forecast.native_reservation,
                    "Vehicle TYPE25 native startup exceeded reservation");
    next->forecast = forecast;
    return VehicleType25Source(std::move(next));
}
const physical_scope::DomainEmbedding* VehicleType25Source::embedding() const noexcept {
    return storage_->embedding ? &*storage_->embedding : nullptr;
}
const physical_scope::PhysicalScope& VehicleType25Source::source() const noexcept { return storage_->source; }
const tl::fea::NodalNodeDomain& VehicleType25Source::domain() const noexcept { return storage_->domain; }
const native::Model& VehicleType25Source::model() const noexcept { return storage_->model; }
Declaration VehicleType25Source::declaration() const noexcept { return storage_->declaration; }
const Forecast& VehicleType25Source::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::modelio::type25
