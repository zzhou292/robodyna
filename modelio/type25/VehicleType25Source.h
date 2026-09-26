#pragma once
#include "modelio/physical_scope/PhysicalScope.h"
#include "modelio/physical_scope/DomainEmbedding.h"
#include "modelio/source_assembly/SourceAssemblySpotweldInput.h"
#include "lib_src/assembly/NodalNodeDomain.h"
#include "lib_src/elements/type25/Type25Model.h"

namespace crash::modelio::type25 {
namespace native = tl::fea::type25;
enum class Policy { OriginalDefaultSpotweldsV1 };
struct Declaration {
    Policy policy;
    // Explicit generated property identity, distinct from every source section.
    std::uint64_t generated_property_id;
};
struct Limits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    native::ModelLimits model = native::ModelLimits::Vehicle();
};
struct Forecast {
    std::size_t previous_phase = 0, retained_source_bound = 0, domain_payload = 0;
    std::size_t decode_bytes = 0, connection_bytes = 0, native_reservation = 0;
    std::size_t current_phase = 0, total_bytes = 0;
};
// Complete original source-backed TYPE25 model on a supplied common domain.
// No coefficient accumulation, rigid/DOF admission, force owner or clock.
class VehicleType25Source {
  public:
    static Forecast Preflight(const physical_scope::PhysicalScope&, const tl::fea::NodalNodeDomain&,
                              Declaration, Limits = {});
    static VehicleType25Source Prepare(const physical_scope::PhysicalScope&, const tl::fea::NodalNodeDomain&,
                                      Declaration, Limits = {});
    // Explicit additive-domain path; only authentic original spotwelds are
    // mapped. The suffix receives no invented connector or coefficient source.
    static Forecast PreflightEmbedded(const physical_scope::DomainEmbedding&, Declaration, Limits = {});
    static VehicleType25Source PrepareEmbedded(const physical_scope::DomainEmbedding&, Declaration, Limits = {});
    const physical_scope::DomainEmbedding* embedding() const noexcept;
    const physical_scope::PhysicalScope& source() const noexcept;
    const tl::fea::NodalNodeDomain& domain() const noexcept;
    const native::Model& model() const noexcept;
    Declaration declaration() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    static VehicleType25Source PrepareChecked(const physical_scope::PhysicalScope&,
        const tl::fea::NodalNodeDomain&, Declaration, Limits, Forecast,
        const physical_scope::DomainEmbedding*);
    struct Storage;
    explicit VehicleType25Source(std::shared_ptr<const Storage> storage) : storage_(std::move(storage)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::type25
