#pragma once
#include "modelio/physical_scope/PhysicalScope.h"
#include "modelio/physical_scope/DomainEmbedding.h"
#include "lib_src/assembly/ElementMassContributions.h"

namespace crash::modelio::point_mass {
struct Limits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    tl::fea::ElementMassLimits native{4096, 524288, 32 * 1024 * 1024};
};
struct Disposition {
    std::size_t source_record = SIZE_MAX;
    // SIZE_MAX explicitly records an original card outside this domain.
    std::size_t domain_node = SIZE_MAX;
};
struct Forecast {
    std::size_t previous_phase = 0, retained_source = 0, decode_bytes = 0;
    std::size_t native_reservation = 0, current_phase = 0, total_bytes = 0;
};
// Once-only source ELEMENT_MASS contributions for a declared original domain.
// Every original card has a disposition. All retained nodes receive their real
// source mass, including cards outside rigid PARTs. This does not enlarge the
// domain, invent inertia or qualify omitted connections/mechanical closure.
class VehiclePointMassSource {
  public:
    static Forecast Preflight(const physical_scope::PhysicalScope&, const tl::fea::NodalNodeDomain&, Limits = {});
    static VehiclePointMassSource Prepare(const physical_scope::PhysicalScope&, const tl::fea::NodalNodeDomain&,
                                         Limits = {});
    // Explicit additive-domain path. The immutable certificate proves every
    // original node/card association; this producer adds no suffix mass itself.
    static Forecast PreflightEmbedded(const physical_scope::DomainEmbedding&, Limits = {});
    static VehiclePointMassSource PrepareEmbedded(const physical_scope::DomainEmbedding&, Limits = {});
    const physical_scope::DomainEmbedding* embedding() const noexcept;
    const physical_scope::PhysicalScope& source() const noexcept;
    const tl::fea::ElementMassContributions& contributions() const noexcept;
    const std::vector<Disposition>& dispositions() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    static VehiclePointMassSource PrepareChecked(const physical_scope::PhysicalScope&,
        const tl::fea::NodalNodeDomain&, Limits, Forecast, const physical_scope::DomainEmbedding*);
    struct Storage;
    explicit VehiclePointMassSource(std::shared_ptr<const Storage> data) : storage_(std::move(data)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::point_mass
