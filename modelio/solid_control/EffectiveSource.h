#pragma once
#include "DirectSource.h"
namespace crash::modelio::solid_control {
struct EffectiveData {
    std::vector<PartControl> parts;
    std::vector<PartOrigin> origins; // Same sorted PID order as parts.
    InterfaceCensus interfaces;
    std::string source_digest;
    bool shared_canonical_backing = true;
    // Additional direct canonical backing beyond the supplied import context;
    // not included in own payload bytes. Consumers account shared inputs once.
    std::size_t additional_backing_reservation_bytes = 0;
};
struct Preparation;
// Immutable effective property authority. No physical model, material slots,
// nodal coefficients, controlled force history or solver clock is owned here.
class EffectiveSource {
  public:
    static Preparation Prepare(const DirectSource&, const ids::ImportContext&,
                               const ids::ImportMembers&, Limits = {});
    const DirectSource& direct() const noexcept;
    const EffectiveData& data() const noexcept;
    std::size_t owned_payload_bytes() const noexcept;
  private:
    struct Data;
    explicit EffectiveSource(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
struct Preparation {
    Report report;
    std::optional<EffectiveSource> source;
};
} // namespace crash::modelio::solid_control
