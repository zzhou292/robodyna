#include "Internal.h"

namespace crash::modelio::solid_source {
struct VehicleSolidSource::Storage {
    explicit Storage(const source::CanonicalSource& value) : canonical(value) {}
    source::CanonicalSource canonical;
    Data data;
    Forecast forecast;
};
Forecast VehicleSolidSource::Preflight(const source::CanonicalSource& source, Policy policy, Limits limits) {
    return detail::Budget(source.data(), policy, limits);
}
VehicleSolidSource VehicleSolidSource::Prepare(const source::CanonicalSource& source,
        const std::string& member, Policy policy, Limits limits) {
    const auto forecast = Preflight(source, policy, limits);
    const auto& canonical = source.data();
    output::Require(member.size() == canonical.inputs.source_member.bytes &&
                    output::Sha256(member) == canonical.inputs.source_member.sha256,
                    "Solid original member authentication failed");
    auto next = std::make_shared<Storage>(source);
    next->forecast = forecast;
    next->data.policy = policy;
    detail::ReadDeclarations(canonical, member, next->data, limits);
    detail::ReadGeometry(canonical, member, next->data, limits);
    detail::PrepareReferences(canonical, next->data, limits);
    const auto& data = next->data;
    const auto census = detail::ExpectedCensus(policy);
    output::Require(data.parts.size() == census.parts && data.rows.size() == census.parents &&
                    data.solid18.size() == census.solid18 && data.solid24.size() == census.solid24 &&
                    data.solid6z.size() == census.solid6z &&
                    data.solid18_law44.size() == census.solid18_law44 &&
                    data.solid18_law90.size() == census.solid18_law90 && data.original_solids == 15234 &&
                    data.outside_solids == 15234 - census.parents,
                    "Original retained solid census changed");
    next->data.owned_payload_bytes = detail::OwnedPayload(next->data, limits);
    return VehicleSolidSource(std::move(next));
}
const source::CanonicalSource& VehicleSolidSource::canonical() const noexcept { return storage_->canonical; }
const Data& VehicleSolidSource::data() const noexcept { return storage_->data; }
const Forecast& VehicleSolidSource::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::modelio::solid_source
