#include "Internal.h"

namespace crash::modelio::self_contact {

struct OriginalSelection::Storage {
    explicit Storage(const source::CanonicalSource& value)
        : canonical(value) {}
    source::CanonicalSource canonical;
    Data data;
};

OriginalSelection OriginalSelection::Prepare(
    const source::CanonicalSource& canonical,
    const std::string& auxiliary_member,
    const std::string& combine_member, Limits limits) {
    const auto budget = detail::Preflight(canonical.data(),
        auxiliary_member.size(), combine_member.size(), limits);
    detail::Draft draft;
    detail::ReadSources(canonical.data(), auxiliary_member,
        combine_member, draft, limits);
    detail::ResolveCards(draft, limits);
    detail::BuildCensus(canonical.data(), draft, limits);
    draft.data.startup_budget_bytes = budget;
    draft.data.owned_payload_bytes =
        detail::OwnedPayload(draft.data, limits.host_bytes);
    auto storage = std::make_shared<Storage>(canonical);
    storage->data = std::move(draft.data);
    return OriginalSelection(std::move(storage));
}

std::size_t OriginalSelection::Forecast(
    const source::CanonicalSource& canonical,
    std::size_t auxiliary_member_bytes,
    std::size_t combine_member_bytes, Limits limits) {
    return detail::Preflight(canonical.data(), auxiliary_member_bytes,
        combine_member_bytes, limits);
}

const source::CanonicalSource&
OriginalSelection::canonical() const noexcept {
    return storage_->canonical;
}

const Data& OriginalSelection::data() const noexcept {
    return storage_->data;
}

}  // namespace crash::modelio::self_contact
