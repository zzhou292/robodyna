#include "Internal.h"
#include "Storage.h"
namespace crash::modelio::solid_control {
struct DirectSource::Data {
    explicit Data(const source::CanonicalSource& input) : canonical(input) {}
    source::CanonicalSource canonical;
    DirectData values;
    std::size_t owned_bytes = 0;
};
DirectSource DirectSource::Prepare(const source::CanonicalSource& canonical,
                                  const ids::ImportMembers& members, DirectLimits limits) {
    const DirectLimits hard;
    output::Require(limits.metadata_bytes && limits.metadata_bytes <= hard.metadata_bytes &&
        limits.retained_bytes && limits.retained_bytes <= hard.retained_bytes, "Invalid direct solid-control capacity");
    auto next = std::make_shared<Data>(canonical);
    next->values = detail::ReadDirect(canonical.data(), members, limits.metadata_bytes);
    next->owned_bytes = detail::DirectBytes(next->values, sizeof(Data));
    output::Require(next->owned_bytes <= limits.retained_bytes,
                    "Direct solid-control retained evidence exceeds reservation");
    return DirectSource(std::move(next));
}
const source::CanonicalSource& DirectSource::canonical() const noexcept { return data_->canonical; }
const DirectData& DirectSource::data() const noexcept { return data_->values; }
std::size_t DirectSource::owned_payload_bytes() const noexcept { return data_->owned_bytes; }
} // namespace crash::modelio::solid_control
