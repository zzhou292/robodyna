#pragma once
#include "../MixedStarterSource.h"
#include "../TopologyDigestFields.h"
namespace crash::cases::vehicle_self_contact::native::mixed_starter::detail {
struct Failure { Report report; };
[[noreturn]] inline void Reject(Status status, const char* reason) {
    throw Failure{{status, reason}};
}
Forecast Budget(const MainSource&, Limits, const DomainEmbedding* = nullptr);
const tl::fea::NodalNodeDomain& OriginalDomain(const MainSource&) noexcept;
void CheckEmbedding(const MainSource&, const DomainEmbedding&);
struct CombinedInput {
    std::vector<std::uint64_t> ids;
    std::vector<double> positions;
    std::size_t capacity_bytes() const noexcept;
    s::Input Input(const MainSource&) const noexcept;
};
CombinedInput PackCombined(const MainSource&, const DomainEmbedding&, std::size_t capacity);
s::NodePrefixExtension Prefix(const MainSource&) noexcept;
c::Topology NormalTopology(const s::Snapshot&) noexcept;
std::string Digest(const Provenance&, const s::Snapshot&, const s::Input&, std::size_t cap);
}
