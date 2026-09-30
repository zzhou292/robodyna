#include "Internal.h"

namespace crash::modelio::self_contact::detail {
namespace {

void Add(std::size_t& total, std::size_t count, std::size_t width,
    std::size_t cap) {
    output::Require(total <= cap && count <= (cap - total) / width,
        "Original self-contact selection exceeds host budget");
    total += count * width;
}

void Text(std::size_t& total, const std::string& value, std::size_t cap) {
    Add(total, value.capacity() + 1, 1, cap);
}

}  // namespace

std::size_t Preflight(const source::CanonicalData& canonical,
    std::size_t auxiliary_bytes, std::size_t combine_bytes, Limits limits) {
    const Limits hard;
    output::Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
            limits.auxiliary_member_bytes &&
            limits.auxiliary_member_bytes <= hard.auxiliary_member_bytes &&
            limits.combine_member_bytes &&
            limits.combine_member_bytes <= hard.combine_member_bytes &&
            limits.metadata_bytes &&
            limits.metadata_bytes <= hard.metadata_bytes &&
            limits.blocks && limits.blocks <= hard.blocks &&
            limits.parts && limits.parts <= hard.parts &&
            auxiliary_bytes && auxiliary_bytes <= limits.auxiliary_member_bytes &&
            combine_bytes && combine_bytes <= limits.combine_member_bytes &&
            canonical.parts.size() <= limits.parts &&
            canonical.canonical_bytes.size() <= source::SourceLimits{}.file_bytes,
        "Invalid original self-contact source limits or member extents");

    std::size_t total = sizeof(Data) + sizeof(source::CanonicalData) + 512;
    Add(total, canonical.canonical_bytes.size(), 4, limits.host_bytes);
    Add(total, auxiliary_bytes, 3, limits.host_bytes);
    Add(total, combine_bytes, 3, limits.host_bytes);
    Add(total, limits.metadata_bytes, 2, limits.host_bytes);
    Add(total, limits.parts,
        sizeof(PartDisposition) + 5 * sizeof(SourceId) + 128,
        limits.host_bytes);
    Add(total, limits.blocks,
        sizeof(tied_shell::SourceEvidence) + 2 * sizeof(SourceSet) + 256,
        limits.host_bytes);
    return total;
}

std::size_t OwnedPayload(const Data& data, std::size_t cap) {
    std::size_t total = sizeof(Data);
    Text(total, data.auxiliary_filename, cap);
    Text(total, data.auxiliary_sha256, cap);
    Text(total, data.combine_filename, cap);
    Text(total, data.combine_sha256, cap);
    Add(total, data.selected_part_ids.capacity(), sizeof(SourceId), cap);
    Add(total, data.parts.capacity(), sizeof(PartDisposition), cap);
    Add(total, data.sources.capacity(), sizeof(tied_shell::SourceEvidence), cap);
    for (const auto& source : data.sources) {
        Text(total, source.block.filename, cap);
        Text(total, source.block.keyword, cap);
        Text(total, source.block.raw_text, cap);
        Text(total, source.block.sha256, cap);
        Add(total, source.cards.capacity(),
            sizeof(std::pair<std::size_t, std::string>), cap);
        for (const auto& card : source.cards) Text(total, card.second, cap);
    }
    return total;
}

}  // namespace crash::modelio::self_contact::detail
