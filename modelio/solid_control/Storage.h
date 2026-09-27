#pragma once
#include "Types.h"
#include "output/ArtifactIO.h"
#include <limits>
namespace crash::modelio::solid_control::detail {
inline void Charge(std::size_t& total, std::size_t count, std::size_t width = 1) {
    output::Require(width && count <= (SIZE_MAX-total)/width, "Solid-control retained byte count overflows");
    total += count*width;
}
inline void StringBytes(std::size_t& total, const std::string& value) {
    Charge(total, value.capacity()+1); Charge(total, 32); // Includes allocator overhead; SSO is conservatively double charged.
}
template<class T> void VectorBytes(std::size_t& total, const std::vector<T>& values) {
    Charge(total, values.capacity(), sizeof(T)); Charge(total, 32);
}
inline std::size_t DirectBytes(const DirectData& data, std::size_t object_bytes) {
    std::size_t bytes = object_bytes+256; // Shared ownership/control and allocator slack.
    VectorBytes(bytes, data.parts); VectorBytes(bytes, data.evidence);
    StringBytes(bytes, data.combine_sha256); StringBytes(bytes, data.auxiliary_sha256);
    for (const auto& item : data.evidence) {
        StringBytes(bytes, item.block.filename); StringBytes(bytes, item.block.keyword);
        StringBytes(bytes, item.block.raw_text); StringBytes(bytes, item.block.sha256);
        VectorBytes(bytes, item.cards);
        for (const auto& card : item.cards) StringBytes(bytes, card.second);
    }
    return bytes;
}
inline std::size_t EffectiveBytes(const EffectiveData& data, std::size_t object_bytes) {
    std::size_t bytes = object_bytes+256;
    VectorBytes(bytes, data.parts); VectorBytes(bytes, data.origins); StringBytes(bytes, data.source_digest);
    for (const auto& origin : data.origins) { StringBytes(bytes, origin.member); StringBytes(bytes, origin.block_sha256); }
    return bytes;
}
}
