#include "Internal.h"
#include <type_traits>

namespace crash::modelio::tied_shell::classification_detail {
void Add(std::size_t& total, std::size_t count, std::size_t width, std::size_t cap) {
    Require(width && total <= cap && count <= (cap - total) / width,
            "Tied classification source byte cap exceeded");
    total += count * width;
}
std::size_t OwnedPayload(const ClassificationSourceReceipt& receipt, std::size_t cap) {
    std::size_t bytes = sizeof(receipt);
    const auto text = [&](const std::string& value) { Add(bytes, value.capacity() + 1, 1, cap); };
    const auto array = [&](const auto& value) {
        Add(bytes, value.capacity(), sizeof(typename std::decay_t<decltype(value)>::value_type), cap);
    };
    array(receipt.roles);
    for (const auto& role : receipt.roles) {
        text(role.block.filename);
        text(role.block.keyword);
        text(role.block.sha256);
    }
    const auto& wall = receipt.wall;
    text(wall.filename);
    text(wall.sha256);
    array(wall.node_ids);
    array(wall.shell_ids);
    array(wall.sources);
    for (const auto& source : wall.sources) {
        text(source.block.filename);
        text(source.block.keyword);
        text(source.block.sha256);
        text(source.block.raw_text);
        array(source.cards);
        for (const auto& card : source.cards) text(card.second);
    }
    return bytes;
}
}
