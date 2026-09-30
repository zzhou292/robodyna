#include "JsonReader.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace crash::modelio::assembly::reader {
const Value& Member(const Value& object, const char* key) {
    Require(object.IsObject() && object.HasMember(key), "Missing assembly JSON member");
    return object[key];
}
const Value& Array(const Value& object, const char* key, std::size_t cap, std::size_t minimum) {
    const auto& value = Member(object, key);
    Require(value.IsArray() && value.Size() <= cap && value.Size() >= minimum, "Assembly array exceeds shape/cap");
    return value;
}
std::string Text(const Value& value) {
    Require(value.IsString(), "Expected assembly text");
    return {value.GetString(), value.GetStringLength()};
}
std::string Text(const Value& object, const char* key) { return Text(Member(object, key)); }
std::uint64_t Unsigned(const Value& value, std::uint64_t cap) {
    Require(value.IsUint64() && value.GetUint64() <= cap, "Invalid assembly unsigned integer");
    return value.GetUint64();
}
std::uint64_t Unsigned(const Value& object, const char* key, std::uint64_t cap) {
    return Unsigned(Member(object, key), cap);
}
double Real(const Value& value) {
    Require(value.IsNumber() && std::isfinite(value.GetDouble()), "Invalid assembly finite real");
    return value.GetDouble();
}
double Real(const Value& object, const char* key) { return Real(Member(object, key)); }
std::optional<double> OptionalReal(const Value& object, const char* key) {
    const auto& value = Member(object, key);
    return value.IsNull() ? std::optional<double>{} : Real(value);
}
void Flag(const Value& object, const char* key, bool expected) {
    const auto& value = Member(object, key);
    Require(value.IsBool() && value.GetBool() == expected, "Unexpected assembly scope flag");
}
void TextIs(const Value& object, const char* key, std::string_view expected) {
    Require(Text(object, key) == expected, "Unexpected assembly schema/policy text");
}
void Same(double a, double b) {
    Require(output::Bits(a) == output::Bits(b), "Assembly binary64 association changed");
}
void UniqueKeys(const Value& value, unsigned depth) {
    Require(depth <= 32, "Assembly JSON nesting cap exceeded");
    if (value.IsObject()) {
        std::set<std::string_view> names;
        for (auto it = value.MemberBegin(); it != value.MemberEnd(); ++it) {
            Require(names.emplace(it->name.GetString(), it->name.GetStringLength()).second,
                    "Duplicate assembly JSON member");
            UniqueKeys(it->value, depth + 1);
        }
    } else if (value.IsArray()) {
        for (const auto& entry : value.GetArray()) UniqueKeys(entry, depth + 1);
    }
}
std::vector<SourceId> Ids(const Value& value, std::size_t cap, bool ascending) {
    Require(value.IsArray() && value.Size() <= cap, "Assembly ID array exceeds cap");
    std::vector<SourceId> ids;
    std::set<SourceId> unique;
    for (const auto& entry : value.GetArray()) {
        const auto id = Unsigned(entry, INT64_MAX);
        Require(id && unique.insert(id).second && (!ascending || ids.empty() || ids.back() < id),
                "Repeated, zero or unordered assembly source ID");
        ids.push_back(id);
    }
    return ids;
}
SourceBlock Block(const Value& value) {
    SourceBlock block{Text(value, "filename"), Text(value, "keyword"), Text(value, "raw_text"),
                      Text(value, "sha256"), Unsigned(value, "first_line", SIZE_MAX),
                      Unsigned(value, "last_line", SIZE_MAX)};
    Require(block.first_line && block.first_line <= block.last_line &&
            output::Sha256(block.raw_text) == block.sha256, "Assembly source block evidence mismatch");
    return block;
}
std::vector<DeclarationCard> Cards(const Value& object, std::size_t cap) {
    std::vector<DeclarationCard> cards;
    for (const auto& value : Array(object, "cards", cap, 1).GetArray()) {
        DeclarationCard card;
        card.source_line = Unsigned(value, "source_line", SIZE_MAX);
        card.blank_mask = Unsigned(value, "blank_field_mask", 255);
        card.raw_text = Text(value, "raw_data_text");
        const auto& names = Array(value, "names", 8, 1);
        const auto& values = Array(value, "values", names.Size(), names.Size());
        for (unsigned i = 0; i < names.Size(); ++i) {
            card.names.push_back(Text(names[i]));
            Require(values[i].IsNull() == bool(card.blank_mask & (1u << i)), "Declaration blank mask mismatch");
            card.values.push_back(values[i].IsNull() ? std::optional<double>{} : Real(values[i]));
        }
        cards.push_back(std::move(card));
    }
    return cards;
}
std::vector<RawAttachmentCard> AttachmentCards(const Value& object, std::size_t cap) {
    std::vector<RawAttachmentCard> cards;
    for (const auto& value : Array(object, "cards", cap, 1).GetArray()) {
        RawAttachmentCard card;
        card.source_line = Unsigned(value, "source_line", SIZE_MAX);
        card.blank_mask = Unsigned(value, "blank_mask", 255);
        card.raw_text = Text(value, "raw_text");
        const auto& names = Array(value, "names", 8, 8);
        const auto& fields = Array(value, "fields", 8, 8);
        for (unsigned i = 0; i < 8; ++i) {
            card.names.push_back(Text(names[i])); card.fields.push_back(Text(fields[i]));
            const bool blank = card.fields.back().find_first_not_of(" \t\r\n") == std::string::npos;
            Require(blank == bool(card.blank_mask & (1u << i)), "Attachment blank mask mismatch");
        }
        cards.push_back(std::move(card));
    }
    return cards;
}
std::size_t NodeIndex(const Data& data, SourceId id) {
    const auto found = std::lower_bound(data.nodes.begin(), data.nodes.end(), id,
        [](const auto& node, SourceId key) { return node.source_id < key; });
    Require(found != data.nodes.end() && found->source_id == id, "Missing selected assembly node");
    return static_cast<std::size_t>(found - data.nodes.begin());
}
}  // namespace crash::modelio::assembly::reader
