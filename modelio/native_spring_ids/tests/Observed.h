#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>
namespace crash::modelio::native_spring_ids::test {
// Captured qualification evidence only. These types are never production input
// or a shipping generated-ID lookup table. Original source-line fields and full
// printed native row text are retained alongside the compared identity fields.
struct ObservedGenerated {
    std::string keyword;
    std::uint64_t source_id = 0, native_id = 0, node1 = 0, node2 = 0;
    std::size_t output_line = 0;
    // The original joint extraction did not record these two source lines.
    std::optional<std::size_t> source_keyword_line, source_id_line;
};
struct ObservedNative {
    std::uint64_t native_id = 0, property_id = 0, node1 = 0, node2 = 0;
    std::size_t local = 0, output_line = 0;
    std::string raw_line;
};
struct Observed {
    std::vector<ObservedGenerated> generated;
    std::vector<ObservedNative> native_table;
};
// Require the two exact sealed CSV byte streams, strict schema and complete
// row counts. Throws on any mismatch; no partial observation is returned.
Observed LoadObserved(const std::filesystem::path& directory);
}
