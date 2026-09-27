#pragma once
#include "modelio/native_spring_ids/ImportContext.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace crash::modelio::solid_control {
namespace ids = native_spring_ids;
namespace source = output::full_shell::source;
enum class Status { Ready, InvalidInput, UnsupportedSource, ResourceLimit, NeedsNativePropertyMapping };
struct Report {
    Status status = Status::InvalidInput;
    std::string reason, source_file;
    std::size_t source_line = 0;
};
struct PartControl {
    std::uint64_t part_id = 0, section_id = 0, material_id = 0;
    // Zero is an unavailable unrequested multi-MID clone, never an invented ID.
    std::uint64_t native_property_id = 0;
    bool directly_requested = false, effective_control = false;
};
struct PartOrigin {
    std::uint64_t part_id = 0, original_part_id = 0, original_section_id = 0, original_material_id = 0;
    std::int64_t common_offset = 0;
    std::string member, block_sha256;
    std::size_t first_line = 0, last_line = 0, card_line = 0;
};
struct DirectPart {
    std::uint64_t part_id = 0, section_id = 0, material_id = 0;
    std::size_t original_solids = 0;
};
struct DirectData {
    std::vector<DirectPart> parts;
    std::vector<tied_shell::SourceEvidence> evidence; // Owned CONTACT_INTERIOR and source set cards.
    std::size_t original_solids = 0;
    std::string combine_sha256, auxiliary_sha256;
};
enum class InterfaceDisposition { Unresolved, CompleteNoApplicableType24 };
struct InterfaceCensus {
    InterfaceDisposition disposition = InterfaceDisposition::Unresolved;
    std::size_t type25_sources = 0, type2_sources = 0;
    std::size_t interior_sources = 0, rigid_wall_sources = 0;
    std::size_t checked_source_blocks = 0;
};
struct DirectLimits {
    std::size_t metadata_bytes = 1u << 20, retained_bytes = 4u << 20;
};
struct Limits {
    std::size_t parts = 4096, source_blocks = 8192, metadata_bytes = 1u << 20;
    std::size_t retained_bytes = 8u << 20;
};
} // namespace crash::modelio::solid_control
