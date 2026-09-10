#pragma once
#include "MappingRecords.h"

namespace crash::output::full_shell::source::detail {
Document MappingDocument(const CanonicalSource&, const std::string& digest,
    const std::array<arrays::Descriptor, 8>&);
std::array<arrays::Descriptor, 8> ParseMappingDocument(const CanonicalSource&,
    const Value&, const std::string& expected_digest);
} // namespace crash::output::full_shell::source::detail
