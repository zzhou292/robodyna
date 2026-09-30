#pragma once
#include "MappingRecords.h"

namespace crash::output::full_shell::source::detail {
Document MappingDocument(const CanonicalSource&, const std::string& digest,
    const std::array<arrays::Descriptor, 8>&,const MappingExecution* = nullptr);
std::array<arrays::Descriptor, 8> ParseMappingDocument(const CanonicalSource&,
    const Value&, const std::string& expected_digest,MappingExecution* = nullptr);
} // namespace crash::output::full_shell::source::detail
