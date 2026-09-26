#pragma once
#include "Internal.h"
#include <climits>
#include <cmath>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
namespace tied = modelio::tied_shell;
inline std::uint64_t Shift(std::uint64_t value, std::int64_t offset) {
    if (!value || value > INT_MAX || offset < -INT_MAX || offset > INT_MAX ||
        std::int64_t(value) + offset <= 0 || std::int64_t(value) + offset > INT_MAX)
        Reject(Status::UnsupportedSource, "Native part/section/material identity or include offset is out of range");
    return std::uint64_t(std::int64_t(value) + offset);
}
inline std::int64_t Offset(const tied::SourceEvidence& source) {
    if (source.block.keyword == "*INCLUDE") return 0;
    if (source.cards.size() != 5)
        Reject(Status::UnsupportedSource, "Incomplete property include-transform descriptor");
    std::optional<std::int64_t> common;
    // First scope admits a common offset across every ID namespace. It does
    // not guess which mixed offset applies to a SECTION versus a SET.
    for (unsigned row = 1; row <= 2; ++row) {
        const unsigned columns = row == 1 ? 7 : 1;
        for (unsigned column = 0; column < columns; ++column) {
            const auto value = modelio::vehicle::detail::SourceScalar(source.cards[row].second, column);
            const double supplied = value.value_or(0.);
            if (supplied < -INT_MAX || supplied > INT_MAX || std::floor(supplied) != supplied)
                Reject(Status::UnsupportedSource, "Nonliteral or overflowing property include offset");
            const auto current = static_cast<std::int64_t>(supplied);
            if (common && current != *common)
                Reject(Status::NeedsNativePropertyMapping, "Mixed native ID-namespace offsets need an explicit property transform map");
            common = current;
        }
    }
    return *common;
}
}
