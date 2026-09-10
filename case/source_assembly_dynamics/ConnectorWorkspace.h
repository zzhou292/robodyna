#pragma once
#include "lib_src/elements/type25/Type25Batch.h"
#include <array>
#include <vector>

namespace crash::cases::source_assembly_dynamics {
struct ConnectorWorkspace {
    explicit ConnectorWorkspace(std::size_t count):results{std::vector<tl::fea::type25::Evaluation>(count),
                                                         std::vector<tl::fea::type25::Evaluation>(count)} {}
    tl::fea::type25::Batch batch;
    // Reuse the case's accepted sample slot. No independent selector, token or
    // accepted clock; these are copied results available after common commit.
    std::array<std::vector<tl::fea::type25::Evaluation>,2> results;
};
} // namespace crash::cases::source_assembly_dynamics
