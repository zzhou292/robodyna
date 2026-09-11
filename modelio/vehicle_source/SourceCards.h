#pragma once
#include "modelio/source_assembly/SourceAssemblyData.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::vehicle::detail {
// Shared declaration-only source association and exact fixed-column conversion.
void CheckSource(const assembly::SourceBlock&, const output::Value& expected,
                 const char* hash_key = "sha256");
void CheckTypedCards(const assembly::SourceBlock&,
                     const std::vector<assembly::DeclarationCard>&, unsigned width = 10);
std::optional<double> SourceScalar(const std::string& line, unsigned field, unsigned width = 10);
} // namespace crash::modelio::vehicle::detail
