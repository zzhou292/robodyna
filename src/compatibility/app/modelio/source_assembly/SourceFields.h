#pragma once
#include "JsonReader.h"
#include <optional>
namespace crash::modelio::assembly::reader {
// Exact fixed-column conversion shared by source adapters. Blank remains
// distinct from a written zero, including signed zero.
std::optional<double> SourceScalar(const std::string&, unsigned field, unsigned width=10);
double RequiredScalar(const std::string&, unsigned field, unsigned width=10);
void RequireBlankFields(const std::string&, unsigned first, unsigned last, unsigned width=10);
} // namespace crash::modelio::assembly::reader
