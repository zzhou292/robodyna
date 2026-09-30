#pragma once
#include "Types.h"

namespace crash::output::full_shell::source::detail {
struct CanonicalSpec { const char* name; arrays::Scalar scalar; unsigned columns; const char* fields; };
const std::array<CanonicalSpec, CanonicalArrayCount>& CanonicalSpecs();
std::vector<std::string> FieldNames(const char* comma_separated);
} // namespace crash::output::full_shell::source::detail
