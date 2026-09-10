#pragma once
#include "Types.h"
#include "output/BoundedArrayJson.h"

namespace crash::output::full_shell::source::detail {
const Value& Field(const Value&, const char*);
std::string ReadFile(const std::filesystem::path&, const RecordFile&, std::size_t cap);
void AddBytes(std::size_t&, std::size_t, std::size_t cap);
void ReadCatalog(CanonicalData&, const Value&);
void ReadScope(CanonicalData&, const Value&, const Value& canonical);
void CheckScopeParts(const CanonicalData&, const Value&);
void CheckCanonicalGeometry(const CanonicalData&);
} // namespace crash::output::full_shell::source::detail
