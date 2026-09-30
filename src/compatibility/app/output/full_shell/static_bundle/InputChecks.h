#pragma once
#include "Types.h"
#include "output/BoundedArrayJson.h"

namespace crash::output::full_shell::source::detail {
const Value& Field(const Value&, const char*);
std::string ReadFile(const std::filesystem::path&, const RecordFile&, std::size_t cap);
std::size_t SourceReadBudget(const SourceInputs&, SourceLimits);
void AddBytes(std::size_t&, std::size_t, std::size_t cap);
void ReadCatalog(CanonicalData&, const Value&);
void ReadCanonicalArrays(CanonicalData&,const Value&,std::size_t nodes,std::size_t shells,std::size_t solids,std::size_t beams);
void ReadDeclaredCatalog(CanonicalData&,const Value&);
void ReadDeclaredScope(CanonicalData&,const Value&);
void CheckDeclaredMemberFormat(const Value& canonical,const std::string& member);
void CheckDeclaredAnnotations(const CanonicalData&);
void ReadScope(CanonicalData&, const Value&, const Value& canonical);
void CheckScopeParts(const CanonicalData&, const Value&);
void CheckCanonicalGeometry(const CanonicalData&);
} // namespace crash::output::full_shell::source::detail
