#pragma once
#include "SourceAssemblyWallSchema.h"

namespace crash::output::assembly::wall_files {
// Complete serialization sizing precedes create-only output; the shared writer
// still performs checked flush/close. Manifest publication rechecks every closed
// artifact's bytes/hash so later truncation cannot acquire a completion marker.
std::size_t JsonBytes(const Document&,std::size_t cap);
void WriteJsonBounded(const std::filesystem::path&,const Document&,std::size_t cap);
void PublishManifest(const std::filesystem::path&,const Document&,std::size_t total_cap);
} // namespace crash::output::assembly::wall_files
