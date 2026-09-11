#pragma once
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <memory>
#include <string>
namespace crash::output {
// Shared geometry-only Chrono archive reader. Exact binary64 coordinates and
// topology, bounded counts/types before allocation, no external/dynamic fields.
std::shared_ptr<chrono::ChTriangleMeshConnected> ReadGeometryMesh(const std::string&,
    std::size_t vertex_cap=4096,std::size_t triangle_cap=8192);
} // namespace crash::output
