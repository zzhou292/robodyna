#pragma once

#include <filesystem>
#include <string>

namespace chrono { class ChTriangleMeshConnected; }

namespace crash::output {

// Creates <stem>.mesh.json and <stem>.obj in the caller-owned directory.
// Uses Chrono's full-precision archive and verifies exact coordinate bits and
// topology after roundtrip. OBJ is visualization precision and is separately
// reloaded/checked. Both destinations must be absent. Throws on failure, may
// leave partial files, and never creates a completion marker or inventory.
void WriteMeshFiles(const std::filesystem::path& directory, const std::string& stem,
                    const chrono::ChTriangleMeshConnected& mesh);

}  // namespace crash::output
