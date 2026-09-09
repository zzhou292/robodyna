#pragma once

// Internal parsing/storage for the bounded replay reader. No solver dependency.
#include "AcceptedReplay.h"
#include "ArtifactIO.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <map>
#include <vector>

namespace crash::output::replay_detail {
constexpr std::size_t kFileCap = 32*1024*1024, kTotalCap = 256*1024*1024;
constexpr std::size_t kVertexCap = 4096, kTriangleCap = 8192, kFrameCap = 1000;
struct Artifact { std::string hash; std::size_t bytes = 0; };
struct Entry { std::uint64_t owner = 0, epoch = 0; double time = 0; std::string mesh, obj; };
struct ContactParentBinding {
    std::uint64_t element = 0, face = 0, feature = 0;
    std::array<std::uint64_t,4> connectivity{};
};
struct Bundle {
    std::filesystem::path directory;
    ReplayInfo info;
    std::map<std::string, Artifact> inventory;
    std::vector<Entry> entries;
    std::vector<std::array<int,3>> topology;
    std::vector<std::array<std::uint64_t,9>> source_triangles;
    double fixed_dt = 0;
    std::uint64_t qualification_id = 0, wall_binding_id = 0;
    std::vector<ContactParentBinding> contact_parents;
};
Document Json(const std::string& bytes);
const Value& Member(const Value&, const char* name);
std::uint64_t Unsigned(const Value&, const char* name);
double Real(const Value&, const char* name);
std::string Text(const Value&, const char* name);
std::string VerifiedBytes(const Bundle&, const std::string& name);
Bundle ReadIndex(const std::filesystem::path&);
std::shared_ptr<chrono::ChTriangleMeshConnected> ReadMesh(const Bundle&, const std::string&);
void CheckFrameFields(const Bundle&, const Entry&, const chrono::ChTriangleMeshConnected&);
void CheckPositionFields(const Value&, const chrono::ChTriangleMeshConnected&);
void ReadGuidedConfiguration(Bundle&, const Document&, const Document&, const Document&);
void CheckGuidedFields(const Bundle&, const Entry&, const chrono::ChTriangleMeshConnected&);
void CheckGuidedWall(const Bundle&, const chrono::ChTriangleMeshConnected&);
}  // namespace crash::output::replay_detail
