#pragma once
#include "Types.h"
#include "output/full_shell/static_bundle/Types.h"
#include <array>
namespace chrono { class ChTriangleMeshConnected; }
namespace crash::output::physical_run {
inline constexpr const char* EnvironmentProfile="native_declared_fixed_elastic_wall_v1";
inline constexpr const char* EnvironmentRunSchema="robo_dyna.physical_accepted_run.v2";
inline constexpr std::size_t EnvironmentFileCap=1u<<20;
inline constexpr std::size_t EnvironmentWorkspaceBytes=8u<<20;
inline constexpr std::size_t EnvironmentRetainedBytes=128u<<10;
inline constexpr std::array<const char*,3> EnvironmentFiles{{
    "environment-wall.mesh.json","environment-wall.obj","environment-wall.json"}};
struct EnvironmentReceipt {
    std::uint64_t source_instance_id=0,wall_binding_id=0,part_id=0;
    std::string source_mapping_sha256;
    std::array<records::RecordFile,3> files;
};
Document EnvironmentDocument(const EnvironmentReceipt&);
EnvironmentReceipt ReadEnvironmentDocument(const Value&);
std::shared_ptr<const chrono::ChTriangleMeshConnected> ReadEnvironmentArtifacts(const std::filesystem::path&,
    const EnvironmentReceipt&,const records::source::CanonicalData&,const records::Context&);
}
