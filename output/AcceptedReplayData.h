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
struct Entry {
    std::uint64_t owner = 0, epoch = 0;
    double time = 0;
    std::string mesh, obj;
    // v2 only: exact preceding interval, retained for <=1000 saved frames.
    std::uint64_t interval_attempt=0;
    double interval_base_time=0;
    double interval_base_velocity_time=0,interval_previous_base_time=0;
    double interval_base_wall_potential=0;
    std::array<std::uint64_t,3> interval_contact_history{};
    std::array<double,28> wall_interval_values{}; // Wall columns 6..33, saved frames only.
};
struct ContactParentBinding {
    std::uint64_t element = 0, face = 0, feature = 0;
    std::array<std::uint64_t,4> connectivity{};
};
struct AssemblyReplayData;
struct Bundle {
    std::shared_ptr<AssemblyReplayData> assembly;
    std::filesystem::path directory;
    ReplayInfo info;
    std::size_t total_cap=kTotalCap,manifest_bytes=0;
    double plastic_curve_maximum=0;
    double plastic_initial_thickness=0,plastic_min_thickness_ratio=0,plastic_max_thickness_ratio=0;
    std::array<double,3> plastic_final_values{};
    std::array<std::uint64_t,2> plastic_final_counts{};
    std::vector<std::array<double,2>> plastic_curve;
    std::vector<std::array<std::uint64_t,3>> plastic_source_parents; // EID, arity, family index.
    std::vector<double> plastic_reference_volume;
    std::map<std::string, Artifact> inventory;
    std::vector<Entry> entries;
    std::vector<std::array<int,3>> topology;
    std::vector<std::array<std::uint64_t,9>> source_triangles;
    double fixed_dt = 0;
    std::uint64_t qualification_id = 0, wall_binding_id = 0;
    std::uint64_t source_configuration_id = 0;
    std::vector<ContactParentBinding> contact_parents;
    std::string contact_integration_backend;
    unsigned contact_depth_limit=16;
    unsigned contact_leaf_limit=4096,contact_visit_limit=16384;
    bool explicit_contact_backend=false;
    bool explicit_guided_experiment=false;
    std::array<double,3> source_initial_velocity{};
    double wall_x=0,wall_penetration_cap=0,source_initial_kinetic=0,wall_energy_allowance=0;
    std::vector<std::array<std::uint64_t,3>> wall_source_parents; // EID, arity, original index.
    std::vector<std::uint64_t> wall_faces;
};
Document Json(const std::string& bytes);
const Value& Member(const Value&, const char* name);
std::uint64_t Unsigned(const Value&, const char* name);
double Real(const Value&, const char* name);
std::string Text(const Value&, const char* name);
std::string VerifiedBytes(const Bundle&, const std::string& name);
Bundle ReadIndex(const std::filesystem::path&);
void CheckReplayTime(double actual,double expected,double dt,std::uint64_t epoch);
std::shared_ptr<chrono::ChTriangleMeshConnected> ReadMesh(const Bundle&, const std::string&);
void CheckFrameFields(const Bundle&, const Entry&, const chrono::ChTriangleMeshConnected&);
void CheckPositionFields(const Value&, const chrono::ChTriangleMeshConnected&);
void ReadSourcePartConfiguration(Bundle&, const Document&, const Document&, const Document&);
void CheckSourcePartFields(const Bundle&, const Entry&, const chrono::ChTriangleMeshConnected&);
void ReadSourcePartIdentity(Bundle&,const Document&);
void CheckSourcePartFieldData(const Bundle&,const Entry&,const chrono::ChTriangleMeshConnected&,
                             const Document&,const std::array<double,3>& startup_velocity);
void ReadSourcePartPlasticConfiguration(Bundle&,const Document&);
void CheckSourcePartPlasticFields(const Bundle&,const Entry&,const Document&);
std::vector<ReplayParentScalar> ReadSourcePartPlasticDisplay(const Bundle&,const Entry&);
void ReadSourcePartWallConfiguration(Bundle&,const Document&,const Document&,const Document&);
void CheckSourcePartWallFields(const Bundle&,const Entry&,const chrono::ChTriangleMeshConnected&);
void ReadGuidedConfiguration(Bundle&, const Document&, const Document&, const Document&);
void CheckGuidedLedgers(Bundle&, const Document& configuration, const Document& manifest);
void CheckGuidedFields(const Bundle&, const Entry&, const chrono::ChTriangleMeshConnected&);
void CheckGuidedWall(const Bundle&, const chrono::ChTriangleMeshConnected&);
}  // namespace crash::output::replay_detail
