#pragma once
#include "output/full_shell/FullShellVisualizationRecords.h"
#include <array>
#include <memory>

namespace crash::output::full_shell::source {
inline constexpr const char* CanonicalSchema = "tlfea.yaris_source_vehicle_geometry.v1";
inline constexpr const char* ScopeSchema = "robo-dyna.full-shell-scope.v1";
inline constexpr const char* MappingSchema = "robo_dyna.full_shell_source_mapping.v1";
inline constexpr std::size_t CanonicalArrayCount = 17;

struct Units {
    std::string mass, length, time;
    double mass_to_kg = 0, length_to_m = 0, time_to_s = 0;
};
struct SourceInputs {
    std::filesystem::path canonical_root, scope_root, member_root;
    RecordFile canonical_manifest, scope_report, source_member;
    // Explicit expected policy/units, supplied by the authenticated source
    // compiler. This I/O layer does not classify tires or parse model units.
    std::string tire_policy;
    Units units;
};
struct SourceLimits {
    std::size_t file_bytes = kArtifactFileCap;
    std::size_t source_member_bytes = 64 * 1024 * 1024;
    std::size_t canonical_array_bytes = 64 * 1024 * 1024;
    std::size_t host_bytes = 384 * 1024 * 1024;
    std::uint32_t nodes = 1048576, parents = 1048576;
};
struct PartDeclaration {
    std::uint64_t part = 0, material = 0, section = 0;
    unsigned source_elform = 0;
    bool shell_section = false;
};
struct NamedArray {
    std::string name;
    arrays::Descriptor descriptor;
    std::string bytes; // Exact immutable canonical little-endian source bytes.
};
struct CanonicalData {
    SourceInputs inputs;
    SourceLimits limits;
    std::string canonical_bytes, scope_bytes;
    std::string archive_sha256;
    std::array<NamedArray, CanonicalArrayCount> arrays;
    std::vector<PartDeclaration> parts; // Sorted original PID table.
    std::vector<std::uint64_t> selected_parts, excluded_parts;
    std::size_t canonical_nodes = 0, canonical_shells = 0;
    std::size_t retained_nodes = 0, retained_shells = 0, retained_q4 = 0, retained_t3 = 0;
    std::size_t excluded_shells = 0, excluded_q4 = 0, excluded_t3 = 0;
    std::string retained_shell_ids_sha256, excluded_shell_ids_sha256, selected_node_ids_sha256;
};
void CheckUnits(const Units&);
bool SameUnits(const Units&, const Units&) noexcept;
const NamedArray& FindArray(const CanonicalData&, const char* name);
const PartDeclaration& FindPart(const CanonicalData&, std::uint64_t id);
// Read and authenticate before parsing; no caller-provided content is inferred
// to have passed source/native mechanics admission.
class CanonicalSource {
  public:
    static CanonicalSource Read(const SourceInputs&, SourceLimits = {});
    // Same source/catalog authority, with complete original member bytes supplied
    // by the bounded chunk reader. Size/hash are checked before interpretation.
    static CanonicalSource ReadWithMemberBytes(const SourceInputs&, const std::string&, SourceLimits = {});
    CanonicalSource(const CanonicalSource&) noexcept = default;
    CanonicalSource(CanonicalSource&& other) noexcept : data_(other.data_) {}
    CanonicalSource& operator=(const CanonicalSource&) = delete;
    CanonicalSource& operator=(CanonicalSource&&) = delete;
    const CanonicalData& data() const noexcept { return *data_; }
  private:
    explicit CanonicalSource(std::shared_ptr<const CanonicalData> data) : data_(std::move(data)) {}
    std::shared_ptr<const CanonicalData> data_;
};
} // namespace crash::output::full_shell::source
