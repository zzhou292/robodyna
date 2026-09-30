#pragma once
#include "TiedShellDeclaration.h"
#include <optional>

namespace crash::modelio::tied_shell {
enum class OriginalWallPolicy { RetainUnresolved, ReplaceWithMeshWall };
enum class AuxiliaryNodeOrigin { CanonicalMember, AuxiliaryMember };
struct AuxiliaryLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t member_bytes = 1024 * 1024;
    std::size_t metadata_bytes = 512 * 1024;
    std::size_t groups = 64, group_members = 8192, blocks = 8192;
};
struct AuxiliaryMemberNode {
    SourceId id = 0;
    AuxiliaryNodeOrigin origin = AuxiliaryNodeOrigin::CanonicalMember;
    std::uint32_t canonical_index = UINT32_MAX;
    std::size_t source = SIZE_MAX, source_line = 0;
    std::array<std::int32_t, 2> source_codes{};
    // Auxiliary cards retain blank versus explicit zero. Canonical codes have
    // already been resolved by the authenticated canonical source compiler.
    unsigned auxiliary_code_blank_mask = 0;
    bool master = false, slave = false;
};
struct AuxiliaryGroup {
    GroupEvidence evidence;
    // Source card values only: no native defaults or generated main node.
    std::array<std::optional<double>, 8> source_fields;
};
struct AuxiliaryCounts {
    std::size_t groups = 0, member_occurrences = 0, distinct_members = 0;
    std::size_t groups_touching_masters = 0, groups_touching_slaves = 0;
    std::size_t distinct_master_members = 0, distinct_slave_members = 0;
    std::size_t auxiliary_member_nodes = 0;
};
struct AuxiliaryFileIdentity {
    std::string filename, sha256; // Authenticated canonical inventory identity.
};
struct AuxiliaryData {
    std::string filename, member_sha256;
    std::size_t member_bytes = 0;
    std::vector<SourceEvidence> sources; // Auxiliary file/line order.
    std::vector<AuxiliaryGroup> groups;  // Original group declaration order.
    std::vector<AuxiliaryMemberNode> member_nodes; // Ascending original NID.
    OriginalWallPolicy wall_policy = OriginalWallPolicy::RetainUnresolved;
    // Complete original wall identities from the authenticated canonical
    // inventory. ReplaceWithMeshWall explicitly excludes all these cards.
    std::vector<UnresolvedBlock> original_walls;
    std::vector<AuxiliaryFileIdentity> wall_source_files;
    // Any retained_source index here refers to declaration().data().sources.
    std::vector<UnresolvedBlock> other_unresolved_constraints;
    AuxiliaryCounts counts;
    std::size_t startup_budget_bytes = 0, owned_payload_bytes = 0;
};

// Source evidence only. Retains the declaration/canonical authority by value;
// does not construct IKINE, rigid registration order, native CIN/PEN, or owner
// admission. Original walls remain unresolved unless explicitly replaced.
class TiedAuxiliaryConstraints {
  public:
    static TiedAuxiliaryConstraints Prepare(const TiedShellDeclaration&,
        const std::string& auxiliary_member, OriginalWallPolicy, AuxiliaryLimits = {});
    static std::size_t Forecast(const TiedShellDeclaration&, std::size_t auxiliary_bytes,
                               AuxiliaryLimits = {});
    TiedAuxiliaryConstraints(const TiedAuxiliaryConstraints&) noexcept = default;
    TiedAuxiliaryConstraints(TiedAuxiliaryConstraints&& other) noexcept : storage_(other.storage_) {}
    TiedAuxiliaryConstraints& operator=(const TiedAuxiliaryConstraints&) = delete;
    TiedAuxiliaryConstraints& operator=(TiedAuxiliaryConstraints&&) = delete;
    const TiedShellDeclaration& declaration() const noexcept;
    const AuxiliaryData& data() const noexcept;
  private:
    struct Storage;
    explicit TiedAuxiliaryConstraints(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::tied_shell
