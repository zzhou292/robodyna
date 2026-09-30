#pragma once
#include "TiedAuxiliaryConstraints.h"
#include "modelio/rigid_part/RigidPartSource.h"

namespace crash::modelio::tied_shell {
inline constexpr const char* ClassificationSourcePolicy =
    "openradioss_a62b27e_original_tied_slave_projection_mesh_wall_v1";
enum class OriginalWallAssemblyPolicy { ReplaceWholeOriginalWallWithMeshWall };
enum class ClassificationSourceRole {
    RigidMembersOutsideObservedSlaves,
    SpringJointOutsideObservedSlaves,
    ConvertedSpring,
    SelectedType2,
    NonType2Contact,
    ReplacedOriginalWall,
    LinearSolidTopology,
};
struct ClassificationRoleEvidence {
    UnresolvedBlock block;
    ClassificationSourceRole role;
};
struct OriginalWallAssemblyReceipt {
    std::string filename, sha256;
    SourceId part_id = 0, section_id = 0, material_id = 0;
    std::vector<SourceId> node_ids, shell_ids;
    std::vector<SourceEvidence> sources;
};
struct ClassificationSourceReceipt {
    std::vector<ClassificationRoleEvidence> roles; // Original file/block order.
    OriginalWallAssemblyReceipt wall;
    std::size_t source_files = 0, source_blocks = 0, observed_slaves = 0;
    std::size_t plain_groups = 0, auxiliary_groups = 0, rigid_parts = 0;
    std::size_t joints = 0, checked_joint_node_fields = 0;
    std::size_t original_type2_interfaces = 0, replaced_primitive_walls = 0;
    std::size_t startup_budget_bytes = 0, owned_payload_bytes = 0;
};
struct ClassificationSourceLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t metadata_bytes = 8 * 1024 * 1024;
    std::size_t wall_member_bytes = 64 * 1024;
    std::size_t source_files = 16, source_blocks = 16384, slaves = 65536;
};
// Source proof for observed original slaves only. Unobserved native conditions
// remain unspecified. This does not run ITAGSL2/KINCHK or admit an owner.
class TiedClassificationContext {
  public:
    static TiedClassificationContext Prepare(const TiedAuxiliaryConstraints&,
        const vehicle::rigid_part::RigidPartSource&, const std::string& original_wall_member,
        OriginalWallAssemblyPolicy, ClassificationSourceLimits = {});
    static std::size_t Forecast(const TiedAuxiliaryConstraints&,
        const vehicle::rigid_part::RigidPartSource&, std::size_t wall_member_bytes,
        ClassificationSourceLimits = {});
    TiedClassificationContext(const TiedClassificationContext&) noexcept = default;
    TiedClassificationContext(TiedClassificationContext&& other) noexcept : storage_(other.storage_) {}
    TiedClassificationContext& operator=(const TiedClassificationContext&) = delete;
    TiedClassificationContext& operator=(TiedClassificationContext&&) = delete;
    const TiedAuxiliaryConstraints& auxiliary() const noexcept;
    const vehicle::rigid_part::RigidPartSource& rigid() const noexcept;
    const ClassificationSourceReceipt& receipt() const noexcept;
  private:
    struct Storage;
    explicit TiedClassificationContext(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
}
