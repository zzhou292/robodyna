#pragma once
#include "RigidPartDeclarations.h"
#include "modelio/tied_shell/TiedShellDeclaration.h"
#include "lib_src/constraints/NodalRigidPartTopology.h"
namespace crash::modelio::vehicle::rigid_part {
struct Limits {
    std::size_t host_bytes=512*1024*1024, member_bytes=64*1024*1024;
    std::size_t metadata_bytes=2*1024*1024, members=16384, parts=1024, blocks=16384;
};
struct Body {
    std::uint64_t source_part_id=0;
    std::size_t source_part_index=SIZE_MAX, shell_count=0;
    Declaration declaration;
    // PART union is ascending original NID; extras preserve literal SET order.
    // Neither is asserted to be native internal-node/centroid reduction order.
    std::vector<std::uint64_t> part_nodes, extra_nodes;
    std::size_t extra_source=SIZE_MAX, node_set_source=SIZE_MAX;
    std::uint64_t node_set_id=0;
};
struct RootNode {
    std::uint64_t source_node_id=0;
    std::size_t root_index=SIZE_MAX;
};
struct SourceData {
    std::vector<Body> bodies; // Ascending original PID, not native RBODY registration.
    std::vector<tied_shell::SourceEvidence> sources; // Exact original line order.
    std::vector<std::uint64_t> plain_rigid_members;
    std::vector<std::size_t> part_to_body; // Original part index -> body or SIZE_MAX.
    std::vector<tl::fea::rigid::PartTopologyMerge> merges;
    std::vector<RootNode> node_roots; // Ascending NID, source-only connection lookup.
    std::size_t shell_count=0, startup_budget_bytes=0;
};
// Immutable source topology and literal cards. No centroid, M/J, generated ID,
// native registration order, owner, force suppression or runtime admission.
class RigidPartSource {
  public:
    static RigidPartSource Prepare(const VehicleSourcePlan&,const std::string& original_member,Limits={});
    static std::size_t AdditionalForecast(const VehicleSourcePlan&,Limits={});
    RigidPartSource(const RigidPartSource&) noexcept=default;
    RigidPartSource(RigidPartSource&& other) noexcept:storage_(other.storage_) {}
    RigidPartSource& operator=(const RigidPartSource&)=delete;
    const VehicleSourcePlan& source() const noexcept;
    const SourceData& data() const noexcept;
    const tl::fea::rigid::NodalRigidPartTopology& topology() const noexcept;
    const Body* body(std::size_t source_part) const noexcept;
    std::size_t root_index(std::size_t source_part) const noexcept;
    std::size_t root_for_node(std::uint64_t source_node) const noexcept;
  private:
    struct Storage;
    explicit RigidPartSource(std::shared_ptr<const Storage> value):storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
}
