#include "classification/Internal.h"
#include <iterator>

namespace crash::modelio::tied_shell {
struct TiedClassificationContext::Storage {
    Storage(const TiedAuxiliaryConstraints& a, const vehicle::rigid_part::RigidPartSource& r)
        : auxiliary(a), rigid(r) {}
    TiedAuxiliaryConstraints auxiliary;
    vehicle::rigid_part::RigidPartSource rigid;
    ClassificationSourceReceipt receipt;
};
std::size_t TiedClassificationContext::Forecast(const TiedAuxiliaryConstraints& auxiliary,
        const vehicle::rigid_part::RigidPartSource& rigid, std::size_t wall_bytes,
        ClassificationSourceLimits limits) {
    using output::Require;
    using classification_detail::Add;
    const ClassificationSourceLimits hard;
    const std::size_t values[] = {limits.host_bytes, limits.metadata_bytes, limits.wall_member_bytes,
        limits.source_files, limits.source_blocks, limits.slaves};
    const std::size_t maximum[] = {hard.host_bytes, hard.metadata_bytes, hard.wall_member_bytes,
        hard.source_files, hard.source_blocks, hard.slaves};
    for (std::size_t i = 0; i < std::size(values); ++i)
        Require(values[i] && values[i] <= maximum[i], "Invalid classification source limits");
    const auto& declaration = auxiliary.declaration();
    const auto& canonical = declaration.canonical().data();
    Require(&canonical == &rigid.source().canonical().data(), "Classification source backing identity differs");
    Require(wall_bytes && wall_bytes <= limits.wall_member_bytes &&
            canonical.canonical_bytes.size() <= limits.metadata_bytes &&
            !declaration.data().slave_nodes.empty() && declaration.data().slave_nodes.size() <= limits.slaves,
            "Classification source count or input cap exceeded");
    // The rigid API exposes an inclusive startup reservation. Retaining it is
    // conservative: its former parser/scratch is not reallocated here. It also
    // charges the shared VehicleSourcePlan/canonical backing exactly once.
    std::size_t bytes = rigid.data().startup_budget_bytes;
    Add(bytes, declaration.data().owned_payload_bytes, 1, limits.host_bytes);
    Add(bytes, auxiliary.data().owned_payload_bytes, 1, limits.host_bytes);
    Add(bytes, sizeof(Storage), 1, limits.host_bytes);
    Add(bytes, canonical.canonical_bytes.size(), 6, limits.host_bytes);
    Add(bytes, limits.source_blocks, sizeof(ClassificationRoleEvidence) + 512, limits.host_bytes);
    Add(bytes, wall_bytes, 12, limits.host_bytes);
    return bytes;
}
TiedClassificationContext TiedClassificationContext::Prepare(const TiedAuxiliaryConstraints& auxiliary,
        const vehicle::rigid_part::RigidPartSource& rigid, const std::string& wall,
        OriginalWallAssemblyPolicy policy, ClassificationSourceLimits limits) {
    const auto forecast = Forecast(auxiliary, rigid, wall.size(), limits);
    output::Require(policy == OriginalWallAssemblyPolicy::ReplaceWholeOriginalWallWithMeshWall,
                    "Unsupported original wall assembly replacement policy");
    auto next = std::make_shared<Storage>(auxiliary, rigid);
    const auto& declaration = auxiliary.declaration().data();
    const auto& canonical = auxiliary.declaration().canonical().data();
    classification_detail::ReadSet(declaration, auxiliary.data(), rigid.data(), next->receipt);
    classification_detail::SourceRoles(canonical, declaration, auxiliary.data(), rigid.data(), next->receipt, limits);
    next->receipt.wall = classification_detail::Wall(canonical, declaration, wall, limits);
    next->receipt.startup_budget_bytes = forecast;
    next->receipt.owned_payload_bytes = classification_detail::OwnedPayload(next->receipt, limits.host_bytes);
    return TiedClassificationContext(std::move(next));
}
const TiedAuxiliaryConstraints& TiedClassificationContext::auxiliary() const noexcept { return storage_->auxiliary; }
const vehicle::rigid_part::RigidPartSource& TiedClassificationContext::rigid() const noexcept { return storage_->rigid; }
const ClassificationSourceReceipt& TiedClassificationContext::receipt() const noexcept { return storage_->receipt; }
}
