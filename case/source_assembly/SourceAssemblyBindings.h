#pragma once
#include "modelio/source_assembly/SourceAssemblyMaterialInput.h"
#include "modelio/source_assembly/SourceAssemblyShellInput.h"
#include "lib_src/constraints/NodalRigidGroupModel.h"
#include <memory>
#include <stdexcept>

namespace crash::cases::source_assembly {
namespace source=crash::modelio::assembly;
struct SourceAssemblyBindingOptions {
    SourceAssemblyBindingOptions(std::uint64_t instance, source::MaterialRatePolicy policy)
        : source_instance_id(instance), material_rate_policy(policy) {}
    std::uint64_t source_instance_id;
    source::MaterialRatePolicy material_rate_policy;
    tl::fea::ShellHostBindingLimits shell_limits{}, material_limits{};
    tl::fea::NodalRigidGroupLimits rigid_limits{};
};
enum class SourceAssemblyBindingStage { Input, Shells, Materials, RigidGroups };
class SourceAssemblyBindingError : public std::runtime_error {
  public:
    SourceAssemblyBindingError(SourceAssemblyBindingStage stage, unsigned status, std::string message,
                               std::uint64_t element=0, std::uint64_t part=0)
        : std::runtime_error(std::move(message)), stage(stage), status(status), source_element_id(element), source_part_id(part) {}
    const SourceAssemblyBindingStage stage;
    const unsigned status;
    const std::uint64_t source_element_id, source_part_id;
};
struct PartNativeMassLedger {
    std::uint64_t source_part_id=0;
    std::size_t parent_count=0, qeph_count=0, t3_count=0;
    // Ordered source-parent/local-node sums of the already prepared native
    // contributions, not a surface-mass formula or a second physical owner.
    tl::fea::ShellBindingMass native;
};

// Immutable case startup composition. TL remains the only producer of native
// references, mass/J and rigid-group properties. The full authenticated source
// accompanies the explicit instance ID; that ID is not a reduced source hash.
// All stages publish together. No FE owner, clock, trial state or dynamics API.
class SourceAssemblyBindings {
  public:
    static SourceAssemblyBindings Prepare(const source::SourceAssembly&, const SourceAssemblyBindingOptions&);
    SourceAssemblyBindings(const SourceAssemblyBindings&) noexcept=default;
    SourceAssemblyBindings(SourceAssemblyBindings&& other) noexcept
        : SourceAssemblyBindings(static_cast<const SourceAssemblyBindings&>(other)) {}
    SourceAssemblyBindings& operator=(const SourceAssemblyBindings&)=delete;
    SourceAssemblyBindings& operator=(SourceAssemblyBindings&&)=delete;
    const source::SourceAssembly& source() const noexcept;
    std::uint64_t source_instance_id() const noexcept;
    source::MaterialRatePolicy material_rate_policy() const noexcept;
    const tl::fea::ShellBatchBinding& shells() const noexcept;
    const tl::fea::ShellBatchPlasticityBinding& materials() const noexcept;
    const std::vector<PartNativeMassLedger>& part_mass_ledger() const noexcept;
    // Null only when the source declares no internal rigid groups. External
    // groups/welds/ties remain explicitly released in source().data().boundary.
    const tl::fea::NodalRigidGroupModel* rigid_groups() const noexcept;
  private:
    struct Impl;
    explicit SourceAssemblyBindings(std::shared_ptr<const Impl> impl): impl_(std::move(impl)) {}
    std::shared_ptr<const Impl> impl_;
};
} // namespace crash::cases::source_assembly
