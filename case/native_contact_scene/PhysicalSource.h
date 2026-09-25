#pragma once
#include "modelio/native_contact_scene/DeclaredSource.h"
#include "modelio/native_contact_scene/MaterialBridge.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include "lib_src/constraints/tied_shell/TiedCinAttachmentModel.h"
#include "lib_src/elements/ShellBatchStartup.h"
namespace crash::cases::native_scene {
struct SourceLimits {std::size_t nodes=2048,parents=1024;};
// Immutable source handles only: no current nodal buffers, material trial,
// physical owner, simulation clock or captured native coefficient arrays.
class PhysicalSource {
  public:
    static PhysicalSource Prepare(const modelio::native_scene::DeclaredSource&,std::uint64_t source_instance,SourceLimits={});
    const modelio::native_scene::DeclaredSource& declared() const noexcept;
    const modelio::native_scene::LinearHardeningBridge& hardening() const noexcept;
    const tl::fea::ShellPhysicalBinding& physical() const noexcept;
    const tl::fea::NodalRigidAssemblyBinding& rigid() const noexcept;
    const tl::constraints::tied_shell::TiedCinAttachmentModel& cin() const noexcept;
    const tl::fea::ShellBatchStartup& startup() const noexcept;
    const std::vector<std::uint8_t>& translation_fixed_bits() const noexcept;
    const std::vector<std::uint8_t>& rotation_fixed() const noexcept;
  private:
    struct Data;
    explicit PhysicalSource(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::native_scene
