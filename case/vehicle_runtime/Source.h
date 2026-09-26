#pragma once
#include "SourceIdentity.h"
#include "SourceRoles.h"
#include "Packing.h"
#include <array>
#include <memory>
namespace tl::fea::type45 { class Model; }
namespace crash::cases::vehicle_wall::native { class EnvelopeOwnerSource; }
namespace crash::cases::vehicle_startup::joints { class VehicleJointModel; }
namespace crash::cases::vehicle_runtime {
enum class SourceKind { OriginalVehicle, VehicleWithEnvironment };
// Immutable authenticated source graph for the existing physical runtime.
// Case declarations stay in their real original/environment handles. This
// common view creates no owner, participant, trial, contact receipt or clock.
class Source {
  public:
    static Source Original(const Execution&, const Attachments&,
        const vehicle_startup::joints::VehicleJointModel* = nullptr);
    static Source WithEnvironment(const vehicle_wall::native::EnvelopeOwnerSource&);
    SourceKind kind() const noexcept;
    const tl::fea::ShellPhysicalBinding& physical() const noexcept;
    const tl::fea::NodalCoefficientLedger& coefficients() const noexcept;
    const tl::fea::NodalRigidAssemblyBinding& rigid() const noexcept;
    const tl::fea::solids::Model& solids() const noexcept;
    const tl::fea::type13::Model& beams() const noexcept;
    const tl::fea::beam18::Model* structural_beams() const noexcept;
    const tl::fea::type45::Model* joints() const noexcept;
    const vehicle_startup::TiedCinAttachments& cin() const noexcept;
    const vehicle_startup::TiedCinWitnessRoster& witnesses() const noexcept;
    tl::fea::NodalCinWitnessSource witness_source() const noexcept;
    tl::fea::ShellBatchStartup startup() const noexcept;
    SourceRoles roles() const;
    detail::OwnerPacking PackOwner(const SourceRoles&, std::size_t cap) const;
    std::uint8_t translation_fixed(std::size_t node) const noexcept;
    std::uint8_t rotation_fixed(std::size_t node) const noexcept;
    std::size_t vehicle_nodes() const noexcept;
    std::size_t retained_host_upper_bound(std::size_t cap) const;
    std::size_t additional_joint_source_bytes() const noexcept;
    std::array<std::size_t,3> construction_peaks() const noexcept;
    // Legacy adapters are deliberately partial: a combined source cannot be
    // passed off as the original VehiclePhysicalModel or original execution.
    const Execution& original_execution() const;
    const Attachments& original_attachments() const;
    const vehicle_wall::native::EnvelopeOwnerSource* environment() const noexcept;
  private:
    struct Data;
    explicit Source(std::shared_ptr<const Data> value) : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};
}
