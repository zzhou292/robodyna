#pragma once
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include "lib_src/collision/radioss_type25/source_nodal/Types.h"
#include "lib_src/collision/radioss_type25/CoefficientTypes.h"
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace crash::modelio::native_spring_ids { struct ImportMembers; }

namespace crash::cases::vehicle_self_contact::native::nodal_seed {
namespace n = tlfea::contact::radioss_type25;
using PhysicalModel = vehicle_startup::physical_model::VehiclePhysicalModel;
using JointModel = vehicle_startup::joints::VehicleJointModel;

enum class ContributorKind {
    Solid18Law36, HephLaw42, PentaLaw42, Solid18Law44, Solid18Law90,
    Beam18, Type13, Type25, Type45, ShellQ4, ShellT3
};
enum class Channel { Volume, DirectStiffness, ShellAverage };

// Descriptive source order. Native IDs are produced from the authenticated
// import context, never inferred from a generated weld/joint's original ID.
struct Contributor {
    ContributorKind kind = ContributorKind::Solid18Law36;
    Channel channel = Channel::Volume;
    std::uint64_t original_id = 0, native_id = 0;
    std::uint64_t part_id = 0, material_id = 0;
    std::size_t source_index = SIZE_MAX;
    std::array<std::uint32_t, 8> nodes{};
    unsigned slots = 0;
};
struct Counts {
    std::size_t nodes = 0, shells = 0, solids = 0;
    std::array<std::size_t, 5> solid_families{};
    std::size_t beams = 0, type13 = 0, type25 = 0, type45 = 0;
    std::size_t namespace_only_springs = 0;
    std::size_t volume_occurrences = 0, stiffness_occurrences = 0;
    std::size_t shell_occurrences = 0;
};
struct InteriorPart {
    std::uint64_t part_id = 0, section_id = 0, material_id = 0;
    std::size_t retained_solids = 0, original_solids = 0;
};
struct InteriorDisposition {
    // Direct requested CONTACT_INTERIOR part-set membership only. Native
    // property sharing can extend effective Icontrol to other parts; that
    // association is not resolved by these counts. No PM107/correction or
    // mechanics enable switch is supplied here.
    std::vector<InteriorPart> parts;
    std::size_t retained_solids = 0, original_solids = 0;
    std::string combine_sha256, auxiliary_sha256;
};
struct PreCorrectionFields {
    double shell_young_thickness_sum = 0;
    double stiffness_before_control = 0;
    int shell_incidence_count = 0;
};
struct Provenance {
    std::string canonical_sha256, import_source_digest, spring_mapping_digest;
    std::string contributor_digest;
    n::UnitScale units;
};
struct Limits {
    // Includes retained model and joint backing, plus all simultaneous startup
    // arrays. This reservation is not a process RSS or driver-memory promise.
    std::size_t host_bytes = std::size_t{8} << 30;
    std::size_t nodes = 524288, shells = 524288, solids = 16384;
    std::size_t springs = 16384, beams = 1024;
    std::size_t metadata_bytes = 1u << 20;
};
struct Forecast {
    std::size_t physical_reservation = 0, joint_additional = 0;
    std::size_t source_context = 0, resolution_workspace = 0, roster_bytes = 0;
    std::size_t value_inputs = 0, seed_output = 0, nodal_output = 0;
    std::size_t numeric_scratch = 0, context_scratch = 0, fixed_bytes = 0;
    std::size_t peak_bytes = 0;
    Counts counts;
};

// Complete declared V5 startup operands and ASSTIFI result BEFORE the native
// distortion-control correction. This type cannot supply a runtime main/secondary
// source: it has no ready-interface flag, gap product, owner, epoch or clock.
// Prepare privately authenticates the complete import closure and resolves SPRING
// IDs; a caller-populated Resolution is not accepted as source authority.
class PreCorrectionNodalSource {
  public:
    static Forecast Preflight(const PhysicalModel&, const JointModel&,
        const modelio::native_spring_ids::ImportMembers&, Limits = {});
    static PreCorrectionNodalSource Prepare(const PhysicalModel&, const JointModel&,
        const modelio::native_spring_ids::ImportMembers&, Limits = {});
    const PhysicalModel& physical() const noexcept;
    const JointModel& joints() const noexcept;
    const Forecast& forecast() const noexcept;
    const Counts& counts() const noexcept;
    const Provenance& provenance() const noexcept;
    const InteriorDisposition& interior() const noexcept;
    const std::vector<Contributor>& contributors() const noexcept;
    n::NativeNodalSeedView seed() const noexcept;
    tl::util::ConstView<PreCorrectionFields> fields() const noexcept;
  private:
    struct Data;
    explicit PreCorrectionNodalSource(std::shared_ptr<const Data> data)
        : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed
