#pragma once
#include "modelio/tied_shell/TiedShellDeclaration.h"
#include "lib_src/elements/solid18/Solid18Types.h"
#include "lib_src/elements/solid24/Solid24Types.h"
#include "lib_src/elements/solid6z/Solid6zForceTypes.h"
#include "lib_src/materials/law36/Types.h"
#include "lib_src/materials/law42/Types.h"
#include "lib_src/elements/solid18/law44/Types.h"
#include "lib_src/materials/law44/solid/Types.h"
#include "lib_src/materials/law90/Types.h"
#include "lib_src/elements/solid18/total_strain/ReferenceTypes.h"

namespace crash::modelio::solid_source {
namespace source = output::full_shell::source;
enum class Policy {
    OriginalAdhesive18RubberHephS6zV1,
    // Adds only rear-mount/antiroll rubber PIDs17/393/509/521. This is a
    // source/reference profile, not a complete connected-vehicle admission.
    OriginalAdhesive18ExtendedRubberHephS6zV2,
    OriginalAdhesive18ExtendedRubberRearLaw44V3,
    OriginalExtendedSolidsV4 // V3 plus original radiator foam, actual blank-HU LAW90.
};
enum class Family { Solid18, Solid24, Solid6z, Solid18Law44, Solid18Law90 };
enum class MaterialLaw { Law36, Law42, Law44, Law90 };
struct Limits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t member_bytes = 64 * 1024 * 1024, metadata_bytes = 1024 * 1024;
    std::size_t parents = 4096, nodes = 524288, source_solids = 16384, blocks = 16384;
    static Limits ExtendedSolids() { Limits result; result.parents = 8192; return result; }
};
struct Forecast {
    std::size_t canonical_bytes = 0, parsing_bytes = 0, geometry_bytes = 0;
    std::size_t reference_bytes = 0, fixed_bytes = 0, total_bytes = 0;
};
struct Part {
    std::uint64_t id = 0, section_id = 0, material_id = 0, hourglass_id = 0;
    std::array<std::size_t, 3> sources{}; // Original PART, SECTION, MATERIAL.
    std::size_t hourglass_source = SIZE_MAX, curve_source = SIZE_MAX;
    // Original ELFORM2 adhesive; blank rubber ELFORM and IHQ2/QM.1 remain raw.
    // Rubber converter Isolid1 is explicitly replaced by the named demo policy.
    unsigned converter_isolid = 0;
    MaterialLaw material_law = MaterialLaw::Law36;
    double density_kg_m3 = 0;
    tl::material::law36::Parameters law36;
    tl::material::law42::Parameters law42;
    tl::material::law44::solid::Parameters law44;
    tl::material::law90::PreparationInput law90_input;
    tl::material::law90::PreparedMaterial law90;
};
struct Row {
    std::uint64_t element_id = 0, part_id = 0;
    std::uint32_t canonical_row = 0, source_line = 0, part_index = 0;
    std::array<std::uint64_t, 8> raw_node_ids{};
    std::array<std::uint32_t, 8> canonical_nodes{};
    std::array<std::uint8_t, 6> six_to_raw{};
    Family family = Family::Solid18;
    std::size_t reference_index = SIZE_MAX;
    std::string raw_card;
};
struct Data {
    Policy policy = Policy::OriginalAdhesive18RubberHephS6zV1;
    std::vector<tied_shell::SourceEvidence> sources;
    std::vector<Part> parts; // Ascending original PID.
    std::vector<Row> rows; // Complete selected original solid record order.
    std::vector<std::uint32_t> canonical_nodes; // Ascending original NID order.
    std::vector<double> plastic_strain, yield_stress_pa; // Owned LAW36 curve.
    std::vector<tl::fea::solid18::Reference> solid18;
    std::vector<tl::fea::solid24::Reference> solid24;
    std::vector<tl::fea::solid6z::Reference> solid6z;
    tl::fea::solid6z::ForceProfile wedge_force_profile;
    std::vector<double> rear_plastic_strain, rear_yield_stress_pa;
    std::vector<tl::fea::solid18::law44::Reference> solid18_law44;
    std::vector<tl::fea::solid18::total_strain::Reference> solid18_law90;
    // Native curve ordinates before YFAC; original MPa values with scale1e6.
    std::vector<double> foam_compression_strain, foam_curve_ordinate;
    std::size_t original_solids = 0, outside_solids = 0;
    std::size_t owned_payload_bytes = 0;
};
// Original source/profile/reference admission only. Canonical backing stays
// alive, and every native mass comes from its typed reference. There is no
// physical domain, ledger, DOF, active history or owner admission in this value.
class VehicleSolidSource {
  public:
    static Forecast Preflight(const source::CanonicalSource&, Policy, Limits = {});
    static VehicleSolidSource Prepare(const source::CanonicalSource&, const std::string& member,
                                      Policy, Limits = {});
    VehicleSolidSource(const VehicleSolidSource&) noexcept = default;
    VehicleSolidSource(VehicleSolidSource&& other) noexcept : storage_(other.storage_) {}
    VehicleSolidSource& operator=(const VehicleSolidSource&) = delete;
    const source::CanonicalSource& canonical() const noexcept;
    const Data& data() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Storage;
    explicit VehicleSolidSource(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::solid_source
