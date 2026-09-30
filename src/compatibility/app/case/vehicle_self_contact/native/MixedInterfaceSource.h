#pragma once
#include "InitialSurfaceSource.h"
#include "lib_src/collision/RadiossType25InterfaceSurface.h"

namespace crash::cases::vehicle_self_contact::native::mixed_interface {
namespace f = tlfea::contact::radioss_type25::surface_interface;
namespace s = tlfea::contact::radioss_type25::startup;
using Initial = initial_surfaces::InitialSurfaceSource;
enum class Status { Ready, InvalidInput, UnsupportedSource, ResourceLimit, NeedsNativeReaderOrder };
enum class Stage { ClassifiedAndSidesBeforeSupport };
enum class NumericalStage { None, InterfacePreflight, InterfaceBuild, SidesPreflight, SidesBuild };
struct Report {
    Status status = Status::InvalidInput;
    std::string reason;
    std::size_t raw_face = SIZE_MAX;
    std::uint64_t source_element = 0;
    NumericalStage numerical_stage = NumericalStage::None;
    f::Report interface_report;
    s::Report sides_report;
};
struct Limits {
    // Inclusive source plus new stage; still below the unchanged 10GiB guard.
    std::size_t host_bytes = (std::size_t{19} << 29);
    std::size_t metadata_bytes = 1u << 20;
};
struct Forecast {
    std::size_t initial_source_reservation = 0, packed_inputs = 0;
    std::size_t source_role_workspace = 0, interface_output = 0, shared_scratch = 0;
    std::size_t sides_output = 0, digest_and_report = 0, peak_bytes = 0;
};
struct RoleObservation {
    int native_role = 0;
    // Unique IN24 membership witness only; NOT later IELEM_M support/removal.
    // Zero for solid input and ordinary shell. No representative table row escapes.
    std::uint64_t unique_coating_solid_eid = 0;
};
struct Certificate {
    std::size_t raw_shells = 0, raw_solids = 0, unique_coatings = 0, ordinary_shells = 0;
    std::size_t filtered_primaries = 0, shell_primaries = 0, solid_primaries = 0;
    std::size_t multi_origin_primaries = 0, coalesced_origins = 0;
    bool unique_selected_membership = false, complete_origins = false, complete_solid_flags = false;
};
struct Provenance {
    Stage stage = Stage::ClassifiedAndSidesBeforeSupport;
    std::string source_digest, initial_digest, output_digest;
    bool native_reader_ordinals_available = false;
};
struct MassExample {
    std::uint64_t source_node_id = 0;
    std::uint32_t domain_node = 0;
    double raw_mass_kg = 0;
    bool rigid_member = false, tied_source_candidate = false;
};
struct AdmissionCensus {
    std::size_t nodes = 0, positive_mass = 0, zero_mass = 0, negative_mass = 0, nonfinite_mass = 0;
    std::size_t rigid_groups = 0, rigid_members = 0, nonpositive_rigid_members = 0;
    std::size_t tied_source_candidates = 0, nonpositive_tied_candidates = 0;
    // Fixed bounded examples; no unbounded output or second physical model.
    std::array<MassExample, 32> mass_examples{};
    std::size_t mass_example_count = 0;
    // QEPH/T3/QBAT rows by None/ConstantAllPoints/Tab1AnyPoint SOURCE policy.
    std::array<std::array<std::size_t, 3>, 3> shell_failure_policies{};
    bool actual_rigid_binding_available = false;
    bool finalized_cin_available = false; // Source candidates are not actual CIN.
};
struct Preparation;
// Source-only IN24/filter/SH2 handle. Complete source origins remain in initial().
// Neither this success nor Main's zero neighbor fields admit normals, K/gaps,
// support/removal ownership, physical failure, or a runtime transaction.
class MixedInterfaceSource {
  public:
    MixedInterfaceSource(const MixedInterfaceSource&) noexcept = default;
    MixedInterfaceSource(MixedInterfaceSource&& other) noexcept : data_(other.data_) {}
    MixedInterfaceSource& operator=(const MixedInterfaceSource&) = delete;
    static Forecast Preflight(const Initial&, Limits = {});
    static Preparation Prepare(const Initial&, Limits = {});
    const Initial& initial() const noexcept;
    const s::MixedSidesSnapshot& sides() const noexcept;
    const s::PrimaryFace* primary() const noexcept;
    const std::vector<RoleObservation>& raw_roles() const noexcept;
    const Certificate& certificate() const noexcept;
    const Provenance& provenance() const noexcept;
    const Forecast& forecast() const noexcept;
    // Retained source payload bound; excludes this producer's retired scratch.
    std::size_t retained_host_upper_bound(std::size_t cap) const;
    const AdmissionCensus& admission_census() const noexcept;
  private:
    struct Data;
    explicit MixedInterfaceSource(std::shared_ptr<const Data> value) : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};
struct Preparation {
    Report report;
    std::optional<MixedInterfaceSource> source;
};
output::Document ForecastDocument(const Forecast&);
output::Document ResultDocument(const Preparation&, std::size_t cap = 1u << 20);
}
