#pragma once
#include "CorrectedNodalSource.h"
#include "CoatedSource.h"
#include "lib_src/collision/RadiossType25SurfaceSource.h"
#include <optional>

namespace crash::cases::vehicle_self_contact::native::initial_surfaces {
namespace values = tlfea::contact::radioss_type25::source_surfaces;
using Context = nodal_correction::CorrectedNodalSource;
using Selection = modelio::self_contact::OriginalSelection;
enum class Status { Ready, InvalidInput, UnsupportedSource, ResourceLimit, NeedsNativeReaderOrder };
enum class Stage { InitialClauseBeforeI25Classification };
struct Identity {
    values::ParentKind kind = values::ParentKind::Solid;
    std::uint64_t element_id = 0, part_id = 0;
    std::uint32_t canonical_row = 0, source_line = 0;
    std::uint8_t solid_face = 0; // Native raw face1..6; shell0. Not a new EID.
};
struct Face {
    Identity source;
    std::array<std::uint32_t, 4> nodes{}; // Existing physical-domain indices.
    int raw_role = 0; //1/3/7 before IN24/I25SURFI/SH2; not final MSEGTYP.
};
struct Certificate {
    std::size_t queried_solid_faces = 0, matching_physical_shells = 0;
    std::size_t equal_node_key_groups = 0;
    bool membership_complete = false, sort_order_complete = false;
};
struct Census {
    std::size_t nodes = 0, physical_shells = 0, physical_solids = 0;
    std::size_t original_selected_solids = 0, omitted_selected_solids = 0;
    values::Counts extraction;
    std::size_t faces = 0, quad_faces = 0, triangle_faces = 0;
};
enum class NumericalStage { None, ProbePreflight, PartPreflight, ProbeBuild, PartBuild };
struct Report {
    Status status = Status::InvalidInput;
    std::string reason;
    std::uint64_t solid_element = 0, first_candidate_element = 0, conflicting_candidate_element = 0;
    std::uint8_t solid_face = 0;
    NumericalStage numerical_stage = NumericalStage::None;
    // The lower value producer's row is a representative borrowed table index.
    // It is not a source EID or an authenticated native reader ordinal.
    values::Report numerical;
};
struct Provenance {
    Stage stage = Stage::InitialClauseBeforeI25Classification;
    std::string source_digest, selection_digest, input_digest, output_digest;
    // Representative physical table traversal is private and is NOT native
    // reader ELEM or later storage order. Only typed external identities escape.
    bool native_reader_ordinals_available = false;
    std::string control_rule;
};
struct Limits {
    std::size_t host_bytes = std::size_t{8} << 30;
    std::size_t nodes = 524288, shells = 524288, solids = 16384, parts = 4096;
    std::size_t metadata_bytes = 1u << 20;
};
struct Forecast {
    std::size_t upstream_geometry_reservation = 0, context_reservation = 0;
    std::size_t packed_inputs = 0, certificate_workspace = 0, extraction_output = 0;
    std::size_t extraction_scratch = 0, retained_faces = 0, digest_workspace = 0;
    std::size_t peak_bytes = 0;
    std::size_t maximum_faces = 0;
};
struct Preparation;
// Immutable source-backed initial surface buffer, NOT an interface/runtime
// source. Corrected K is never consumed as geometry/main K. Complete physical
// shell lookup context and node ordering are authenticated before certification.
class InitialSurfaceSource {
  public:
    InitialSurfaceSource(const InitialSurfaceSource&) noexcept = default;
    InitialSurfaceSource(InitialSurfaceSource&& other) noexcept : data_(other.data_) {}
    InitialSurfaceSource& operator=(const InitialSurfaceSource&) = delete;
    static Forecast Preflight(const Context&, const Selection&, Limits = {});
    static Preparation Prepare(const Context&, const Selection&, const std::string& original_member, Limits = {});
    const Context& context() const noexcept;
    const Selection& selection() const noexcept;
    const coated::Inputs& geometry() const noexcept;
    const std::vector<Face>& faces() const noexcept;
    const std::vector<std::uint8_t>& emitted_solid_flags() const noexcept;
    const Certificate& certificate() const noexcept;
    const Census& census() const noexcept;
    const Provenance& provenance() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Data;
    explicit InitialSurfaceSource(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
struct Preparation {
    Report report;
    std::optional<InitialSurfaceSource> source;
};
output::Document ResultDocument(const Preparation&, std::size_t cap = 1u << 20);
output::Document ForecastDocument(const Forecast&);
}
