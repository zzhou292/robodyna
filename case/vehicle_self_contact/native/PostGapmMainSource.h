#pragma once
#include "MixedInterfaceSource.h"
#include "ContactGapOperands.h"
#include "lib_src/collision/radioss_type25/startup/PostGapmTypes.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm {
namespace n = tlfea::contact::radioss_type25;
namespace s = n::startup;
using Mixed = mixed_interface::MixedInterfaceSource;
using GapOperands = gap_operands::ContactGapOperands;
enum class Status { Ready, InvalidInput, UnsupportedSource, ResourceLimit, NonfiniteResult, NeedsNativeReaderOrder, NeedsNativeShellOrder };
enum class Stage { PreparedSupportAndPreBucGaps };
struct Report {
    Status status = Status::InvalidInput;
    std::string reason;
    std::size_t primary = SIZE_MAX;
    std::uint64_t first_element = 0, second_element = 0;
};
struct PhysicalOwner {
    s::PhysicalSupportKind kind = s::PhysicalSupportKind::Unspecified;
    std::uint64_t source_element = 0, source_part = 0;
    // Original retained shell row or solid geometry row, according to kind.
    // This is not a native combined-table ordinal or a raw-origin choice.
    std::size_t physical_row = SIZE_MAX;
};
enum GeometryField : unsigned { Area = 1, FirstVolume = 2, SecondVolume = 4, SolidLength = 8, ExteriorProjection = 16 };
struct Geometry {
    unsigned defined = 0;
    double area = 0, first_volume = 0, second_volume = 0, solid_length = 0, exterior_projection = 0;
};
struct Counts {
    std::size_t primaries = 0, expanded = 0, shell_owners = 0, solid_owners = 0;
    std::size_t solid_matches[3]{}, pre_shell_internal = 0, final_internal = 0, primary_reversals = 0;
    std::size_t negative_first_volumes = 0, negative_second_volumes = 0, negative_exterior_first_volumes = 0;
    std::size_t positive_coefficients = 0, zero_coefficients = 0, negative_coefficients = 0;
};
struct Provenance {
    Stage stage = Stage::PreparedSupportAndPreBucGaps;
    n::UnitScale units;
    std::string source_digest, mixed_digest, gap_operand_digest, output_digest;
};
struct Limits {
    std::size_t host_bytes = std::size_t{10} << 30;
    std::size_t nodes = 524288, primaries = 524288, mains = 1048576, parts = 4096;
    std::size_t metadata_bytes = 1u << 20;
    n::source_gaps::Limits gaps;
};
struct Forecast {
    std::size_t mixed_retained = 0, gap_additional_retained = 0;
    std::size_t input_maps = 0, query_workspace = 0, output_values = 0;
    std::size_t temporary_mains = 0, gap_scratch = 0, metadata = 0;
    std::size_t mixed_input_construction_peak = 0, gap_input_construction_peak = 0;
    std::size_t prior_construction_peak = 0, current_phase = 0, peak_bytes = 0, retained_bytes = 0;
};
struct Preparation;
// Actual immutable source values/supports. It retains all mixed raw origins,
// and does not create neighbors/normals, removal/history, a physical owner or
// erosion update. Scalar GAPMIN/search controls remain separate authority.
class PostGapmMainSource {
  public:
    static Forecast Preflight(const Mixed&, const GapOperands&, Limits = {});
    static Preparation Prepare(const Mixed&, const GapOperands&, const std::string& combine_member, Limits = {});
    PostGapmMainSource(const PostGapmMainSource&) noexcept = default;
    PostGapmMainSource(PostGapmMainSource&& other) noexcept : data_(other.data_) {}
    PostGapmMainSource& operator=(const PostGapmMainSource&) = delete;
    const Mixed& mixed() const noexcept;
    const GapOperands& gap_operands() const noexcept;
    s::Input startup_input() const noexcept;
    s::PostGapmTopology post_gapm() const noexcept;
    tl::util::ConstView<double> coefficients() const noexcept;
    tl::util::ConstView<PhysicalOwner> primary_owners() const noexcept;
    tl::util::ConstView<Geometry> primary_geometry() const noexcept;
    tl::util::ConstView<std::uint32_t> secondary_nodes() const noexcept;
    tl::util::ConstView<std::uint32_t> main_nodes() const noexcept;
    tl::util::ConstView<double> secondary_gaps() const noexcept;
    tl::util::ConstView<double> main_node_gaps() const noexcept;
    // Exact I25INI_GAP_N output BEFORE I25BUC_VOX1 can normalize corner1.
    // Initializer must carry its finalized gaps forward; solid_length is separate.
    tl::util::ConstView<n::source_gaps::MainGapFields> main_gaps() const noexcept;
    const n::source_gaps::Profile& gap_profile() const noexcept;
    const n::source_gaps::Report& gap_report() const noexcept;
    const Counts& counts() const noexcept;
    const Provenance& provenance() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Data;
    explicit PostGapmMainSource(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
struct Preparation { Report report; std::optional<PostGapmMainSource> source; };
}
