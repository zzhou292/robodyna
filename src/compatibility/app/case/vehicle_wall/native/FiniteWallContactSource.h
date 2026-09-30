#pragma once
#include "EnvelopeOwnerSource.h"
#include "case/vehicle_self_contact/native/PostGapmMainSource.h"
#include "lib_src/collision/RadiossType25Transaction.h"
#include "lib_src/collision/RadiossType25SearchStartup.h"
namespace crash::cases::vehicle_wall::native::wall_interface {
namespace n=tlfea::contact::radioss_type25;
namespace s=n::startup;
using VehicleSource=vehicle_self_contact::native::post_gapm::PostGapmMainSource;
enum class Profile { Unspecified, AllRetainedVehicleNodesToFixedMeshV1 };
enum class Stage { PreparedBeforeGeneralInitialization };
enum class Status { Prepared, InvalidInput, SourceMismatch, UnsupportedProfile, ResourceLimit, NonfiniteResult };
enum class NumericalStage { None, WallComponent, SecondaryCoefficient, Starter, FixedReady, Gaps };
struct Declaration {
    Profile profile=Profile::Unspecified;
    std::uint64_t topology_generation=0, source_generation=0;
};
struct Controls {
    n::TransactionConfig runtime;
    n::source_gaps::Profile gaps;
    n::search_startup::Profile search;
    int level=1, solid_erosion=0;
    unsigned native_workers=1, force_packet_size=128;
};
struct Limits {
    std::size_t nodes=524288, shells=524288;
    std::size_t own_bytes=std::size_t{512}<<20;
    // Existing complete-owner source allowance. This is a conservative source
    // reservation, not permission to change the process/qualification guard.
    std::size_t coexistence_bytes=EnvelopeOwnerLimits{}.host_bytes;
    std::size_t metadata_bytes=1u<<20;
};
struct Forecast {
    std::size_t owner_retained=0, owner_prior_peak=0;
    std::size_t vehicle_retained=0, vehicle_prior_peak=0;
    std::size_t own_retained=0, coefficient_scratch=0, topology_scratch=0, gap_scratch=0;
    std::size_t constraint_packing=0, gap_shell_copy=0, digest_scratch=0;
    std::size_t own_peak=0, coexistence_peak=0, complete_construction_bound=0;
};
struct Report {
    Status status=Status::InvalidInput;
    std::string reason;
    NumericalStage numerical_stage=NumericalStage::None;
    n::CoefficientStatus coefficient_status=n::CoefficientStatus::InvalidInput;
    s::Report startup;
    n::source_gaps::Report gaps;
    std::size_t row=SIZE_MAX;
};
struct Provenance {
    Stage stage=Stage::PreparedBeforeGeneralInitialization;
    std::uint64_t interface_id=0;
    std::string source_digest, vehicle_digest, wall_digest, output_digest;
    bool all_retained_vehicle_additional_nodes=false;
};
struct Preparation;
// Owns source inputs for one new declared node-to-wall-mesh TYPE25 interface.
// Starter and fixed-ready normal phases stay separate. No final removal CSR,
// initial history, runtime receipt, mass model, owner or clock is created here.
class FiniteWallContactSource {
  public:
    static Forecast Preflight(const EnvelopeOwnerSource&,const VehicleSource&,Declaration,Limits={});
    static Preparation Prepare(const EnvelopeOwnerSource&,const VehicleSource&,Declaration,Limits={});
    FiniteWallContactSource(const FiniteWallContactSource&) noexcept=default;
    FiniteWallContactSource(FiniteWallContactSource&& other) noexcept:data_(other.data_){}
    FiniteWallContactSource& operator=(const FiniteWallContactSource&)=delete;
    const VehicleSource& vehicle_source() const noexcept;
    const WallSource& wall() const noexcept;
    const modelio::physical_scope::DomainEmbedding& embedding() const noexcept;
    const tl::fea::NodalNodeDomain& domain() const noexcept;
    const Declaration& declaration() const noexcept;
    const Controls& controls() const noexcept;
    const Provenance& provenance() const noexcept;
    const Forecast& forecast() const noexcept;
    s::Input startup_input() const noexcept;
    const s::Snapshot& starter() const noexcept;
    const s::FixedMainView& fixed_ready() const noexcept;
    tl::util::ConstView<n::lifecycle::Node> nodes() const noexcept;
    tl::util::ConstView<std::uint32_t> secondary_nodes() const noexcept;
    tl::util::ConstView<std::uint32_t> main_nodes() const noexcept;
    tl::util::ConstView<double> global_coefficients() const noexcept;
    tl::util::ConstView<double> secondary_coefficients() const noexcept;
    tl::util::ConstView<double> secondary_gaps() const noexcept;
    // Completed interface-specific GAPS_MN/GAPS_MX before general initialization.
    const n::source_gaps::Report& gap_report() const noexcept;
    tl::util::ConstView<double> main_coefficients() const noexcept;
    // I25INI_GAP_N values before BUC corner1 normalization.
    tl::util::ConstView<n::source_gaps::MainGapFields> main_gaps() const noexcept;
  private:
    struct Data;
    explicit FiniteWallContactSource(std::shared_ptr<const Data> value):data_(std::move(value)){}
    std::shared_ptr<const Data> data_;
};
struct Preparation {Report report;std::optional<FiniteWallContactSource> source;};
}
