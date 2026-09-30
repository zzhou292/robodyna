#pragma once
#include "PostGapmMainSource.h"
#include "lib_src/collision/radioss_type25/runtime/Types.h"
#include "case/vehicle_wall/native/WallSource.h"
#include "modelio/native_spring_ids/ImportContext.h"
namespace crash::cases::vehicle_self_contact::native::initial_controls {
namespace n=tlfea::contact::radioss_type25;
using MainSource=post_gapm::PostGapmMainSource;
using Wall=vehicle_wall::native::WallSource;
enum class Status { Ready,InvalidInput,UnsupportedSource,NeedsNativeNamespace,ResourceLimit,NonfiniteResult };
enum class InterfaceKind { Type2,Type25 };
enum class InterfaceOrigin { OriginalDefinition,DeclaredAdditionalInterface };
enum class ExecutionProfile { SerialDoubleExplicitFixedStep };
struct Controls {
    int level=1,gap_mode=1,initial_penetration=5,damping_flag=1,sharp=1;
    int arithmetic_precision=0,partitions=1,starter_workers=1,edge_mode=0,thermal=0,curvature=0;
    unsigned native_packet_size=128; // NVSIZ from selected ARCHINFO branch, not MVSIZ storage.
    int neighbor_removal=2,tied_removal=1,thickness_update=0,stiffness_formulation=4,stiffness_mass_update=0;
    bool gap_load_cards=false;
    double base_multiplier=static_cast<double>(.20f),drad=0,gap_load=0;
    std::uint64_t native_voxel_capacity=8000000;
};
struct RawControls {
    int source_soft=0,source_ignore=0,reader_gap_mode=0,reader_idel=0;
    int reader_tied_removal=0,reader_sharp=0,property_type=1;
    double reader_global_gap=0;
    double sti3_initial_gap_minimum=0;
    bool final_gap_minimum_available=false; // Recomputed I25STI3 output is not consumed/published here.
    double stfac=1,slsfac=1,dtstif=0,global_stiffness=-1,friction_viscosity=0;
    bool source_death_blank=false,sensor_disabled=true,stop_nonnegative=true;
    // Blank DT can retain its DYNA default; literal0 is normalized to EP30.
    // Both are active at TT0 and throughout the selected short case horizon.
    // No chosen bound is advertised as the exact native TSTOP value.
    bool exact_stop_time_available=false;
    double stop_time_lower_bound_native=1e20;
};
struct GapScalars {
    // I25GAPM's selected-shell GAPS2 reduction BEFORE its final cap and
    // I25INI_GAP_N. It excludes the separate solid VOL/AREA GAP_N1 channel.
    double pre_ini_main_maximum=0,secondary_maximum=0,global_search_gap=0;
    std::size_t shell_supports=0;
};
struct PopulationRange {
    // Exact cardinality is unavailable. This complete source-derived interval
    // is consumed only by a separately qualified same-multiplier-branch path.
    std::size_t lower=0,upper=0,complete_original_nodes=0,explicit_node_bound=0;
    std::size_t rigid_definition_bound=0,discrete_bound=0,transform_bound=0,rigid_wall_bound=0;
    bool exact_count_available=false;
};
struct Interface {
    InterfaceKind kind=InterfaceKind::Type25;
    InterfaceOrigin origin=InterfaceOrigin::OriginalDefinition;
    std::uint64_t native_id=0;
    std::uint32_t native_storage_ordinal=0;
    modelio::assembly::SourceBlock source;
};
struct Provenance {
    ExecutionProfile execution=ExecutionProfile::SerialDoubleExplicitFixedStep;
    std::string source_digest,main_digest,wall_digest,output_digest;
    std::size_t original_interfaces=0,declared_interfaces=0,checked_blocks=0;
};
struct Limits {
    std::size_t host_bytes=20000000000ULL;
    std::size_t interfaces=1024,blocks=8192,metadata_bytes=1u<<20;
    modelio::native_spring_ids::Limits import;
};
struct Forecast {
    std::size_t upstream_retained=0,wall_retained_bound=0,prior_peak=0;
    std::size_t import_workspace=0,namespace_workspace=0,output_values=0;
    std::size_t current_phase=0,retained_bytes=0,peak_bytes=0;
};
struct Report {Status status=Status::InvalidInput;std::string reason,file;std::size_t line=0;};
struct Preparation;
// Immutable source controls only. Genuine Starter normals, finalized TYPE2
// rows, generated rigid primary namespace and GPU seed remain separate stages.
class InitializerControlsSource {
  public:
    static Forecast Preflight(const MainSource&,const modelio::native_spring_ids::ImportMembers&,const Wall* =nullptr,Limits={});
    static Preparation Prepare(const MainSource&,const modelio::native_spring_ids::ImportMembers&,const Wall* =nullptr,Limits={});
    const MainSource& main() const noexcept;
    const Wall* wall() const noexcept;
    const Controls& controls() const noexcept;
    // Original converted law on the explicitly declared serial/fixed-step case.
    // It creates no transaction, participant identity or history authority.
    const n::TransactionConfig& self_runtime_controls() const noexcept;
    const RawControls& raw_controls() const noexcept;
    const Controls* wall_controls() const noexcept;
    const RawControls* wall_raw_controls() const noexcept;
    // Original self interface only. Wall uses its own complete secondary gap
    // reduction plus declared half thickness; these scalars are never shared.
    const GapScalars& gaps() const noexcept;
    const PopulationRange& native_population() const noexcept;
    tl::util::ConstView<Interface> interfaces() const noexcept;
    std::uint64_t self_interface_id() const noexcept;
    std::uint64_t wall_interface_id() const noexcept;
    const Provenance& provenance() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Data;
    explicit InitializerControlsSource(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
struct Preparation {Report report;std::optional<InitializerControlsSource> source;};
}
