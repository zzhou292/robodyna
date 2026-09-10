#pragma once

#include "qualification/source_contact/SourceShellCollection.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "SourcePartElasticWallMetrics.h"
#include "case/source_part_plastic/SourcePartPlasticState.h"
#include "case/source_part_plastic/SourcePartMaterial.h"
#include <array>
#include <memory>
#include <string>

namespace tlfea::contact { struct NodalWallDeviceResults; }
namespace crash::case_data { class CanonicalWall; }
namespace crash::cases::source_part_wall { class SourcePartWallSetup; struct SourcePartWallSettings; }

namespace crash::cases::source_part_elastic {
namespace source = crash::qualification::source_contact;
inline constexpr std::size_t NodeCount = source::NodeCount;

// A named source-part elastic experiment. Original source coordinates, density,
// thickness and parent identity are immutable; source MAT024 and attachments
// are retained as unapplied input metadata. The existing source adapter declares
// LAW1 E=200 GPa, nu=.3. None of these limits is a vehicle admission.
enum class Experiment { ElasticPulse,UniformFlight,MeshWallImpact };
enum class MaterialModel { ElasticLaw1,ExperimentalRateIndependentTabulatedJ2,SourceCowperSymonds };
struct Config {
    Experiment experiment = Experiment::ElasticPulse;
    MaterialModel material_model = MaterialModel::ElasticLaw1;
    source_part_plastic::SourcePartMaterial material;
    tl::material::TabulatedShellPlasticityRate rate;
    std::array<double,3> initial_velocity{};
    double dt = 0, pulse_duration = 0, acceleration = 0;
    unsigned spatial_axis = 0;
    std::array<double,3> direction{0,0,1};
    double maximum_displacement = 0, maximum_rotation = 0;
    double maximum_strain = 0, maximum_thickness_curvature = 0;
    double minimum_area_ratio = 0, maximum_area_ratio = 0;
    double minimum_thickness_ratio = 0, maximum_thickness_ratio = 0;
    double maximum_energy_residual = 0; // Absolute J floor in the residual allowance.
    double relative_energy_residual = 0; // Pulse: absolute drift work; wall: fixed initial K0.
    double maximum_native_dt_fraction = 0;
    std::uint64_t configuration_id = 0, qualification_id = 0;
};

enum class Status { Ok, InvalidInput, NotInitialized, AlreadyInitialized,
                    ComponentFailure, DeviceFailure, EnvelopeFailure };
struct Report {
    Status status = Status::InvalidInput;
    const char* message = "Invalid source-part case request";
    std::uint64_t source_parent_id = 0;
    double measured = 0, limit = 0;
    explicit operator bool() const noexcept { return status == Status::Ok; }
};

struct Diagnostics {
    tl::fea::ShellBatchDiagnostics shells;
    double external_kick_work = 0;       // Cumulative actual kick force work.
    double external_drift_work = 0;      // Cumulative endpoint-trapezoidal F.dx.
    double absolute_external_drift_work = 0; // Sum of absolute per-node-component trapezoid work.
    double kinetic_work_residual = 0;    // THIS interval's carried K change minus all kick work.
    double kinetic_work_allowance = 0;   // Arithmetic roundoff, not a physical tolerance.
    double synchronized_kinetic = 0;     // Reconstructed endpoint v/omega, total native J.
    double total_internal_work = 0;      // Native EINT(0)+EINT(1)+QEPH EVIS, counted once.
    double energy_residual = 0;          // Ksync + native work + wall potential - external work - K0.
    double max_relative_displacement = 0; // Translation removed using native mass centroid.
    double maximum_chord_change = 0;     // Capture-only pairwise length change; zero in step diagnostics.
    double maximum_rotation = 0;
    double maximum_area_ratio = 1, maximum_thickness_ratio = 1;
};
struct Snapshot {
    std::array<double,3*NodeCount> position{}, velocity{}, omega{};
    // Derived comparison fields from the complete endpoint RHS. Raw fields
    // above retain the owner's actual velocity phase and are never replaced.
    std::array<double,3*NodeCount> synchronized_velocity{}, synchronized_omega{};
    std::array<double,4*NodeCount> orientation{};
    tl::fea::NodalStamp stamp;
    Diagnostics diagnostics;
    source_part_plastic::PlasticSummary plastic;
};

bool ValidConfig(const Config&) noexcept;
// C1 temporal profile with exact zero outside [0,T]. Loads use accepted-base
// time. Spatial shape is xi^2 minus its native-mass weighted mean, xi in [-1,1].
double PulseScale(double time, double duration) noexcept;

// Application composition only: one TL owner/clock, two existing typed shell
// batches and their existing atomic publication coordinator. Every candidate
// readback and numerical check precedes the sole commit. Calls are serialized;
// initialization allocates bounded storage once, stepping allocates nothing.
class SourcePartElasticCase {
  public:
    SourcePartElasticCase();
    ~SourcePartElasticCase();
    SourcePartElasticCase(const SourcePartElasticCase&) = delete;
    SourcePartElasticCase& operator=(const SourcePartElasticCase&) = delete;
    Report Initialize(const source::SourcePartContactFixture&, const Config&);
    Report Initialize(const source::SourcePartContactFixture&,const Config&,const case_data::CanonicalWall&,
        const std::string& authenticated_wall_bytes,const source_part_wall::SourcePartWallSettings&);
    Report Step();
    Report Capture(Snapshot*);
    Report CapturePlasticSectionHistory(source_part_plastic::SourcePartPlasticState*) const;
    bool initialized() const noexcept;
    tl::fea::FENodalState& owner() noexcept;
    const source::SourcePartContactFixture& source() const noexcept;
    const source::SourceShellCollection& collection() const noexcept;
    const tl::fea::ShellBatchBinding& binding() const noexcept;
    const Config& config() const noexcept;
    const Diagnostics& diagnostics() const noexcept;
    double initial_kinetic_energy() const noexcept;
    const source_part_wall::SourcePartWallSetup* wall_setup() const noexcept;
    const tlfea::contact::NodalWallDeviceResults* accepted_contact() const noexcept;
    const WallMetrics* wall_metrics() const noexcept;
    tl::fea::NodalAllocationInfo allocations() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend struct SourcePartElasticTestAccess;
};
} // namespace crash::cases::source_part_elastic
