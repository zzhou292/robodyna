#pragma once

#include "GuidedPlateCase.h"
#include <array>
#include <cstdint>
#include <string>

namespace crash::case_data {
inline constexpr std::size_t kGuidedStudySamples=201;
inline constexpr std::uint64_t kGuidedStudyStepCap=100000;
using GuidedStudyInterval=tlfea::contact::Q4IntegralInterval;
using GuidedStudyCertificate=tlfea::contact::Q4CertifiedIntegral;

struct GuidedStudyConfig {
    std::uint64_t owner_id=0,qualification_id=0,wall_binding_id=0,base_steps=0;
    unsigned refinement=0;
    double fixed_dt=0,horizon=0,initial_energy=0,wall_x=0;
    std::string experiment_sha256; // Shared immutable reference/mass/section serialization, not a source authentication claim.
    std::array<double,18> reference_position{};
    std::array<double,24> reference_rotation{};
    std::array<tlfea::contact::PreparedQ4PlanarParent,2> contact_reference{};
    GuidedStudyInterval total_reference_area;
    // Execution identity is separate from the physical experiment fingerprint.
    tlfea::contact::Q4PlanarIntegrationBackend integration_backend =
        tlfea::contact::Q4PlanarIntegrationBackend::ScalarDyadicSquares;
    reference::GuidedPlateExperiment experiment = reference::GuidedPlateExperiment::Original;
};
// Exactly ceil(j*base_steps/200)*refinement, without overflowing products.
// The admitted base count is >=200, so all 201 epochs are distinct.
bool GuidedStudySampleEpoch(const GuidedStudyConfig&,std::size_t j,std::uint64_t& output) noexcept;
bool PrepareGuidedStudyConfig(const GuidedPlateMetrics& initial,const reference::ElasticCouponData&,
                             const reference::GuidedPlateData&,tlfea::contact::Q4PlanarReferenceView,
                             unsigned refinement,GuidedStudyConfig& output,std::string& diagnostic);

enum class GuidedStudyEnergy : std::size_t { Translation, PhysicalRotation, ArtificialDrilling, Shell, Contact, Count };
struct GuidedStudySample {
    std::uint64_t epoch=0; double time=0;
    std::array<double,2> normal_displacement{},normal_velocity{},world_z_rotation{}; // tip {4,5}, middle {0,3}
    double curvature_proxy=0; // (tip - 2*middle + clamp)/(0.1m)^2, not a Gauss strain.
    GuidedStudyInterval minimum_signed_gap; // min_i(wall_x-x_i), outward enclosure, all six nodes.
    GuidedStudyInterval tip_normal_velocity;
    std::array<double,static_cast<std::size_t>(GuidedStudyEnergy::Count)> energy{};
    double shell_bending_energy=0,maximum_penetration=0;
    GuidedStudyCertificate normal_wall_force,contact_potential;
};
struct GuidedStudyEvent {
    bool observed=false;
    std::uint64_t lower_epoch=0,upper_epoch=0;
    double lower_time=0,upper_time=0;
};
struct GuidedStudySummary {
    std::uint64_t accepted_epoch=0,last_attempt=0;
    double accepted_time=0;
    std::size_t sample_count=0;
    // Max value and [max(all lowers),max(all uppers)] are intentionally
    // different statistics. Error encloses their actual endpoint distances.
    GuidedStudyCertificate sampled_peak_normal_force,normal_wall_impulse;
    double maximum_penetration=0;
    double maximum_value_energy_relative_error=0,maximum_certified_energy_relative_error=0;
    GuidedStudyEvent activation,pressure_release;
    bool certified_partial_area=false,certified_unequal_nodal_force=false,separated_rebounding=false;
    std::uint64_t separation_sample_epoch=0;
    double separation_sample_time=0;
    double minimum_curvature=0,maximum_curvature=0,maximum_abs_curvature=0;
    std::array<double,2> minimum_rotation{},maximum_rotation{};
};
struct GuidedStudyData {
    GuidedStudyConfig config;
    GuidedStudySummary summary;
    std::array<double,18> initial_position{};
    std::array<double,24> initial_rotation{};
    std::array<GuidedStudySample,kGuidedStudySamples> samples{};
    bool complete=false;
};

// Observer of accepted data only: no state/force mutation, integrator, clock,
// device read, file I/O or per-step allocation. Scalar staging plus at most one
// sample preserves all published data on failure; the 201-sample history is
// never copied per interval. Calls are externally serialized.
class GuidedPlateStudy {
 public:
    bool Initialize(const GuidedStudyConfig&,const GuidedPlateMetrics&,const GuidedPlateFrame&,std::string& diagnostic);
    // Require contiguous accepted endpoints. Supply a captured frame exactly
    // when NeedsSample is true; its endpoint cache may have been refreshed.
    bool Record(const GuidedPlateMetrics&,const GuidedPlateFrame* sample,std::string& diagnostic);
    bool NeedsSample(std::uint64_t accepted_epoch) const noexcept;
    const GuidedStudyData* data() const noexcept { return initialized_?&data_:nullptr; }
    // Missing separation is retained as an explicit outcome. It does not
    // prevent recording a completed horizon; comparison reports the gate fail.
    bool Finish(GuidedStudyData& output,std::string& diagnostic);
 private:
    GuidedStudyData data_;
    std::uint64_t last_inactive_epoch_=0,last_active_epoch_=0;
    double last_inactive_time_=0,last_active_time_=0;
    bool initialized_=false;
};

struct GuidedStudyComparison {
    bool passed=false;
    // Largest (difference + applicable uncertainty)/frozen acceptance budget.
    double displacement_ratio=0,velocity_ratio=0,rotation_ratio=0,force_ratio=0;
    double impulse_ratio=0,energy_ratio=0,event_ratio=0,penetration_ratio=0;
    bool energy_envelopes=false,deforming_contact_evidence=false,events_complete=false;
    std::string diagnostic;
};
// Valid input with a failed numerical gate still publishes a comparison with
// passed=false. Malformed/stale/mismatched input rejects and preserves output.
// Owner IDs are process-local and may match across independent report files.
// Physical experiment tables, initial geometry and execution backend must agree.
// Read-only completed-record validation for bounded StudyIO. No fake comparison.
bool ValidateGuidedPlateStudy(const GuidedStudyData&,std::string& diagnostic);
bool CompareGuidedPlateStudies(const GuidedStudyData& coarse,const GuidedStudyData& fine,
                              GuidedStudyComparison& output,std::string& diagnostic);
// Same-h wall response: distinct nonzero wall bindings, identical schedule and
// physical experiment/initial configuration and backend. The canonical response supplies
// the directed 5% scales. This function does not authenticate wall provenance:
// callers must separately validate regenerated wall/source sidecars and exact
// report-byte bindings. No sidecar/file I/O or new mechanics gate occurs here.
// Malformed inputs preserve output; valid failed comparisons publish all ratios.
bool CompareGuidedPlateWallStudies(const GuidedStudyData& derived,const GuidedStudyData& canonical,
                                  GuidedStudyComparison& output,std::string& diagnostic);
} // namespace crash::case_data
