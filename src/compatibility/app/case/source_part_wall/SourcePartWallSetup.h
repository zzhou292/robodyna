#pragma once
#include "SourcePartContactGeometry.h"
#include "case/PlacedCanonicalWall.h"
#include "lib_src/elements/ShellBatchBinding.h"
#include "lib_src/solvers/FENodalState.h"
#include <memory>

namespace crash::cases::source_part_wall {
struct SourcePartWallSettings {
    std::array<double,3> initial_velocity{1,0,0};
    double leading_gap=.0005,area_floor=1e-5,design_penetration=.000375,penetration_cap=.0005;
    double kinetic_budget_factor=1.10,motion_margin=.020,exposed_clearance=1e-6;
    double parent_force_error=1e-6,parent_energy_error=1e-9,maximum_step_rate=.125;
    std::uint64_t configuration_id=0,qualification_id=0,wall_binding_id=0;
};
struct SourcePartWallCertificate {
    contact::Q4CertifiedIntegral native_initial_kinetic;
    contact::Q4IntegralInterval leading_gap;
    std::array<double,3> initial_velocity{};
    double measured_initial_kinetic=0,fixed_dt=0;
    double kinetic_budget_upper=0,stiffness_per_area=0,design_potential_lower=0;
    double minimum_nodal_area_lower=0;
    unsigned minimum_area_node=UINT32_MAX,kappa_upward_steps=0;
    contact::PlanarWallBoxCoverage coverage;
};
enum class SourcePartWallStatus { Ok,InvalidInput,AlreadyInitialized,NotInitialized,
    GeometryFailure,CertificateFailure,MassMismatch,StepLimit,ResourceLimit,DeviceFailure,ContributorFailure };
struct SourcePartWallReport {
    SourcePartWallStatus status=SourcePartWallStatus::InvalidInput;
    const char* message="Invalid source wall request";
    unsigned node=UINT32_MAX,parent=UINT32_MAX;
    explicit operator bool() const noexcept {return status==SourcePartWallStatus::Ok;}
};

// Host-only immutable preparation. The calling case authenticates this stamp
// through its live owner and both native shell startup bindings. Supplied
// inverse masses must match 1/native M bitwise; the actual contributor checks
// them against the owner's device mass before any assembly write. No alternate
// lumping or mass normalization is introduced. The initial energy enclosure is
// independently computed from native masses and declared uniform translation;
// the case's measured common K0 must lie inside it before publication.
class SourcePartWallSetup {
  public:
    SourcePartWallSetup();
    ~SourcePartWallSetup();
    SourcePartWallSetup(const SourcePartWallSetup&)=delete;
    SourcePartWallSetup& operator=(const SourcePartWallSetup&)=delete;
    SourcePartWallReport Initialize(const source::SourcePartContactFixture&,const tl::fea::ShellBatchBinding&,
        const tl::fea::NodalStamp&,const double* native_inverse_mass,double measured_initial_kinetic,
        const case_data::CanonicalWall&,
        const std::string& authenticated_wall_bytes,const SourcePartWallSettings&);
    bool initialized() const noexcept;
    const SourcePartWallSettings* settings() const noexcept;
    const SourcePartWallCertificate* certificate() const noexcept;
    const SourcePartContactGeometry* source_geometry() const noexcept;
    const case_data::PlacedCanonicalWall* placed_wall() const noexcept;
    const tl::fea::NodalStamp* owner_stamp() const noexcept;
    const double* inverse_mass() const noexcept;
    const double* initial_positions() const noexcept;
    const std::uint8_t* translation_fixed_bits() const noexcept;
  private:
    struct Data;
    std::unique_ptr<const Data> data_;
};
// Shared host enclosure of h*sqrt(lambda). Does not select a structural step
// or prove a combined history/contact recurrence. Failure preserves output.
SourcePartWallReport CheckSourcePartContactStep(double dt,double stiffness_rate,double maximum_step_rate,
                                               double* upper);
} // namespace crash::cases::source_part_wall
