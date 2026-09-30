#pragma once

#include "ShellPatchAudit.h"
#include "GuidedPlateExperiment.h"
#include "collision/PlanarWallGeometry.h"
#include "collision/Q4PlanarGeometry.h"
#include "collision/Q4PlanarStiffness.h"

namespace crash::reference {
inline constexpr std::size_t kGuidedPlateDofs = 16;

struct GuidedPlateData {
    static constexpr double initial_gap = .001;
    static constexpr double initial_tip_displacement = -.002;
    static constexpr double requested_horizon = .2;
    static constexpr double maximum_penetration = .0005;
    static constexpr double exposed_clearance = 1e-6;
    GuidedPlateExperiment experiment = GuidedPlateExperiment::Original;
    std::uint64_t qualification_id = kGuidedOriginalExperiment.qualification_id;
    double stiffness_per_area = kGuidedOriginalExperiment.stiffness_per_area;
    double target_penetration = kGuidedOriginalExperiment.target_penetration;
    ElasticCouponPose pose;
    std::uint64_t wall_binding_id = 0;
    std::array<std::uint8_t,kCouponNodes> translation_fixed_bits{{6,7,7,6,6,6}};
    std::array<std::uint8_t,kCouponNodes> rotation_fixed{{0,1,1,0,0,0}};
    std::array<tlfea::contact::SurfaceQ4,kCouponElements> parents{};
    tlfea::contact::Q4IntegrationLimits integration;
};

// Startup-only guided fixture. The same posed ElasticCouponModel owns the
// actual Chrono reference oracle and one immutable copied TL setup/mass table.
// No second element model, state, solver or clock. The exact cyclic pose puts
// the shell normal along +X; two short-edge nodes stay clamped. Other nodes
// retain world X translation and all three spins, with fixed Y/Z guides.
// Placement uses the supplied finite wall bounds, then requires actual C3
// coverage of BOTH parents. Holes/outside/exposed crossings reject; no search
// or silent repositioning occurs. Invalid construction throws before publication.
class GuidedPlateModel {
  public:
    GuidedPlateModel(tlfea::contact::PlanarWallView wall,std::uint64_t wall_binding_id,
                     GuidedPlateExperiment experiment=GuidedPlateExperiment::Original);
    ~GuidedPlateModel();
    GuidedPlateModel(const GuidedPlateModel&)=delete;
    GuidedPlateModel& operator=(const GuidedPlateModel&)=delete;
    const GuidedPlateData& data() const;
    const ElasticCouponModel& shell() const;
    const tlfea::contact::PlanarWallGeometry& wall() const;
    const tlfea::contact::Q4PlanarGeometry& contact_geometry() const;
    const tlfea::contact::Q4PlanarStiffness& contact_stiffness() const;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Nodes {0,3,4,5}, each {world X, world spin X,Y,Z}. No Y/Z perturbation.
patch_audit::Layout<kGuidedPlateDofs> GuidedPlateLayout();
ElasticCouponStatus ApplyGuidedPlateIncrement(
    const ElasticCouponConfiguration& base,const std::array<double,kGuidedPlateDofs>& increment,double scale,
    ElasticCouponConfiguration& output,std::string& diagnostic);
}  // namespace crash::reference
