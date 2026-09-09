#pragma once

#include "elements/ReissnerShellMass.h"

#include <array>
#include <cstddef>
#include <memory>
#include <string>

namespace crash::reference {

inline constexpr std::size_t kCouponNodes = 6;
inline constexpr std::size_t kCouponElements = 2;
inline constexpr std::size_t kCouponFreeDofs = 24;
inline constexpr std::array<std::size_t, 4> kCouponFreeNodes{{0, 3, 4, 5}};

struct ElasticCouponConfiguration {
    std::array<tl::fea::reissner::Vec3, kCouponNodes> position{};
    std::array<tl::fea::reissner::Quaternion, kCouponNodes> rotation{};
};

// Fixed, declared synthetic fixture; this is not a general material/model input
// format. The owning model publishes only a const reference after one Setup.
struct ElasticCouponData {
    static constexpr double length = .2;
    static constexpr double width = .1;
    static constexpr double thickness = .02;
    static constexpr double young_modulus = 1.2e6;
    static constexpr double poisson_ratio = .3;
    static constexpr double density = 1000;
    static constexpr double initial_tip_displacement = .002;
    ElasticCouponConfiguration reference_configuration;
    std::array<std::array<std::size_t, 4>, kCouponElements> connectivity{};
    std::array<tl::fea::reissner::ShellReference, kCouponElements> reference{};
    std::array<tl::fea::reissner::ElasticSection, kCouponElements> section{};
    std::array<tl::fea::reissner::ShellMass, kCouponElements> element_mass{};
    // Shared-node contributions summed once. J_d=J gives total J*I, while
    // physical tangential and artificial drilling values remain separate.
    std::array<tl::fea::reissner::ShellNodalMass, kCouponNodes> nodal_mass{};
    std::array<bool, kCouponNodes> fixed{{false, true, true, false, false, false}};
    std::array<double, kCouponNodes> inverse_mass{};
    std::array<double, kCouponNodes> inverse_isotropic_inertia{};
};

struct ElasticCouponEvaluation {
    std::array<tl::fea::reissner::Vec3, kCouponNodes> force{};
    std::array<tl::fea::reissner::Vec3, kCouponNodes> couple{};  // WORLD, including fixed-node reactions.
    std::array<tl::fea::reissner::ShellResult, kCouponElements> element{};
    double energy = 0;
    double bending_energy = 0;
};

// Immutable startup pose, applied before the donor's one Setup call. The exact
// cyclic quaternion (.5,.5,.5,.5) uses coordinate permutation (x,y,z)->(z,x,y).
struct ElasticCouponPose {
    tl::fea::reissner::Quaternion rotation;
    tl::fea::reissner::Vec3 translation;
};

enum class ElasticCouponStatus {
    kSuccess,
    kInvalidConfiguration,
    kForceFailure,
    kNonfiniteResult,
    kModalFailure,
    kAuditRejected
};

// Startup/reference oracle only: no state owner, integrator, clock or accepted
// frames. Uses actual coherent Chrono forces and immutable copied setup; never
// requests the deliberately unqualified analytic tangent. Evaluations mutate
// only private Chrono scratch, so serialize calls on each model instance.
// Prescribed evaluation may move fixed nodes (e.g. a rigid covariance check);
// the dynamics coordinator, not this reference oracle, enforces constraints.
// Constructor throws on unavailable/invalid setup; failed evaluations preserve
// the complete output and set a diagnostic. Success clears the diagnostic.
class ElasticCouponModel {
  public:
    ElasticCouponModel();
    explicit ElasticCouponModel(const ElasticCouponPose&);
    ~ElasticCouponModel();
    ElasticCouponModel(const ElasticCouponModel&) = delete;
    ElasticCouponModel& operator=(const ElasticCouponModel&) = delete;

    const ElasticCouponData& data() const;
    ElasticCouponStatus EvaluateChrono(const ElasticCouponConfiguration& configuration,
                                      ElasticCouponEvaluation& output, std::string& diagnostic) const;
    ElasticCouponStatus EvaluateTL(const ElasticCouponConfiguration& configuration,
                                  ElasticCouponEvaluation& output, std::string& diagnostic) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Free vector order: nodes {0,3,4,5}, each x/y/z/world-spin-x/y/z. Uses the
// same checked world quaternion increment as TL nodal stepping. This operation
// changes only free nodes and stages output; base/output may be the same object.
ElasticCouponStatus ApplyElasticCouponIncrement(
    const ElasticCouponConfiguration& base,
    const std::array<double, kCouponFreeDofs>& increment, double scale,
    ElasticCouponConfiguration& output, std::string& diagnostic);

}  // namespace crash::reference
