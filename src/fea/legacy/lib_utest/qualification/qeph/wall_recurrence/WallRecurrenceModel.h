#pragma once
#include "lib_utest/qualification/qeph/free_response/RecurrenceAudit.h"
#include "lib_src/collision/NodalWallContact.h"
#include "lib_src/collision/Q4ParametricContact.h"

namespace tl::qualification::qeph::wall_recurrence {
namespace contact=tlfea::contact;
constexpr double ImpactSpeed=8,TargetDepth=.000375,MaximumDepth=.0005,InitialGap=TargetDepth/4;
constexpr double ParentForceError=5e-7,ParentEnergyError=1.2500000000000005e-12;
constexpr double MotionExtent=.05,WallClearance=1e-6,MassRateSpreadLimit=1e-12;
constexpr std::array<double,3> VelocityBaselines{{-ImpactSpeed,0,ImpactSpeed}};

// Immutable prescribed qualification model; no physical state, cache owner,
// clock or dynamics admission. Native material reference is at X=-InitialGap;
// the disposable native probe baseline is touching at X=0. Contact weights
// bind the former, independently of native structural mass and rotary inertia.
class WallRecurrenceModel {
 public:
  bool prepared() const { return prepared_; }
  const recurrence::Model& native() const { return native_; }
  const contact::Q4ParametricReference& reference() const { return reference_; }
  const contact::NodalWallWeights& weights() const { return weights_; }
  const contact::PlanarWallGeometry& wall() const { return wall_; }
  const contact::PlanarWallBoxCoverage& coverage() const { return coverage_; }
  const contact::NodalWallConfig& law() const { return law_; }
  const contact::NodalWallResult& touching() const { return touching_; }
  // rho, rho*t, rho*t*v, rho*t*v*v, previous/d, previous/d.
  const std::array<contact::Q4IntegralInterval,6>& penalty_chain() const { return penalty_chain_; }
  const std::array<contact::Q4CertifiedIntegral,recurrence::MaxNodes>& mass_rates() const { return mass_rates_; }
  double rate_spread_upper() const { return rate_spread_upper_; }
  double maximum_frequency() const { return maximum_frequency_; }
  unsigned coordinate(recurrence::Group,unsigned entity,unsigned component) const;
 private:
  recurrence::Model native_;
  contact::Q4ParametricReference reference_;
  contact::NodalWallWeights weights_;
  contact::PlanarWallGeometry wall_;
  contact::PlanarWallBoxCoverage coverage_;
  contact::NodalWallConfig law_;
  contact::NodalWallResult touching_;
  std::array<contact::Q4IntegralInterval,6> penalty_chain_{};
  std::array<contact::Q4CertifiedIntegral,recurrence::MaxNodes> mass_rates_{};
  double rate_spread_upper_=0,maximum_frequency_=0;
  bool prepared_=false;
  friend bool BuildWallRecurrenceModel(unsigned,WallRecurrenceModel&,std::string&);
};
bool BuildWallRecurrenceModel(unsigned cells,WallRecurrenceModel&,std::string&);
bool FrozenStep(double h);
bool FrozenVelocity(double normal_velocity);
} // namespace tl::qualification::qeph::wall_recurrence
