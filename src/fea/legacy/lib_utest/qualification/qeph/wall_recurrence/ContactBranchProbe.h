#pragma once
#include "WallRecurrenceModel.h"

namespace tl::qualification::qeph::wall_recurrence {
enum class ContactBranch { Inactive,Active };
bool BuildContactKick(const WallRecurrenceModel&,double h,ContactBranch,
                      Eigen::MatrixXd& output,std::string&);
bool BuildContactBranch(const WallRecurrenceModel&,double h,ContactBranch,
                        const Eigen::MatrixXd& shell,Eigen::MatrixXd& output,std::string&);
struct ContactDirection { std::string name; Eigen::VectorXd value; };
bool SignConeDirections(const WallRecurrenceModel&,ContactBranch,
                        std::vector<ContactDirection>& output,std::string&);

// Compact recording of the owning result: retain every used point/parent
// record without copying the unused 128-parent/node capacity into each probe.
struct ContactMapSample {
  Eigen::VectorXd state;
  std::vector<contact::NodalWallPointResult> nodes;
  std::vector<contact::NodalWallParentResult> parents;
  contact::Q4CertifiedIntegral resultant,potential;
  contact::Vec3 wall_reaction,wall_moment;
  double surface_power=0;
  std::uint64_t base_epoch=0,attempt=0;
};
// Actual host law on represented x/v, THEN its kick on copied normalized v,
// THEN the retained native full map's internal-cache kick, drift and Q2.
// No produced cache enters this same kick. Every failure preserves output.
bool EvaluateContactNativeMap(const WallRecurrenceModel&,double h,double normal_velocity,
                              const Eigen::VectorXd& input,ContactMapSample&,std::string&);
struct ContactDirectionProbe {
  ContactDirection direction;
  std::array<ContactMapSample,3> samples;
  std::array<Eigen::VectorXd,2> quotients;
  std::array<double,2> residual_upper{},budget{};
  unsigned completed_samples=0;
  bool complete=false,passed=false;
  std::string diagnostic;
};
struct ContactBranchProbe {
  double h=0,normal_velocity=0;
  ContactBranch branch=ContactBranch::Inactive;
  Eigen::MatrixXd full;
  ContactMapSample baseline;
  std::vector<ContactDirectionProbe> directions;
  bool baseline_complete=false,complete=false,passed=false;
  std::string diagnostic;
};
// All n+7 physical sign-cone directions at the three frozen amplitudes.
// Retain actual sample data before decisions; no spectral/gain admission.
// Caller binds native_shell to this model/h/boost. A plain matrix operand has
// no intrinsic provenance and these directions cannot authenticate it.
ContactBranchProbe ProbeContactBranch(const WallRecurrenceModel&,double h,double normal_velocity,
                                      ContactBranch,const Eigen::MatrixXd& native_shell);
} // namespace tl::qualification::qeph::wall_recurrence
