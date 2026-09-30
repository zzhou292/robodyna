#pragma once
#include "lib_utest/qualification/native/qeph/QephForceReference.h"
#include <Eigen/Core>
#include <array>
#include <complex>
#include <string>
#include <vector>

namespace tl::qualification::qeph::recurrence {
constexpr double Length=.02,Young=200e9,Density=7890,Thickness=.001648,Poisson=.3,H0=0x1p-24;
constexpr std::array<double,6> Steps{{H0/8,H0/4,H0/2,H0,2*H0,4*H0}};
constexpr std::array<double,3> Amplitudes{{0x1p-18,0x1p-19,0x1p-20}};
constexpr double MatrixTolerance=5e-8,DecompositionTolerance=1e-10,MaximumGramGain=64;
constexpr unsigned MaxNodes=6,MaxElements=2;
enum class Group {Position,OrientationTangent,Velocity,Spin,History,ForceCache};
struct Coordinate {
  Group group; unsigned entity=0,component=0; double scale=1;
  bool feedback=false; std::string name,unit;
};
struct Model {
  unsigned elements=0,nodes=0;
  std::array<Reference,MaxElements> reference;
  std::array<std::array<unsigned,4>,MaxElements> connectivity{};
  std::array<Vec3,MaxNodes> position{};
  std::array<double,MaxNodes> mass{},inertia{};
  std::vector<Coordinate> dictionary;
};
bool BuildModel(unsigned cells,Model&,std::string&);
std::vector<unsigned> FeedbackIndices(const Model&);
// One native full-kick map of nondimensional perturbations around reference/rest.
// Epoch1/timeh -> epoch2/time2h are fixed probe labels, never a runtime clock.
// Input/output use the complete dictionary; failure preserves caller output.
bool NativeMap(const Model&,double h,const Eigen::VectorXd&,Eigen::VectorXd&,std::string&);
struct MatrixProbe {
  double amplitude=0; Eigen::MatrixXd full;
  unsigned completed_columns=0; bool complete=false; std::string diagnostic;
};
MatrixProbe Differentiate(const Model&,double h,double amplitude);
struct MapAnalysis {
  bool complete=false,passed=false;
  double schur_residual=0,orthogonality_error=0,spectral_radius=0;
  double gram_gain=0,gram_antisymmetry=0,zero_feedback_error=0;
  double observer_error=0,rigid_error=0,rigid_basis_condition=0,rigid_projection_residual=0;
  unsigned near_one=0,near_zero=0;
  std::vector<std::complex<double>> eigenvalues;
  std::string diagnostic;
};
// Mathematical finite-horizon Gram, sum k=0..count-1 (A^k)^T A^k.
// Binary composition handles non-powers of two; no symmetry assumption on A.
bool PowerGram(const Eigen::MatrixXd&,unsigned count,Eigen::MatrixXd&,std::string&);
MapAnalysis Analyze(const Model&,double h,const MatrixProbe&);
struct StepAudit {
  double h=0; std::array<MatrixProbe,3> probes; std::array<MapAnalysis,3> analysis;
  std::array<double,2> matrix_differences{},gram_relative_differences{};
  bool passed=false; std::string diagnostic;
};
struct CaseAudit {
  unsigned elements=0,nodes=0;
  std::vector<Coordinate> dictionary; std::array<StepAudit,6> steps;
  bool complete=false; std::string diagnostic;
};
struct Audit {
  std::array<CaseAudit,2> cases;
  double selected_h=0; bool audit_passed=false;
};
// All raw matrices for both fixtures are retained before any numerical verdict.
Audit CollectProbes();
void Decide(Audit&);
Audit RunAudit();
} // namespace tl::qualification::qeph::recurrence
