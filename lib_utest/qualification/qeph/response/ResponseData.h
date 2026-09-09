#pragma once
// Qualification observations only. FENodalState remains the physical owner.
#include "lib_src/elements/qeph/QephForceData.h"
#include <array>
#include <string>
#include <vector>

namespace tl::qualification::qeph::response {
namespace port=tl::fea::qeph;
constexpr double H0=0x1p-24, Pulse=1024*H0, Horizon=4096*H0;
constexpr double Side=.02, Young=200e9, Density=7890, Thickness=.001648, Poisson=.3;
constexpr double Theta=1e-4, Delta=Side*Theta;
constexpr std::uint64_t Qualification=0x4251345245535031ULL, Configuration=0x4251344d4f444531ULL;
constexpr unsigned SampleCount=257, MaxFields=512, MaxNodes=6, MaxElements=2;
constexpr std::size_t FileCap=32*1024*1024;
struct Config { unsigned cells=0, refinement=0; };
bool ValidConfig(Config) noexcept;
double BendingScale() noexcept;
double PulseFactor(double time) noexcept;

struct Model {
  unsigned cells=0,nodes=0;
  std::array<port::ReferenceData,MaxElements> reference{};
  std::array<std::array<unsigned,4>,MaxElements> connectivity{};
  std::array<double,3*MaxNodes> initial_position{};
  std::array<double,MaxNodes> mass{},inertia{},physical{},added{};
};
// Same frozen model as BQ3; this function builds only immutable reference data.
bool BuildModel(unsigned cells,Model&,std::string& error);
struct State {
  std::array<double,3*MaxNodes> x{},v{},omega{};
  std::array<double,4*MaxNodes> q{};
};
using Results=std::array<port::ForceTrial,MaxElements>;
struct Field { std::string name,unit; double scale=1; bool compare=false; };
struct Extremum { double value=0,time=0; unsigned field=0; double lower=0; };
struct Sample {
  std::uint64_t epoch=0;
  double time=0,carried_velocity_time=0,kick_dt=0;
  bool interval_available=false;
  std::array<double,MaxFields> values{};
  // Translation, total rotation, physical isotropic, area-added isotropic.
  std::array<double,4> carried_kinetic{},synchronous_kinetic{};
  std::array<double,3> source_work{}; // EINT0, EINT1, EVIS; not a potential.
  double external_work=0,residual=0;
};
struct Limits {
  double displacement_over_side=0,rotation_angle=0,strain=0,thickness_curvature=0;
  double minimum_area_ratio=1,maximum_area_ratio=1,minimum_thickness_ratio=1,maximum_thickness_ratio=1;
};
struct Run {
  Config config;
  Model model;
  std::vector<Field> fields;
  std::vector<Sample> samples; // Reserved to 257 before the first interval.
  Sample last_accepted;
  std::uint64_t owner_id=0,accepted_steps=0,attempted_steps=0;
  bool completed=false; // A completed single run does not admit refinement.
  std::string failure;
  double elapsed_seconds=0,external_work_at_pulse=0,maximum_abs_residual=0,residual_time=0;
  std::array<double,3> external_linear_impulse{},external_angular_impulse{};
  std::array<double,7> ledger_maxima{};
  Limits observed;
  Extremum maximum_response;
  std::size_t owner_device_bytes=0,batch_device_bytes=0,owner_allocations=0,batch_allocations=0;
  std::size_t cuda_free_before=0,cuda_free_after_initialize=0,cuda_free_after_run=0;
};
struct Difference { double maximum=0,time=0; unsigned field=0; double lower=0; };
struct Comparison {
  bool passed=false;
  Difference coarse_medium,medium_fine;
  double energy_normalization=0;
  std::array<double,3> residual_ratios{};
  std::string diagnostic;
};
std::vector<Field> Dictionary(const Model&);
double ExperimentEnergy(const Model&) noexcept;
bool ValidateRun(const Run&,bool require_complete,std::string& error);
Comparison Compare(const std::array<Run,3>&);
} // namespace tl::qualification::qeph::response
