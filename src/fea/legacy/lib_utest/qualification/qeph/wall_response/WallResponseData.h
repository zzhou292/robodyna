#pragma once
// Qualification observations only; no owner, force evaluator or commit hook.
#include "../response/ResponseData.h"
#include "../wall_recurrence/WallRecurrenceModel.h"
#include "../wall_recurrence/WallSwitchingSchedule.h"
#include <type_traits>

namespace tl::qualification::qeph::wall_response {
namespace port=tl::fea::qeph;
namespace contact=tlfea::contact;
namespace wr=tl::qualification::qeph::wall_recurrence;
namespace free_response=tl::qualification::qeph::response;
using Interval=contact::Q4IntegralInterval;
using State=free_response::State;
using Results=free_response::Results;
using Field=free_response::Field;
constexpr double H0=0x1p-24,Horizon=4096*H0;
constexpr unsigned SampleCount=257,MaxSteps=32768,MaxNodes=6,MaxElements=2,MaxFields=640;
constexpr std::uint64_t Qualification=0x4357325245535031ULL,ShellConfiguration=0x4357325253484531ULL;
constexpr std::uint64_t WallConfiguration=0x4357325257414c31ULL,WallBinding=0x4357325246494e31ULL;
constexpr double MaximumAnalyticError=.02,MaximumEnergyRatio=.02;
struct Config {
  unsigned cells=0,refinement=0;
  double selected_h=0;
  std::string screen_index_sha;
};
bool ValidConfig(const Config&) noexcept;
double Step(const Config&) noexcept;
unsigned Steps(const Config&) noexcept;
unsigned SampleStride(const Config&) noexcept;
struct NodeFields { unsigned x=0,q=0,v=0,w=0,vs=0,ws=0,angle=0; };
struct ElementFields {
  unsigned stress=0,material=0,bending=0,hourglass=0,strain=0,thickness=0;
  unsigned work=0,viscous=0,active=0,force=0,couple=0,geometry=0;
};

class Model {
 public:
  bool prepared() const noexcept { return prepared_; }
  const wr::WallRecurrenceModel& screened() const noexcept { return screened_; }
  const free_response::Model& fields() const noexcept { return fields_; }
  double energy() const noexcept { return energy_; }
  double momentum() const noexcept { return momentum_; }
  double force_scale() const noexcept { return force_scale_; }
  double time_scale() const noexcept { return wr::TargetDepth/wr::ImpactSpeed; }
  const std::vector<Field>& dictionary() const noexcept { return dictionary_; }
  const NodeFields& node_fields(unsigned n) const noexcept { return node_fields_[n]; }
  const ElementFields& element_fields(unsigned e) const noexcept { return element_fields_[e]; }
  unsigned native_field_count() const noexcept { return native_field_count_; }
 private:
  wr::WallRecurrenceModel screened_;
  free_response::Model fields_;
  std::vector<Field> dictionary_;
  std::array<NodeFields,MaxNodes> node_fields_{};
  std::array<ElementFields,MaxElements> element_fields_{};
  unsigned native_field_count_=0;
  double energy_=0,momentum_=0,force_scale_=0;
  bool prepared_=false;
  friend bool BuildModel(unsigned,Model&,std::string&);
};
bool BuildModel(unsigned cells,Model&,std::string& error);

// Copied from the SAME already checked candidate/contact readback. At epoch0
// use accepted-base results. The runtime authenticates owner/participant IDs;
// these numerical records do not authenticate arbitrary device writes.
struct ContactState {
  std::uint64_t base_epoch=0,attempt=0;
  unsigned node_count=0;
  bool candidate=false;
  std::array<contact::NodalWallPointResult,MaxNodes> nodes{};
  contact::Q4CertifiedIntegral resultant,potential;
};
struct EndpointInput {
  std::uint64_t epoch=0;
  double time=0,carried_velocity_time=0,kick_dt=0;
  State state;
  Results elements;
  ContactState contact;
  // Positive opposite wall impulse accumulated with actual s_n, not h drift.
  double wall_impulse=0,wall_impulse_error=0;
};
struct Sample {
  std::uint64_t epoch=0;
  double time=0,carried_velocity_time=0,kick_dt=0;
  State state;
  ContactState contact;
  double wall_impulse=0,wall_impulse_error=0;
  double synchronous_wall_impulse=0,synchronous_wall_impulse_error=0;
  // Native history/cache/kinematics are flattened once through the owning
  // ResponseFieldVisitor below, never deserialized into a fabricated History.
  std::array<double,3*MaxNodes> synchronous_velocity{},synchronous_omega{},rotation_vector{};
  std::array<double,3*MaxNodes> velocity_error{},omega_error{},endpoint_rhs{},endpoint_couple{};
  std::array<double,3*MaxNodes> endpoint_native_force_error{},endpoint_native_couple_error{},endpoint_contact_force_error{};
  std::array<double,4> carried_kinetic{},synchronous_kinetic{},kinetic_error{};
  std::array<double,3> source_work{};
  // Each value's absolute uncertainty is explicit; raw recorded x has zero.
  std::array<double,MaxFields> values{},errors{};
  Interval residual{},absolute_residual{},analytic_difference{};
  double minimum_gap=0,maximum_gap=0,minimum_velocity=0,maximum_velocity=0;
  double relative_displacement=0,rotation_angle=0,strain=0,thickness_curvature=0;
  double minimum_area_ratio=1,maximum_area_ratio=1,minimum_thickness_ratio=1,maximum_thickness_ratio=1;
  unsigned mask=0;
};
struct EventBracket {
  bool observed=false;
  std::uint64_t before_epoch=0,after_epoch=0;
  double before_time=0,after_time=0;
};
enum class Phase { BeforeEntry,Compression,Unloading,Separated };
struct Summary {
  bool initialized=false;
  std::uint64_t last_epoch=0;
  double last_time=0;
  unsigned last_mask=0;
  double last_minimum_velocity=0,last_maximum_velocity=0;
  Phase phase=Phase::BeforeEntry;
  EventBracket entry,peak,exit;
  Interval maximum_absolute_residual{},maximum_analytic_difference{};
  double residual_time=0,analytic_time=0;
  double maximum_depth=0,maximum_resultant=0,maximum_resultant_error=0;
  double depth_time=0,resultant_time=0;
  double maximum_relative_displacement=0,maximum_rotation=0,maximum_strain=0,maximum_thickness_curvature=0;
  double minimum_area_ratio=1,maximum_area_ratio=1,minimum_thickness_ratio=1,maximum_thickness_ratio=1;
};
struct Run {
  Config config;
  Model model;
  std::vector<Sample> samples; // Reserve exactly257 before any owner step.
  Sample last_accepted;
  Summary summary;
  std::uint64_t owner_id=0,accepted_steps=0,attempted_steps=0,native_cell_intervals=0;
  bool completed=false;
  std::string failure;
  double elapsed_seconds=0;
  std::array<double,10> ledger_maxima{};
  std::size_t owner_device_bytes=0,batch_device_bytes=0,wall_device_bytes=0;
  std::size_t owner_allocations=0,batch_allocations=0,wall_allocations=0;
};
static_assert(std::is_trivially_copyable<Sample>::value,"Postcommit sample publication must only copy fixed data");
static_assert(std::is_trivially_copyable<Summary>::value,"Postcommit summary publication must only copy fixed data");

// Neither function changes output on failure. Runtime stages BOTH before its
// unchanged joint commit and checks reserved sample capacity prospectively.
bool ObserveEndpoint(const Model&,const Config&,const EndpointInput&,Sample&,std::string& error);
bool ValidateSample(const Model&,const Config&,const Sample&,std::string& error);
bool StageSummary(const Model&,const Config&,const Summary&,const Sample&,Summary&,std::string& error);
bool CompleteSummary(const Model&,const Config&,const Summary&,std::string& error);
// Independent continuous extrema, normalized by d and F*, rederived from the
// immutable rate certificates and retained measured extrema, without a run.
bool PeakTruth(const Model&,const Summary&,Interval& depth_difference,Interval& force_difference,std::string& error);
} // namespace tl::qualification::qeph::wall_response
