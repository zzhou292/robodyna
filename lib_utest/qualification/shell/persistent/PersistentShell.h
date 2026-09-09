#pragma once

#include "lib_src/solvers/FENodalStateView.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace tl::qualification::shell {
constexpr std::size_t MaxElements = 2;
constexpr std::size_t MaxNodes = 8;
constexpr double Young = 210e9, Poisson = .3, Yield = 1e12;
constexpr double Temperature = 293.15, ShearFactor = 5.0/6.0;

enum class GeometryMode { FrozenReference = 1, Current = 2 };
enum class Status { Ok, InvalidInput, UnsupportedGeometry, ResourceLimit,
  InvalidOutput, Rejected, StaleTrial, NoTrial, HistoryLimit, DeviceFailure };
enum class RejectAfter { None, Geometry, Material, Assembly };
struct Report { Status status = Status::InvalidInput; std::string message; };
struct Stabilization {
  double H1=0,H2=0,H3=0,SRH1=0,SRH2=0,SRH3=0,HVISC=0,HELAS=0,HVLIN=0;
};
struct Element {
  std::array<int,4> nodes{};
  double thickness = .01;
};
struct Configuration {
  std::array<Element,MaxElements> elements{};
  std::size_t element_count = 1, node_count = 4;
  GeometryMode geometry = GeometryMode::FrozenReference;
  bool update_thickness = false;
  Stabilization stabilization{};
  std::size_t max_device_bytes = 1024*1024;
};

struct PointState {
  std::array<double,5> stress{}; // local xx,yy,xy,yz,xz
  double plastic_strain=0,plastic_rate=0,plastic_increment=0,temperature=0;
  std::array<double,3> backstress{};
};
struct ElementState {
  double off=1, area=0, thickness=0, step_thickness=0;
  double px1=0,px2=0,py1=0,py2=0,vhx=0,vhy=0;
  std::array<double,9> frame{}; // world basis columns, xyz for each column
  std::array<double,6> reference_coordinates{};
  std::array<double,8> generalized_strain{}; // accumulated donor GSTR
  std::array<double,5> normalized_force{},hour{};
  std::array<double,3> normalized_moment{};
  std::array<double,2> work{}; // membrane+shear; bending+hourglass, native semantics
  std::array<PointState,3> points{};
  double material_work_increment=0,hourglass_work_increment=0;
};
struct Snapshot {
  std::array<ElementState,MaxElements> elements{};
  std::array<double,3*MaxNodes> position{},velocity{},angular_velocity{};
  std::array<double,3*MaxNodes> force{},couple{};
  std::size_t element_count=0,node_count=0,device_bytes=0;
  std::uint64_t epoch=0;
  double time=0,completed_dt=0;
};

// Prescribed completed interval: positions at time_end, carried interval V/VR
// and dt=time_end-time_begin. For rigid-path tests use exact endpoint positions
// and secant translation velocity. This request does not integrate acceleration.
struct Request {
  fea::HostNodalKinematicsView kinematics{};
  double time_begin=0,time_end=0;
  RejectAfter reject_after=RejectAfter::None;
};

class PersistentShell;
struct TrialToken {
  // Borrowed owner identity. Tokens must not outlive their PersistentShell.
 private:
  friend class PersistentShell;
  const PersistentShell* owner=nullptr;
  std::uint64_t epoch=0,attempt=0;
};

// Qualification owner with two preallocated device slabs and one node space.
// Element histories are private; the real TL nodal views are consumed directly
// by K1/K3. Readbacks support numerical tests; this is not a production solver.
class PersistentShell {
 public:
  PersistentShell();
  ~PersistentShell();
  PersistentShell(const PersistentShell&)=delete;
  PersistentShell& operator=(const PersistentShell&)=delete;
  Report Initialize(const Configuration&,fea::HostNodalKinematicsView initial);
  Report Evaluate(const Request&,TrialToken*);
  Status Commit(const TrialToken&) noexcept;
  void Discard() noexcept;
  const Snapshot& accepted() const noexcept;
  const Snapshot* trial() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Snapshot empty_{};
};
}  // namespace tl::qualification::shell
