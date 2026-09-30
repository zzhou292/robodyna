#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace shell_hourglass {
// Fixture-local value type; hourglass mechanics does not depend on contact.
// A production common-math view can be introduced separately when needed.
struct Vec3 { double x=0,y=0,z=0; };
static_assert(sizeof(Vec3)==3*sizeof(double),"Contiguous xyz fixture values required");

// S2b qualification only: active planar Q4, NPT3, IHBE1, FOR=MOM=0.
enum class Mode { UniformRectangle=1, CorrectedPlanar=2 };
enum class Status { Ok, InvalidInput, UnsupportedGeometry, ResourceLimit,
  CudaError, InvalidOutput, TrialRejected, NoTrial, StaleTrial, HistoryLimit };
struct Report { Status status=Status::InvalidInput; std::string message; std::size_t device_bytes=0; };

struct Parameters {
  double H1=0,H2=0,H3=0;
  double SRH1=0,SRH2=0,SRH3=0;
  double HVISC=0,HELAS=0,HVLIN=0;
};
struct Element {
  std::array<int,4> nodes{};  // Zero-based, distinct, perimeter order.
  double thickness=.1,young=1000,nu=.25,density=1,sound_speed=32;
  double shear_factor=5.0/6.0;
};
struct Configuration {
  Mode mode=Mode::UniformRectangle;
  Parameters parameters{};
  std::vector<Element> elements;
  std::size_t node_count=0;
};
// Caller owns all input arrays and keeps them alive until EvaluateTrial returns.
// Vec3 is contiguous xyz; these views are host memory, copied to private device
// scratch. Positions/V/VR are world coordinates, metres, m/s and rad/s.
struct BorrowedKinematics {
  const Vec3* positions=nullptr;
  const Vec3* velocity=nullptr;
  const Vec3* angular_velocity=nullptr;
  std::size_t node_count=0;
};
struct ElementState {
  // HOUR1-3 are accumulated local-frame force histories; HOUR4-5 are
  // instantaneous rotational moments. They are not five energy components.
  std::array<double,5> hour{};
  std::array<double,2> energy{};  // Donor accumulated work; may decrease on unload.
  double off=1;
  std::array<double,6> reference_coordinates{};
  std::array<double,6> edge_diagonal_lengths{};
  std::array<Vec3,3> frame{};
  double area=0,px1=0,px2=0,py1=0,py2=0,vhx=0,vhy=0;
};
struct Snapshot {
  std::vector<ElementState> elements;
  std::vector<Vec3> forces,moments;
  std::uint64_t revision=0;
  double accepted_time=0;
  std::size_t device_bytes=0;
};
struct Options {
  std::size_t max_device_bytes=1024*1024;
  bool reject_after_assembly=false;  // Explicit fixture-only fault injection.
};

class State;
class Trial;
Report Initialize(const Configuration&,State*);
Report EvaluateTrial(const State&,const BorrowedKinematics&,double,Trial*,const Options& = {});
Status Commit(State*,Trial*) noexcept;
void Discard(Trial*) noexcept;

class State {
 public:
  State()=default;
  State(const State&)=delete;State& operator=(const State&)=delete;
  State(State&&)=delete;State& operator=(State&&)=delete;
  const Snapshot& accepted()const{return accepted_;}
  const Configuration& configuration()const{return configuration_;}
 private:
  friend Report Initialize(const Configuration&,State*);
  friend Report EvaluateTrial(const State&,const BorrowedKinematics&,double,Trial*,const Options&);
  friend Status Commit(State*,Trial*) noexcept;
  Configuration configuration_;
  Snapshot accepted_;
  bool initialized_=false;
};
class Trial {
 public:
  Trial()=default;
  Trial(const Trial&)=delete;Trial& operator=(const Trial&)=delete;
  const Snapshot& candidate()const{return candidate_;}
  bool valid()const{return valid_;}
 private:
  friend Report EvaluateTrial(const State&,const BorrowedKinematics&,double,Trial*,const Options&);
  friend Status Commit(State*,Trial*) noexcept;
  friend void Discard(Trial*) noexcept;
  Snapshot candidate_;
  const State* owner_=nullptr;
  std::uint64_t base_revision_=0;
  bool valid_=false;
};
// Commit swaps already prepared storage and cannot allocate. It validates both
// owner and epoch. Initializing a State twice is rejected, so stale trials cannot
// become valid after a reset. State must outlive every borrowed trial reference.
// Fresh trial evaluation is limited to 128 accepted increments, 16 elements,
// 64 nodes and <=1MiB owned device allocations. This is no time integrator.

}  // namespace shell_hourglass
