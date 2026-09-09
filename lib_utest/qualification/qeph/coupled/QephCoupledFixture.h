#pragma once
// Qualification-only composition of the existing BQ fixture and native Q2.
// No production owner, time integrator, material operation or CUDA kernel here.
#include "../batch/QephBatchFixture.h"
#include "lib_src/math/Quaternion.h"

namespace qeph_coupled_test {
using namespace qeph_batch_test;
constexpr double Side=.02, Young=200e9, Density=7890, Thickness=.001648, Poisson=.3;
constexpr double Theta=1e-4, Delta=Side*Theta, H0=0x1p-24;
constexpr std::uint64_t CoupledQualification=0x4251335052454631ULL; // BQ3 short prefix only.
inline double BendingScale() { return Young*Thickness*Thickness*Thickness/(12*(1-Poisson*Poisson)); }
inline double VelocityBudget() { return 2e-12*std::sqrt(Young/Density)*Theta; }
inline double SpinBudget() { return VelocityBudget()/Side; }
using PortResults=std::array<q::ForceTrial,4>;
using NativeResults=std::array<native::ForceTrial,4>;

struct Prepared {
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  Snapshot state;
  Loads assembled;
};
bool InitializeCoupled(Rig&,double h);
Loads Amplitude(const Rig&);
Loads Applied(const Rig&,double base_time);
bool PrepareCoupled(Rig&,const Loads&,Prepared&);
q::PrescribedInterval Interval(const Rig&,unsigned element,const Snapshot&,double base_time,std::uint64_t epoch);
void OwnerAgreement(const Rig&,const Snapshot&,const Snapshot&);

// Accepted native expected values are copied only after the CUDA joint commit.
// Fixed arrays compose independently rounded scalar kicks/drifts with the real
// native element operation; no force operation is evaluated on read/capture.
struct NativeProposal { Snapshot state; NativeResults cache; Loads rhs; };
class NativeSequence {
 public:
  bool Initialize(const Rig&);
  bool Propose(const Rig&,const Loads&,NativeProposal&) const;
  void Accept(const Rig& r,const NativeProposal& p) { state=p.state; cache=p.cache; time+=r.h; ++epoch; }
  Snapshot state;
  NativeResults cache;
  std::array<double,N> mass{},inertia{};
  double time=0;
  std::uint64_t epoch=0;
 private:
  std::array<native::Reference,4> reference_;
};
} // namespace qeph_coupled_test
