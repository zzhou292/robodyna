#pragma once
#include "Fixture.h"
#include "native/Packet.h"

namespace cin_runtime_test {
// Test packet state, independent of all production force/coefficient helpers.
// Only connectivity and literal source values are shared with the subject.
struct NativeState {
  std::size_t n, r;
  std::vector<int> masters, secondary;
  std::vector<double> x, force, couple, mass, inertia, stif, stifr, saved_mass, saved_inertia;
  std::vector<double> st, dpara, velocity, omega, acceleration, angular_acceleration;
  std::array<double,6> force_integral{};
  double numerical_mass = 0;
  explicit NativeState(const Fixture& f)
      : n(f.mass.size()), r(f.rows.size()), masters(4*r), secondary(r), x(f.x), force(3*n),
        couple(3*n), mass(f.mass), inertia(f.inertia), stif(f.stif), stifr(f.stifr),
        saved_mass(r), saved_inertia(r), st(2*r), dpara(7*r), velocity(f.velocity),
        omega(f.omega), acceleration(3*n), angular_acceleration(3*n) {
    for (std::size_t row = 0; row < r; ++row) {
      secondary[row] = int(f.rows[row].secondary)+1;
      for (unsigned slot = 0; slot < 4; ++slot) masters[4*row+slot] = int(f.rows[row].masters[slot])+1;
    }
    int status = -1;
    tl_cin_native_seed(n, r, secondary.data(), mass.data(), inertia.data(),
        saved_mass.data(), saved_inertia.data(), &status);
    if (status) throw std::runtime_error("Native INIEND seed rejected");
  }
  int Step(const Fixture& f, double time, double dt, double kick) {
    for (std::size_t i = 0; i < n; ++i) {
      for (unsigned axis = 0; axis < 3; ++axis) {
        force[3*i+axis] = f.load[axis*n+i];
        couple[3*i+axis] = f.load[(axis+3)*n+i];
      }
    }
    stif = f.stif;
    stifr = f.stifr;
    return Call(time, dt, kick);
  }
  int Call(double time, double dt, double kick) {
    int status = -1;
    tl_cin_native_stage(n, r, masters.data(), secondary.data(), x.data(), st.data(),
        force.data(), couple.data(), mass.data(), inertia.data(), stif.data(), stifr.data(),
        saved_mass.data(), saved_inertia.data(), &numerical_mass, force_integral.data(),
        dpara.data(), velocity.data(), omega.data(), acceleration.data(), angular_acceleration.data(),
        time, dt, kick, &status);
    return status;
  }
  void Drift(double dt) {
    for (std::size_t i = 0; i < x.size(); ++i) x[i] = x[i]+dt*velocity[i];
  }
};
inline void CompareValues(const Fixture& f, const NativeState& native) {
  using tied_patch_test::Near; // Existing qualified patch comparison budget.
  for (std::size_t i = 0; i < native.n; ++i) {
    SCOPED_TRACE(i);
    EXPECT_DOUBLE_EQ(f.mass[i], native.mass[i]);
    EXPECT_DOUBLE_EQ(f.inertia[i], native.inertia[i]);
    EXPECT_DOUBLE_EQ(f.stif[i], native.stif[i]);
    EXPECT_DOUBLE_EQ(f.stifr[i], native.stifr[i]);
    for (unsigned axis = 0; axis < 3; ++axis) {
      Near(f.load[axis*native.n+i], native.force[3*i+axis]);
      EXPECT_EQ(f.load[(axis+3)*native.n+i], native.couple[3*i+axis]);
      Near(f.velocity[3*i+axis], native.velocity[3*i+axis]);
      Near(f.omega[3*i+axis], native.omega[3*i+axis]);
      Near(f.a[3*i+axis], native.acceleration[3*i+axis]);
      Near(f.ar[3*i+axis], native.angular_acceleration[3*i+axis]);
    }
  }
  EXPECT_DOUBLE_EQ(f.dmas, native.numerical_mass);
  EXPECT_EQ(f.saved_mass, native.saved_mass);
  EXPECT_EQ(f.saved_inertia, native.saved_inertia);
  for (std::size_t row = 0; row < native.r; ++row) {
    for (unsigned k = 0; k < 7; ++k) {
      EXPECT_DOUBLE_EQ(f.patches[row].values().cofactor[k], native.dpara[7*row+k]);
    }
  }
}
} // namespace cin_runtime_test
