#include "OwnerFixture.h"
#include "NativeFixture.h"

namespace cin_runtime_test {
TEST(CinRuntimeCuda, ThreeAcceptedDeviceStagesMatchCompleteNativeForceMotionAndCurrentCoefficients) {
  Fixture f;
  f.inertia[f.rows.back().secondary] = 0;
  NativeState native(f);
  fea::FENodalState owner;
  ASSERT_EQ(Initialize(owner, f).status, fea::NodalStatus::Ok);
  const auto initial_load = f.load;
  Snapshot current(native.n, native.r);
  fea::NodalStamp stamp;
  for (unsigned step = 0; step < 3; ++step) {
    SCOPED_TRACE(step);
    for (std::size_t i = 0; i < f.load.size(); ++i) f.load[i] = initial_load[i]*(step+1);
    const auto dt = f.Config().fixed_dt;
    ASSERT_EQ(native.Step(f, step*dt, dt, step ? dt : dt/2), 0);
    native.Drift(dt);
    fea::NodalTrialToken token;
    fea::NodalAssemblyView view;
    fea::NodalCinAssemblyView cin_view;
    Fill(owner, f, token, view, cin_view);
    ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceStaggeredCin(owner, token, Admission(view)).status, fea::NodalStatus::Ok);
    std::vector<double> forces(6*native.n);
    double* component[] = {view.forces.force_x, view.forces.force_y, view.forces.force_z,
      view.forces.couple_x, view.forces.couple_y, view.forces.couple_z};
    for (unsigned axis = 0; axis < 6; ++axis) {
      ASSERT_EQ(cudaMemcpyAsync(forces.data()+axis*native.n, component[axis], native.n*sizeof(double),
          cudaMemcpyDeviceToHost, view.stream), cudaSuccess);
    }
    ASSERT_EQ(cudaStreamSynchronize(view.stream), cudaSuccess);
    Complete(owner, token, view);
    ASSERT_EQ(owner.Commit(token).status, fea::NodalStatus::Ok);
    Accepted(owner, current, stamp);
    EXPECT_EQ(stamp.epoch, step+1);
    EXPECT_EQ(stamp.reaction_kick_dt, step ? dt : dt/2);
    for (std::size_t i = 0; i < native.n; ++i) {
      SCOPED_TRACE(i);
      EXPECT_DOUBLE_EQ(current.coefficients[i], native.mass[i]);
      EXPECT_DOUBLE_EQ(current.coefficients[native.n+i], native.inertia[i]);
      for (unsigned axis = 0; axis < 3; ++axis) {
        const auto j = 3*i+axis;
        tied_patch_test::Near(current.nodes[j], native.x[j]);
        tied_patch_test::Near(current.nodes[3*native.n+j], native.velocity[j]);
        tied_patch_test::Near(current.nodes[6*native.n+j], native.omega[j]);
        tied_patch_test::Near(forces[axis*native.n+i], native.force[j]);
        EXPECT_EQ(forces[(axis+3)*native.n+i], native.couple[j]);
        EXPECT_EQ(current.nodes[13*native.n+j], 0);
        EXPECT_EQ(current.nodes[16*native.n+j], 0);
      }
    }
    for (std::size_t row = 0; row < native.r; ++row) {
      EXPECT_EQ(current.coefficients[2*native.n+row], native.saved_mass[row]);
      EXPECT_EQ(current.coefficients[2*native.n+native.r+row], native.saved_inertia[row]);
    }
    EXPECT_DOUBLE_EQ(current.coefficients.back(), native.numerical_mass);
  }
}
} // namespace cin_runtime_test
