#pragma once
#include "Fixture.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace cin_runtime_test {
struct Snapshot {
  std::size_t n, r;
  std::vector<double> nodes, coefficients;
  Snapshot(std::size_t nodes_count, std::size_t rows_count)
      : n(nodes_count), r(rows_count), nodes(19*n, -71.), coefficients(2*n+2*r+1, -72.) {}
  fea::NodalSnapshotBuffer Nodes() {
    return {nodes.data(), nodes.data()+3*n, n, nodes.data()+9*n,
      nodes.data()+6*n, nodes.data()+13*n, nodes.data()+16*n};
  }
  fea::NodalCinSnapshotBuffer Cin() {
    return {coefficients.data(), coefficients.data()+n, coefficients.data()+2*n,
      coefficients.data()+2*n+r, coefficients.data()+2*n+2*r, n, r};
  }
};
inline fea::NodalReport Initialize(fea::FENodalState& owner, Fixture& f) {
  return owner.Initialize(f.Config(), f.Kinematics(), f.inverse.data(), f.Dofs(), f.Startup());
}
inline void Fill(fea::FENodalState& owner, const Fixture& f, fea::NodalTrialToken& token,
    fea::NodalAssemblyView& assembly, fea::NodalCinAssemblyView& cin) {
  ASSERT_EQ(owner.BeginTrial(&token, &assembly).status, fea::NodalStatus::Ok);
  ASSERT_EQ(owner.BorrowCinAssembly(token, &cin).status, fea::NodalStatus::Ok);
  const auto n = f.mass.size();
  double* component[] = {assembly.forces.force_x, assembly.forces.force_y, assembly.forces.force_z,
    assembly.forces.couple_x, assembly.forces.couple_y, assembly.forces.couple_z};
  for (unsigned a = 0; a < 6; ++a) {
    ASSERT_EQ(cudaMemcpyAsync(component[a], f.load.data()+a*n, n*sizeof(double),
      cudaMemcpyHostToDevice, assembly.stream), cudaSuccess);
  }
  ASSERT_EQ(cudaMemcpyAsync(cin.translational_stiffness, f.stif.data(), n*sizeof(double),
      cudaMemcpyHostToDevice, assembly.stream), cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(cin.rotational_stiffness, f.stifr.data(), n*sizeof(double),
      cudaMemcpyHostToDevice, assembly.stream), cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(cin.witness_activity, f.flags.data(), f.flags.size(),
      cudaMemcpyHostToDevice, assembly.stream), cudaSuccess);
}
inline fea::NodalCinAdmission Admission(const fea::NodalAssemblyView& view, double angle = .2) {
  return {view.owner_id, view.accepted.base_epoch, view.attempt, 871, 1e-6, angle, true};
}
inline void Complete(fea::FENodalState& owner, const fea::NodalTrialToken& token,
    const fea::NodalAssemblyView& view) {
  ASSERT_EQ(fea::CompleteNodalValidation(owner, token,
    {view.owner_id, view.accepted.base_epoch, view.attempt, 871, true}).status, fea::NodalStatus::Ok);
}
inline void Accepted(fea::FENodalState& owner, Snapshot& out, fea::NodalStamp& stamp) {
  ASSERT_EQ(owner.CopyAccepted(out.Nodes(), &stamp).status, fea::NodalStatus::Ok);
  fea::NodalStamp cin_stamp;
  ASSERT_EQ(owner.CopyAcceptedCin(out.Cin(), &cin_stamp).status, fea::NodalStatus::Ok);
  EXPECT_TRUE(fea::trial_identity::SameStamp(stamp, cin_stamp));
}
inline void Same(const Snapshot& a, const Snapshot& b) {
  ASSERT_EQ(a.nodes.size(), b.nodes.size());
  ASSERT_EQ(a.coefficients.size(), b.coefficients.size());
  EXPECT_EQ(std::memcmp(a.nodes.data(), b.nodes.data(), a.nodes.size()*sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(a.coefficients.data(), b.coefficients.data(), a.coefficients.size()*sizeof(double)), 0);
}
} // namespace cin_runtime_test
