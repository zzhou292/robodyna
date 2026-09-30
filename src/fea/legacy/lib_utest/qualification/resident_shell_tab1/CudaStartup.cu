#include "CudaFixture.h"

namespace resident_tab1_test {
bool Rig::Initialize(Placement plane, bool mismatch, bool omit) {
  Source source(plane);
  if (!source.Prepare(binding, catalog)) return false;
  const auto f = failure.Initialize(catalog, source.failures.data(), source.failures.size());
  EXPECT_EQ(f.status, fe::ShellPlasticityBindingStatus::Success);
  if (f.status != fe::ShellPlasticityBindingStatus::Success) return false;
  initial.n = Nodes;
  initial.h = H;
  for (unsigned n = 0; n < Nodes; ++n) {
    const auto& node = binding.nodes()[n];
    initial.x[3 * n] = node.position.x;
    initial.x[3 * n + 1] = node.position.y;
    initial.x[3 * n + 2] = node.position.z;
    initial.inverse[n] = 1. / node.native.mass;
    initial.inverse_inertia[n] = 1. / node.native.isotropic_inertia;
  }
  const auto initialized = initial.Initialize(owner);
  EXPECT_EQ(initialized.status, fe::NodalStatus::Ok) << initialized.message;
  if (initialized.status != fe::NodalStatus::Ok) return false;
  q::QephBatchConfig qc;
  qc.owner = owner.accepted();
  qc.configuration_id = Configuration;
  qc.qualification_id = Qualification;
  qc.element_count = Parents;
  qc.usage = q::BatchUsage::PrescribedFields;
  t::T3BatchConfig tc;
  tc.owner = qc.owner;
  tc.configuration_id = Configuration;
  tc.qualification_id = Qualification;
  tc.element_count = Parents;
  tc.usage = t::BatchUsage::PrescribedFields;
  fe::ShellBatchFailureBinding other;
  if (mismatch) {
    source.failures.back().tab1.table.triaxiality[2] = .4;
    const auto changed = other.Initialize(catalog, source.failures.data(), source.failures.size());
    EXPECT_EQ(changed.status, fe::ShellPlasticityBindingStatus::Success);
    if (changed.status != fe::ShellPlasticityBindingStatus::Success) return false;
  }
  const auto qr = omit ? qeph.InitializeJoined(qc, binding, catalog) :
      qeph.InitializeJoined(qc, binding, catalog, failure, {});
  const auto tr = omit ? t3.InitializeJoined(tc, binding, catalog) :
      t3.InitializeJoined(tc, binding, catalog, mismatch ? other : failure, {});
  if (omit && plane != Placement::Centered) {
    EXPECT_EQ(qr.status, q::BatchStatus::InvalidInput);
    EXPECT_EQ(tr.status, t::BatchStatus::InvalidInput);
    return qr.status == q::BatchStatus::InvalidInput && tr.status == t::BatchStatus::InvalidInput;
  }
  EXPECT_EQ(qr.status, q::BatchStatus::Success) << qr.message;
  EXPECT_EQ(tr.status, t::BatchStatus::Success) << tr.message;
  // Prove that device parameters and the retained identity do not borrow source rows.
  source.failures.back().tab1.table.failure_strain = -1.;
  return qr.status == q::BatchStatus::Success && tr.status == t::BatchStatus::Success;
}
bool Rig::Bind() {
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  const auto begin = owner.BeginTrial(&token, &view);
  EXPECT_EQ(begin.status, fe::NodalStatus::Ok);
  if (begin.status != fe::NodalStatus::Ok) return false;
  const auto qr = qeph.AssembleAccepted(view);
  const auto tr = t3.AssembleAccepted(view);
  EXPECT_EQ(qr.status, q::BatchStatus::Success) << qr.message;
  EXPECT_EQ(tr.status, t::BatchStatus::Success) << tr.message;
  Discard();
  if (qr.status != q::BatchStatus::Success || tr.status != t::BatchStatus::Success) return false;
  return true;
}
} // namespace resident_tab1_test
