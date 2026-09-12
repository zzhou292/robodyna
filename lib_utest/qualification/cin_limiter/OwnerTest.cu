// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../cin_physical_timestep/OwnerSupport.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include <cstring>

namespace cin_limiter_owner {
namespace support = cin_step_owner;
namespace old = rigid_assembly_owner_test;
namespace fe = tl::fea;
using Code = fe::NodalStatus;
fe::NodalCinAdmission Admission(const fe::NodalAssemblyView& assembly, bool enabled) {
  auto result = support::Admission(assembly);
  result.structural.capture_limiter = enabled;
  return result;
}
TEST(CinLimiterCuda, RepeatedQueryAndEnabledDisabledPacketsMatchAcrossAcceptedIntervals) {
  old::Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(old::Initialize(owner, f, true, true).status, Code::Ok);
  const auto allocation = owner.allocations();
  std::vector<double> kn(f.m.size(), 1e-6), kr(f.m.size());
  for (std::size_t n = 0; n < f.m.size(); ++n) if (f.present[n]) kr[n] = 1e-12;
  for (unsigned epoch = 0; epoch < 3; ++epoch) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    support::Begin(owner, f, kn, kr, token, assembly);
    const auto disabled = fe::AdvanceStaggeredCin(owner, token, Admission(assembly, false));
    ASSERT_EQ(disabled.status, Code::Ok);
    const auto reference = support::Prepared(owner, f, token);
    fe::NodalCinStructuralLimit witness;
    witness.owner_id = 771;
    EXPECT_EQ(owner.CopyPreparedCinStructuralLimit(token, &witness).status, Code::InvalidInput);
    EXPECT_EQ(witness.owner_id, 771u);
    owner.Discard();
    support::Begin(owner, f, kn, kr, token, assembly);
    const auto enabled = fe::AdvanceStaggeredCin(owner, token, Admission(assembly, true));
    ASSERT_EQ(enabled.status, Code::Ok);
    EXPECT_EQ(enabled.stable_dt, disabled.stable_dt);
    EXPECT_EQ(support::Prepared(owner, f, token), reference);
    ASSERT_EQ(owner.CopyPreparedCinStructuralLimit(token, &witness).status, Code::Ok);
    EXPECT_EQ(witness.owner_id, assembly.owner_id);
    EXPECT_EQ(witness.base_epoch, epoch);
    EXPECT_EQ(witness.attempt, assembly.attempt);
    EXPECT_EQ(witness.cin_qualification_id, old::Qualification);
    EXPECT_EQ(witness.source_instance_id, f.domain.source_instance_id());
    EXPECT_EQ(witness.values.minimum_dt_s, enabled.stable_dt);
    EXPECT_EQ(witness.source_node_id, f.domain.nodes()[witness.values.node].source_id);
    if (witness.values.kind == fe::NodalCinLimitKind::RigidTrace) {
      const auto& source = f.binding.groups()[witness.values.group];
      EXPECT_EQ(witness.source_kind, source.source_kind);
      EXPECT_EQ(witness.source_group_id, source.source_id);
      EXPECT_EQ(witness.source_node_set_id, source.source_node_set_id);
    }
    fe::NodalCinStructuralLimit again;
    ASSERT_EQ(owner.CopyPreparedCinStructuralLimit(token, &again).status, Code::Ok);
    EXPECT_EQ(again.attempt, witness.attempt);
    EXPECT_EQ(again.values.minimum_dt_s, witness.values.minimum_dt_s);
    EXPECT_EQ(support::Prepared(owner, f, token), reference);
    old::Commit(owner, token, assembly);
    EXPECT_NE(owner.CopyPreparedCinStructuralLimit(token, &again).status, Code::Ok);
  }
  EXPECT_EQ(owner.allocations().device_bytes, allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations, allocation.device_allocations);
}
TEST(CinLimiterCuda, AliasForeignRejectedAndStaleAttemptsLeaveOutputAndAcceptedStateUntouched) {
  old::Fixture f;
  fe::FENodalState owner, other;
  ASSERT_EQ(old::Initialize(owner, f, true).status, Code::Ok);
  ASSERT_EQ(old::Initialize(other, f, true).status, Code::Ok);
  std::vector<double> kn(f.m.size(), 1e-6), kr(f.m.size());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  support::Begin(owner, f, kn, kr, token, assembly);
  fe::NodalCinStructuralLimit output;
  output.owner_id = 891;
  EXPECT_EQ(owner.CopyPreparedCinStructuralLimit(token, &output).status, Code::WrongPhase);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner, token, Admission(assembly, true)).status, Code::Ok);
  EXPECT_EQ(other.CopyPreparedCinStructuralLimit(token, &output).status, Code::StaleTrial);
  EXPECT_EQ(output.owner_id, 891u);
  alignas(fe::NodalCinStructuralLimit) std::array<unsigned char, sizeof(output)+8> misaligned;
  misaligned.fill(0xA9);
  const auto misaligned_before = misaligned;
  auto* unaligned_output = reinterpret_cast<fe::NodalCinStructuralLimit*>(misaligned.data()+1);
  EXPECT_EQ(owner.CopyPreparedCinStructuralLimit(token, unaligned_output).status, Code::InvalidInput);
  EXPECT_EQ(misaligned, misaligned_before);
  const auto nodes = f.cin_model.domain()->nodes();
  ASSERT_GE(nodes.size() * sizeof(fe::NodalDomainNode), sizeof(output));
  const auto* bytes = reinterpret_cast<const unsigned char*>(nodes.data());
  const std::vector<unsigned char> original(bytes, bytes + sizeof(output));
  auto* alias = reinterpret_cast<fe::NodalCinStructuralLimit*>(const_cast<fe::NodalDomainNode*>(nodes.data()));
  EXPECT_EQ(owner.CopyPreparedCinStructuralLimit(token, alias).status, Code::InvalidInput);
  EXPECT_EQ(std::memcmp(original.data(), nodes.data(), original.size()), 0);
  ASSERT_EQ(owner.CopyPreparedCinStructuralLimit(token, &output).status, Code::Ok);
  const auto prior_token = token;
  const auto good = output;
  owner.Discard();
  old::Snapshot accepted(f.m.size()), after(f.m.size());
  accepted.Read(owner);
  const auto group = f.binding.groups()[1];
  const auto node = f.binding.members()[group.member_offset].domain_node;
  kn[node] = 1e30;
  support::Begin(owner, f, kn, kr, token, assembly);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner, token, Admission(assembly, true)).status, Code::StepTooLarge);
  EXPECT_NE(owner.CopyPreparedCinStructuralLimit(token, &output).status, Code::Ok);
  EXPECT_EQ(output.attempt, good.attempt);
  after.Read(owner);
  old::Same(accepted, after);
  owner.Discard();
  kn[node] = 1e-6;
  support::Begin(owner, f, kn, kr, token, assembly);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner, token, Admission(assembly, true)).status, Code::Ok);
  EXPECT_EQ(owner.CopyPreparedCinStructuralLimit(prior_token, &output).status, Code::StaleTrial);
  ASSERT_EQ(owner.CopyPreparedCinStructuralLimit(token, &output).status, Code::Ok);
  EXPECT_NE(output.attempt, good.attempt);
  EXPECT_EQ(output.values.minimum_dt_s, good.values.minimum_dt_s);
  old::Commit(owner, token, assembly);
}
TEST(CinLimiterCuda, ActualOrdinaryChannelsBothRigidSourceKindsAndExactCapacity) {
  old::Fixture f;
  auto config = f.Config();
  const auto cin = f.Cin();
  const auto forecast = fe::FENodalState::ForecastAssemblyCin(config, f.binding, cin, true);
  ASSERT_EQ(forecast.report.status, Code::Ok);
  config.max_device_bytes = forecast.device_bytes - 1;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config, f.binding, cin, true).report.status, Code::ResourceLimit);
  config.max_device_bytes = forecast.device_bytes;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config, f.binding, cin, true).report.status, Code::Ok);
  fe::FENodalState owner;
  ASSERT_EQ(old::Initialize(owner, f, true).status, Code::Ok);
  std::size_t ordinary = f.m.size();
  for (std::size_t node = 0; node < f.m.size(); ++node) {
    if (!f.present[node] || f.fixed[node] || f.rotation_fixed[node] ||
        f.binding.FindMember(node) || f.m[node] <= 0 || f.j[node] <= 0) continue;
    bool attached = false;
    for (const auto& row : f.cin_model.rows()) {
      attached |= row.secondary_domain_node == node;
      for (const auto master : row.master_domain_nodes) attached |= master == node;
    }
    if (!attached) { ordinary = node; break; }
  }
  ASSERT_LT(ordinary, f.m.size());
  for (unsigned kind = 0; kind < 5; ++kind) {
    std::vector<double> kn(f.m.size()), kr(f.m.size());
    if (kind == 0) kn[ordinary] = .5*f.m[ordinary]/(old::H*old::H);
    if (kind == 1) kr[ordinary] = .5*f.j[ordinary]/(old::H*old::H);
    if (kind == 2 || kind == 3) {
      const auto group = f.binding.groups()[kind-2];
      kn[f.binding.members()[group.member_offset].domain_node] = 1e-6;
    }
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    support::Begin(owner, f, kn, kr, token, assembly);
    const auto advanced = fe::AdvanceStaggeredCin(owner, token, Admission(assembly, true));
    ASSERT_EQ(advanced.status, Code::Ok) << "kind " << kind;
    fe::NodalCinStructuralLimit witness;
    ASSERT_EQ(owner.CopyPreparedCinStructuralLimit(token, &witness).status, Code::Ok);
    if (kind < 2) {
      EXPECT_EQ(witness.values.kind, kind ? fe::NodalCinLimitKind::OrdinaryRotation :
                                          fe::NodalCinLimitKind::OrdinaryTranslation);
      EXPECT_EQ(witness.values.node, ordinary);
      EXPECT_EQ(witness.values.mass_kg, f.m[ordinary]);
      EXPECT_EQ(witness.values.inertia_kg_m2, f.j[ordinary]);
      EXPECT_EQ(witness.values.translation_stiffness_n_per_m, kn[ordinary]);
      EXPECT_EQ(witness.values.rotation_stiffness_nm, kr[ordinary]);
    } else if (kind < 4) {
      EXPECT_EQ(witness.values.kind, fe::NodalCinLimitKind::RigidTrace);
      EXPECT_EQ(witness.values.group, kind-2);
      EXPECT_EQ(witness.source_kind, f.binding.groups()[kind-2].source_kind);
    } else EXPECT_EQ(witness.values.kind, fe::NodalCinLimitKind::Unbounded);
    EXPECT_EQ(witness.values.minimum_dt_s, advanced.stable_dt);
    owner.Discard();
  }
}
} // namespace cin_limiter_owner
