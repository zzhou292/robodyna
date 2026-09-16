// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/collision/RepresentedIntervalCrossing.h"
#include "lib_src/collision/SelfContactTransaction.h"
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/SurfaceContactGeometry.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>
#include <type_traits>
#include <vector>

namespace self_contact_transaction_cuda_test {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace sct = tlfea::contact::self_contact_transaction;
namespace p = physical_publication_test;

bool Good(c::SelfContactTransactionReport report) {
  EXPECT_EQ(report.status, c::SelfContactTransactionStatus::Ok)
      << report.message << " candidate=" << report.candidate
      << " pair=" << report.pair
      << " discovery_task=" << report.discovery_task
      << " discovery_reason="
      << static_cast<unsigned>(report.discovery_reason)
      << " crossing_reason="
      << static_cast<unsigned>(report.crossing_reason);
  return report.status == c::SelfContactTransactionStatus::Ok;
}

struct PriorStreamWork {
  cudaStream_t stream = nullptr;
  cudaEvent_t completed = nullptr;
  void* scratch = nullptr;

  ~PriorStreamWork() {
    if (stream) cudaStreamSynchronize(stream);
    if (completed) cudaEventDestroy(completed);
    if (scratch) cudaFree(scratch);
    if (stream) cudaStreamDestroy(stream);
  }

  bool Queue(cudaStream_t target, unsigned repetition) {
    constexpr std::size_t Bytes = 1u << 20;
    if (cudaStreamCreateWithFlags(
            &stream, cudaStreamNonBlocking) != cudaSuccess ||
        cudaEventCreateWithFlags(
            &completed, cudaEventDisableTiming) != cudaSuccess ||
        cudaMalloc(&scratch, Bytes) != cudaSuccess)
      return false;
    const auto passes = 1 + repetition % 7;
    const auto bytes = (1 + repetition % 4) * (Bytes / 4);
    for (unsigned pass = 0; pass < passes; ++pass)
      if (cudaMemsetAsync(
              scratch, static_cast<int>(repetition + pass),
              bytes, stream) != cudaSuccess)
        return false;
    return cudaEventRecord(completed, stream) == cudaSuccess &&
        cudaStreamWaitEvent(target, completed, 0) == cudaSuccess;
  }
};

void CheckInteriorEeGeometry(const p::Rig& rig) {
  const auto& geometry = rig.fixture.source.shell_input;
  const auto point = [](const tl::math::Vec3& value) {
    return c::Vec3{value.x, value.y, value.z};
  };
  c::SegmentGeometry segments[2];
  segments[0].vertices[0] = point(geometry.t.reference.position[0]);
  segments[0].vertices[1] = point(geometry.t.reference.position[1]);
  segments[1].vertices[0] = point(geometry.q[0].reference.position[3]);
  segments[1].vertices[1] = point(geometry.q[0].reference.position[0]);
  c::SegmentPairGeometry closest;
  ASSERT_EQ(c::ClosestPointsBetweenSegments(
      segments[0], segments[1], &closest), c::Status::kOk);
  EXPECT_GT(closest.parameter_a, 0);
  EXPECT_LT(closest.parameter_a, 1);
  EXPECT_GT(closest.parameter_b, 0);
  EXPECT_LT(closest.parameter_b, 1);
  EXPECT_NEAR(closest.distance, .00025, 1e-15);
}

struct AssemblyFields {
  std::size_t nodes=0;
  std::vector<double> values;
  explicit AssemblyFields(std::size_t count) : nodes(count),values(8*count) {}
  bool Read(const fe::NodalAssemblyView& view,
            const fe::NodalCinAssemblyView& cin) {
    double* source[]{
        view.forces.force_x,view.forces.force_y,view.forces.force_z,
        view.forces.couple_x,view.forces.couple_y,view.forces.couple_z,
        cin.translational_stiffness,cin.rotational_stiffness};
    for (unsigned channel=0;channel<8;++channel)
      if (cudaMemcpyAsync(values.data()+channel*nodes,source[channel],
              nodes*sizeof(double),cudaMemcpyDeviceToHost,view.stream) !=
          cudaSuccess)
        return false;
    return cudaStreamSynchronize(view.stream) == cudaSuccess;
  }
  c::Vec3 Vector(unsigned channel,std::size_t node) const {
    return {values[channel*nodes+node],
            values[(channel+1)*nodes+node],
            values[(channel+2)*nodes+node]};
  }
};

void Append(c::Vec3 value, std::vector<std::uint64_t>* output) {
  output->push_back(p::Bits(value.x));
  output->push_back(p::Bits(value.y));
  output->push_back(p::Bits(value.z));
}

std::vector<std::uint64_t> FieldBits(const AssemblyFields& fields) {
  std::vector<std::uint64_t> result;
  result.reserve(fields.values.size());
  for (const auto value : fields.values)
    result.push_back(p::Bits(value));
  return result;
}

std::vector<std::uint64_t> DiagnosticBits(
    const c::SelfContactForceDiagnostics& value) {
  std::vector<std::uint64_t> result{
      value.event_count, value.vertex_face_event_count,
      value.boundary_vertex_edge_event_count, value.edge_edge_event_count,
      value.active_count};
  Append(value.endpoint_a_resultant_n, &result);
  Append(value.endpoint_b_resultant_n, &result);
  Append(value.equal_opposite_residual_n, &result);
  Append(value.global_moment_n_m, &result);
  result.insert(result.end(), {
      p::Bits(value.potential_j),
      p::Bits(value.maximum_force_norm_n),
      p::Bits(value.maximum_sti_diagonal_n_m),
      p::Bits(value.maximum_represented_stiffness_n_m),
      value.base_epoch, value.attempt,
      value.configuration_id, value.qualification_id,
      value.first_source_order, value.last_source_order,
      static_cast<std::uint64_t>(value.temporal_scheme),
      static_cast<std::uint64_t>(value.velocity_phase),
      p::Bits(value.position_time), p::Bits(value.velocity_time),
      static_cast<std::uint64_t>(value.valid)});
  // owner_id and active_use_identity are lifetime identities. The coupon
  // authenticates them against each fresh fixture instead of comparing
  // unrelated object addresses/IDs as numerical output.
  return result;
}

std::vector<std::uint64_t> PolicySummaryBits(
    const c::SelfContactCandidatePolicySummary& value) {
  return {
      value.outcomes, value.certified_separated,
      value.excluded_same_rigid_group,
      value.excluded_local_intersection,
      value.represented_by_accepted_vf,
      value.represented_by_accepted_ee,
      value.motion_certified_linear_separated,
      value.axis_certified_linear_separated,
      value.edge_axis_certified_linear_separated,
      value.motion_excluded_same_rigid_group,
      value.exact_crossing_pairs, value.exact_crossing_work,
      value.digest, static_cast<std::uint64_t>(value.complete),
      static_cast<std::uint64_t>(value.detailed_publication),
      value.vertex_edge_axis_separated,
      value.vertex_vertex_axis_separated,
      value.motion_certified_nonlinear_accepted_coverage,
      value.motion_certified_nonlinear_exact_exclusion};
}

std::vector<std::uint64_t> PolicyOutcomeBits(
    c::SelfContactCandidatePolicyView view,
    std::vector<std::uint64_t>* event_order) {
  std::vector<std::uint64_t> result{
      view.count, static_cast<std::uint64_t>(view.complete)};
  for (std::size_t i = 0; i < view.count; ++i) {
    const auto& value = view.data[i];
    for (const auto& path : value.pair.paths) {
      result.push_back(path.source_instance_id);
      result.push_back(path.parent_eid);
      result.push_back(path.level);
      result.push_back(path.local_facet);
    }
    result.push_back(static_cast<std::uint64_t>(value.disposition));
    result.push_back(value.accepted_event);
    result.push_back(value.source_order);
    if (value.disposition ==
            c::SelfContactCandidateDisposition::
                RepresentedByAcceptedVertexFace ||
        value.disposition ==
            c::SelfContactCandidateDisposition::
                RepresentedByAcceptedEdgeEdge) {
      EXPECT_NE(value.accepted_event, SIZE_MAX);
      EXPECT_NE(value.source_order, UINT64_MAX);
      event_order->push_back(value.accepted_event);
      event_order->push_back(value.source_order);
    }
  }
  return result;
}

std::vector<std::uint64_t> AcceptedReceiptBits(
    const c::SelfContactAcceptedAssemblyReceipt& value) {
  auto result = DiagnosticBits(value.diagnostics());
  result.insert(result.end(), {
      value.broadphase_pairs(), value.facet_pairs(),
      value.discovered_features(), value.potential_tasks(),
      value.local_masked_tasks(), value.exact_executed_tasks(),
      static_cast<std::uint64_t>(value.valid())});
  return result;
}

std::vector<std::uint64_t> TransactionReceiptBits(
    const c::SelfContactTransactionReceipt& value) {
  auto result = PolicySummaryBits(value.policy_summary());
  result.insert(result.end(), {
      value.source_id(), value.regularity_generation(),
      value.broadphase_pairs(), value.facet_pairs(),
      value.potential_tasks(), value.local_masked_tasks(),
      value.exact_executed_tasks(), value.policy_outcomes(),
      value.active_parents(), value.removing_parents(),
      value.skipped_parents(),
      static_cast<std::uint64_t>(value.valid())});
  return result;
}

std::vector<std::uint64_t> StampBits(const fe::NodalStamp& value) {
  return {
      value.epoch, value.node_count, p::Bits(value.time),
      p::Bits(value.fixed_dt), static_cast<std::uint64_t>(value.has_rotations),
      static_cast<std::uint64_t>(value.reactions_valid),
      value.reaction_base_epoch, p::Bits(value.reaction_time),
      static_cast<std::uint64_t>(value.temporal_scheme),
      static_cast<std::uint64_t>(value.velocity_phase),
      p::Bits(value.velocity_time), p::Bits(value.reaction_kick_dt),
      value.rigid_groups.source_instance_id,
      value.rigid_groups.group_count, value.rigid_groups.member_count,
      value.rigid_groups.part_group_count,
      value.rigid_groups.plain_source_instance_id,
      static_cast<std::uint64_t>(value.has_rotation_presence)};
}

struct DeterminismObservation {
  std::vector<std::uint64_t> fields;
  std::vector<std::uint64_t> rollback_receipt;
  std::vector<std::uint64_t> accepted_receipt;
  std::vector<std::uint64_t> transaction_receipt;
  std::vector<std::uint64_t> policy_summary;
  std::vector<std::uint64_t> policy_outcomes;
  std::vector<std::uint64_t> canonical_event_order;
  std::vector<std::uint64_t> accepted_state;
  std::vector<std::uint64_t> final_stamp;
};

c::Vec3 Cross(c::Vec3 a,c::Vec3 b) {
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
double Norm(c::Vec3 a) {
  return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
}
void Near(c::Vec3 actual,c::Vec3 expected,double scale=1) {
  const double tolerance=2e-12*std::max({1.,scale,Norm(expected)});
  EXPECT_NEAR(actual.x,expected.x,tolerance);
  EXPECT_NEAR(actual.y,expected.y,tolerance);
  EXPECT_NEAR(actual.z,expected.z,tolerance);
}
c::Vec3 DenseAngularAcceleration(const fe::RigidBindingGroup& body,
                                 c::Vec3 moment) {
  c::Vec3 result;
  const auto& axes=body.principal.axes;
  const double inverse[]{
      1/body.principal.inertia.x,1/body.principal.inertia.y,
      1/body.principal.inertia.z};
  for (unsigned axis=0;axis<3;++axis) {
    const c::Vec3 column{
        axes.v[axis],axes.v[3+axis],axes.v[6+axis]};
    const double local=column.x*moment.x+column.y*moment.y+
        column.z*moment.z;
    result=c::Add(result,c::Scale(column,inverse[axis]*local));
  }
  return result;
}

struct Fixture {
  explicit Fixture(bool single = false, bool crossing = false,
                   double t3_failure = 2.5,
                   p::ContactConstraintLayout constraints =
                       p::ContactConstraintLayout::Legacy,
                   unsigned facet_level = 0,
                   unsigned input_permutation = 0)
      : rig(false, t3_failure, !single, constraints,
            !single && constraints == p::ContactConstraintLayout::Legacy),
        single_parent(single), pass_through(crossing),
        t3_failure(t3_failure), facet_level(facet_level),
        input_permutation(input_permutation) {
    if (!single &&
        constraints == p::ContactConstraintLayout::Legacy)
      CheckInteriorEeGeometry(rig);
    if (pass_through) {
      rig.external_force_source_node = 14;
      const auto apex = rig.fixture.domain.Find(14);
      const double mass =
          rig.fixture.ledger.nodes()[apex].coefficients.mass;
      rig.external_force_z_n =
          -2 * mass * .00075 / (p::H * p::H);
    }
  }
  p::Rig rig;
  fe::ShellBatchPlasticityBinding execution_catalog;
  fe::ShellBatchFailureBinding execution_failure;
  fe::ShellExecutionBinding execution;
  fe::ShellPhysicalBinding physical;
  c::SelfContactSurfaceBinding surface;
  c::FixedContactFacetBinding facets;
  c::SelfContactActiveUseBinding uses;
  c::SelfContactTransaction transaction;
  std::vector<c::SelfContactParentSelection> selection;
  c::SelfContactTransactionConfig config;
  cudaStream_t owner_stream = nullptr;
  bool single_parent = false;
  bool pass_through = false;
  double t3_failure = 2.5;
  unsigned facet_level = 0;
  unsigned input_permutation = 0;
  bool authority_prepared = false;

  bool PrepareExecutionAuthority() {
    qbat_catalog_test::Fixture declaration;
    std::array<fe::ShellPlasticityMaterialInput,4> materials;
    std::array<fe::ShellPlasticitySectionInput,4> sections;
    std::copy(declaration.materials.begin(), declaration.materials.end(),
              materials.begin());
    std::copy(declaration.sections.begin(), declaration.sections.end(),
              sections.begin());
    for (unsigned row = 0; row < 2; ++row) {
      materials[row].curve_id = 0;
      materials[row].hardening =
          tl::material::ShellPlasticityHardeningKind::LinearLaw44;
      materials[row].linear = {10e6, 0};
      materials[row].rate = materials[2].rate;
    }
    const bool rigid_contact =
        rig.fixture.contact_constraints ==
            p::ContactConstraintLayout::SameMergedParts ||
        rig.fixture.contact_constraints ==
            p::ContactConstraintLayout::MergedPartAndPlain;
    const bool same_merged_parts =
        rig.fixture.contact_constraints ==
            p::ContactConstraintLayout::SameMergedParts;
    if (rigid_contact) {
      materials[3] = materials[2];
      materials[3].material_id = 3000;
      materials[3].curve_id = 0;
      materials[3].hardening =
          tl::material::ShellPlasticityHardeningKind::Tabulated;
      materials[3].rate = {};
      materials[3].linear = {};
      materials[3].law = fe::ShellSectionLaw::RigidSkin;
      sections[3] = sections[2];
      sections[3].section_id = 3000;
      sections[3].through_thickness_points = 0;
      sections[3].formulation =
          fe::ShellSectionFormulation::Nonconstitutive;
    }
    std::array<fe::ShellPlasticityParentInput,5> parents{{
        {fe::ShellBindingFamily::Qeph, 0, 100, 1000, 1000, 1000},
        {fe::ShellBindingFamily::Qeph, 1, 101, 1001, 1001, 1001},
        {fe::ShellBindingFamily::T3, 0, 102, 2000524, 2000524, 2000524},
        {fe::ShellBindingFamily::Qbat, 0, 103, 2000524, 2000524, 2000524},
        {fe::ShellBindingFamily::T3, 1, 104, 2000524, 2000524, 2000524}}};
    if (rigid_contact) {
      parents[2].source_part_id = 3000;
      parents[2].material_id = 3000;
      parents[2].section_id = 3000;
      if (same_merged_parts) {
        parents[4].source_part_id = 3001;
        parents[4].material_id = 3000;
        parents[4].section_id = 3000;
      }
    }
    const std::size_t parent_count=rigid_contact ? parents.size() : 4;
    const auto catalog_report =
        execution_catalog.InitializeExecutionCatalog(
            rig.fixture.source.shells,
            {nullptr, materials.data(), sections.data(), parents.data(),
             0, rigid_contact ? 4u : 3u,
             rigid_contact ? 4u : 3u, parent_count});
    EXPECT_EQ(catalog_report.status,
              fe::ShellPlasticityBindingStatus::Success)
        << catalog_report.message << " entry=" << catalog_report.entry
        << " family=" << static_cast<int>(catalog_report.family);
    if (catalog_report.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;

    fe::ShellFailureParentInput failures[5];
    for (std::size_t row=0;row<parent_count;++row)
      failures[row] = qbat_catalog_test::Failure(parents[row]);
    if (rigid_contact) {
      failures[2] = {};
      failures[2].source = parents[2];
      if (same_merged_parts) {
        failures[4] = {};
        failures[4].source = parents[4];
      }
      for (const unsigned row : {0u,1u}) {
        failures[row].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
        failures[row].constant = {};
        failures[row].tab1.table = {{-1, 0, 1}, 1};
      }
    } else {
      failures[2].constant.failure_strain = t3_failure;
      for (unsigned row = 0; row < 2; ++row) {
        failures[row].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
        failures[row].constant = {};
        failures[row].tab1.table = {{-1, 0, 1}, 1};
      }
    }
    const auto failure_report = execution_failure.InitializeExecution(
        execution_catalog, failures, parent_count);
    EXPECT_EQ(failure_report.status,
              fe::ShellPlasticityBindingStatus::Success)
        << failure_report.message;
    if (failure_report.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;

    const auto execution_report = execution.Initialize(
        execution_catalog, rig.fixture.ledger, rig.fixture.rigid);
    EXPECT_EQ(execution_report.status,
              fe::ShellPlasticityBindingStatus::Success)
        << execution_report.message << " entry=" << execution_report.entry;
    if (execution_report.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;
    const auto physical_report = physical.InitializeExecution(
        {&rig.fixture.source.shells, &execution_catalog,
         &execution_failure, nullptr},
        rig.fixture.ledger, execution);
    if (!p::Good(physical_report)) return false;

    for (std::size_t row = 0;
         row < execution_catalog.parent_count(); ++row) {
      const auto& parent = *execution_catalog.parent(row);
      if (rigid_contact) {
        if (parent.family != fe::ShellBindingFamily::T3)
          continue;
      } else if (parent.family != fe::ShellBindingFamily::T3 &&
                 parent.family != fe::ShellBindingFamily::Qbat) {
        continue;
      }
      if (single_parent && !selection.empty()) continue;
      selection.push_back({
          row, parent.family, parent.family_index,
          parent.source_parent_id, parent.source_part_id});
    }
    if (selection.size() > 1) {
      const auto shift = input_permutation % selection.size();
      std::rotate(selection.begin(), selection.begin() + shift,
                  selection.end());
      if (input_permutation & 1)
        std::reverse(selection.begin(), selection.end());
    }
    const auto surface_report = surface.Initialize(
        physical, {selection.data(), selection.size()});
    EXPECT_EQ(surface_report.status, c::SelfContactSurfaceStatus::Ok)
        << surface_report.message;
    if (surface_report.status != c::SelfContactSurfaceStatus::Ok)
      return false;
    const auto facet_report =
        facets.Initialize(surface, {{}, facet_level});
    EXPECT_EQ(facet_report.status, c::FixedContactFacetStatus::Ok)
        << facet_report.message;
    if (facet_report.status != c::FixedContactFacetStatus::Ok)
      return false;
    const auto cin = rig.fixture.WitnessSource();
    c::SelfContactActiveUseSource source;
    source.rigid = &rig.fixture.rigid;
    source.cin = {
        cin.model, cin.ranges, cin.witnesses,
        cin.range_count, cin.witness_count};
    const auto use_report = uses.Initialize(facets, source);
    EXPECT_EQ(use_report.status, c::SelfContactActiveUseStatus::Ok)
        << use_report.message << " parent=" << use_report.parent
        << " feature=" << use_report.feature;
    if (use_report.status != c::SelfContactActiveUseStatus::Ok)
      return false;
    authority_prepared = true;
    return true;
  }

  bool InitializeInfrastructure() {
    if (!authority_prepared && !PrepareExecutionAuthority()) return false;
    // Owner, q/t/qbat/PART/plain/CIN mapped participants and publication all
    // consume the same execution-authenticated physical authority retained by
    // surface/facets/uses.
    return rig.InitializeAgainst(physical);
  }

  bool Initialize(c::SelfContactTransactionLimits limits = {}) {
    if (!InitializeInfrastructure()) return false;
    if (rig.owner.BorrowOwnerStream(&owner_stream).status !=
        fe::NodalStatus::Ok)
      return false;
    config.force.owner = rig.owner.accepted();
    config.force.startup = rig.fixture.Identity().startup;
    config.force.stiffness_per_area_n_m3 = 2e9;
    config.force.event_capacity = 512;
    config.force.configuration_id = p::Configuration;
    config.force.qualification_id = p::Qualification;
    config.source_id = p::SelfContactSource;
    const auto plan = c::SelfContactTransaction::Forecast(
        config, uses, rig.fixture.Identity(), limits);
    if (!Good(plan.report)) return false;
    if (!Good(transaction.Initialize(
            config, uses, rig.owner, rig.publication,
            physical, rig.Participants(),
            rig.fixture.Identity(), owner_stream, limits)))
      return false;
    fe::ShellPhysicalScratchRoster roster{
        {}, transaction.roster_entry()};
    if (!p::Good(
            rig.publication.ConfigurePhysicalScratchParticipation(
                rig.owner, physical, rig.Participants(),
                rig.fixture.Identity(), roster,
                limits.participation)))
      return false;
    return true;
  }

  std::size_t Parent(std::uint64_t eid) const {
    for (std::size_t parent = 0;
         parent < uses.parents().size(); ++parent)
      if (uses.parents()[parent].source.source_parent_id == eid)
        return parent;
    return SIZE_MAX;
  }

  std::uint32_t ProbeNode() {
    const auto t3 = Parent(102);
    const auto qbat = Parent(103);
    if (t3 == SIZE_MAX || qbat == SIZE_MAX) return UINT32_MAX;
    std::size_t vertex_use = SIZE_MAX;
    for (std::size_t vertex = 0;
         vertex < uses.vertex_uses().size(); ++vertex) {
      const auto& use = uses.vertex_uses()[vertex];
      if (use.parent == t3 &&
          use.key.kind == c::FacetVertexKind::SourceVertex &&
          use.key.first == 14) {
        vertex_use = vertex;
        break;
      }
    }
    if (vertex_use == SIZE_MAX) return UINT32_MAX;
    const auto feature = uses.vertex_uses()[vertex_use].feature;
    for (std::size_t facet_index = 0;
         facet_index < uses.facet_uses().size(); ++facet_index) {
      const auto& use = uses.facet_uses()[facet_index];
      if (use.parent != qbat) continue;
      bool incident = false;
      for (const auto vertex : use.vertex_features)
        incident = incident || vertex == feature;
      if (incident) continue;
      return uses.vertex_uses()[vertex_use].point.nodes[0];
    }
    return UINT32_MAX;
  }

  bool DriveT3Removal() {
    const auto apex = rig.fixture.domain.Find(14);
    if (apex == SIZE_MAX) return false;
    const double mass =
        rig.fixture.ledger.nodes()[apex].coefficients.mass;
    if (!(mass > 0)) return false;
    rig.external_force_source_node = 14;
    rig.external_force_z_n =
        -2 * mass * .01 / (p::H * p::H);
    return true;
  }

  bool Prepare(const fe::NodalTrialToken& token,
               const fe::NodalAssemblyView& assembly,
               fe::NodalPreparedView& prepared,
               fe::ShellPhysicalDiagnostics& common) {
    fe::ShellPhysicalDiagnostics candidates;
    return rig.Advance(token, assembly, prepared) &&
        rig.Evaluate(token, prepared, candidates) &&
        p::Good(rig.publication.PreparePhysical(
            rig.owner, token,
            {&candidates.qeph, &candidates.t3, &candidates.qbat,
             &candidates.type25, &candidates.type13,
             &candidates.solids},
            &common));
  }

  bool Commit(const fe::NodalTrialToken& token,
              const fe::NodalPreparedView& prepared,
              const fe::ShellPhysicalDiagnostics& common,
              const c::SelfContactTransactionReceipt& receipt) {
    return p::Good(
               rig.publication.SealPhysicalScratchParticipation(
                   rig.owner, token, receipt.scratch_receipts())) &&
        p::Good(rig.publication.CommitPhysical(
            rig.owner, token, common,
            {prepared.owner_id,
             prepared.kinematics.base_epoch,
             prepared.attempt, p::Qualification, true}));
  }

  void Discard() {
    rig.owner.Discard();
    rig.publication.DiscardTrial();
    rig.qeph.DiscardTrial();
    rig.t3.DiscardTrial();
    rig.qbat.DiscardTrial();
    rig.welds.DiscardTrial();
    rig.beams.DiscardTrial();
    rig.solids.DiscardTrial();
    transaction.DiscardTrial();
  }
};

TEST(SelfContactTransactionCuda,
     CertifiedRigidSweepsSeparateDistantBodiesButNotOverlappingArcs) {
  const auto group = [](std::uint64_t id, c::Vec3 center,
                        c::Vec3 omega) {
    fe::NodalRigidGroupSnapshot result;
    result.source_kind = fe::RigidBindingSourceKind::Part;
    result.source_group_id = id;
    result.source_node_set_id = id + 100;
    result.state.center = {center.x, center.y, center.z};
    result.state.omega = {omega.x, omega.y, omega.z};
    return result;
  };
  constexpr auto trajectory =
      fe::NodalRigidMemberTrajectory::
          EndpointCorrectedSecondOrderDriftV1;
  const auto accepted_a = group(1, {0, 0, 0}, {});
  const auto prepared_a = group(1, {.01, 0, 0}, {0, 0, .001});
  const auto accepted_b = group(2, {100, 0, 0}, {});
  const auto prepared_b = group(2, {100.01, 0, 0}, {0, 0, -.001});
  c::SelfContactSweptParentBounds a, distant, overlapping;
  ASSERT_EQ(sct::BuildRigidMemberSweepBounds(
      {1, 0, 0}, {1.01, .00001, 0},
      accepted_a, prepared_a, trajectory, .01, &a),
      sct::RigidMemberSweepStatus::Ok);
  ASSERT_EQ(sct::BuildRigidMemberSweepBounds(
      {101, 0, 0}, {101.01, -.00001, 0},
      accepted_b, prepared_b, trajectory, .01, &distant),
      sct::RigidMemberSweepStatus::Ok);
  auto overlap_accepted = group(2, {.5, 0, 0}, {});
  auto overlap_prepared = group(2, {.51, 0, 0}, {0, 0, -.001});
  ASSERT_EQ(sct::BuildRigidMemberSweepBounds(
      {1, 0, 0}, {1.01, .000005, 0},
      overlap_accepted, overlap_prepared,
      trajectory, .01, &overlapping),
      sct::RigidMemberSweepStatus::Ok);

  sct::MotionSupport rigid_a;
  rigid_a.motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  rigid_a.complete_rigid_group = 0;
  rigid_a.rigid_groups[0] = 0;
  rigid_a.rigid_group_count = 1;
  auto rigid_b = rigid_a;
  rigid_b.complete_rigid_group = 1;
  rigid_b.rigid_groups[0] = 1;
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, a, rigid_b, distant),
      sct::PairMotionAction::CertifiedRigidArcSeparation);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, a, rigid_b, overlapping),
      sct::PairMotionAction::UnsupportedRigidArc);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, a, rigid_a, overlapping),
      sct::PairMotionAction::ExcludedSameRigidGroup);
}

TEST(SelfContactTransactionCuda,
     ExactAffineMixedCertificateAndDecisionAreRepeatable) {
  const double accepted_values[]{1, 0, 0, 0, 2, 0, 0, 0, 3};
  const double prepared_values[]{
      1.01, 0, 0, .01, 2, 0, .01, 0, 3};
  const c::VectorView accepted{accepted_values, 3, 3, 1};
  const c::VectorView prepared{prepared_values, 3, 3, 1};
  const std::uint32_t node_groups[]{0, UINT32_MAX, UINT32_MAX};
  fe::NodalRigidGroupSnapshot accepted_group;
  accepted_group.source_kind = fe::RigidBindingSourceKind::Part;
  accepted_group.source_group_id = 91;
  accepted_group.source_node_set_id = 92;
  fe::NodalRigidGroupSnapshot prepared_group = accepted_group;
  prepared_group.state.center = {.01, 0, 0};
  prepared_group.state.omega = {};
  c::WeightedSurfacePoint point;
  point.count = 3;
  point.nodes[0] = 0;
  point.nodes[1] = 1;
  point.nodes[2] = 2;
  point.weights[0] = .25;
  point.weights[1] = .75;
  point.weights[2] = 0;
  constexpr auto trajectory =
      fe::NodalRigidMemberTrajectory::
          EndpointCorrectedSecondOrderDriftV1;
  const c::SelfContactSweptParentBounds overlap{
      {-1, -1, -1}, {1, 1, 1}};
  sct::MotionSupport ordinary;
  ordinary.certified_affine = true;
  sct::MotionSupport mixed;
  mixed.motion = c::SelfContactFacetMotion::PartialOrMixedRigid;

  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    bool affine = false;
    ASSERT_EQ(sct::CertifyRigidPointAffineMotion(
        point, accepted, prepared, node_groups,
        &accepted_group, &prepared_group, 1,
        trajectory, .01, &affine),
        sct::RigidMemberSweepStatus::Ok);
    ASSERT_TRUE(affine);
    mixed.certified_affine = affine;
    EXPECT_EQ(sct::ClassifyCandidatePairMotion(
        ordinary, overlap, mixed, overlap),
        sct::PairMotionAction::LinearNodalV1);
  }
}

TEST(SelfContactTransactionCuda,
     NonlinearSubdivisionDecisionIsRepeatableAndFailClosed) {
  const auto triangle = [](double x, double y) {
    c::CurrentFixedTriangle value;
    value.vertices[0] = {x - .02, y - .01, 0};
    value.vertices[1] = {x + .02, y - .01, 0};
    value.vertices[2] = {x, y + .02, 0};
    return value;
  };
  const auto coefficients = [](double qy) {
    sct::FacetQuadraticCoefficients value;
    value.complete = true;
    for (unsigned vertex = 0; vertex < 3; ++vertex)
      value.q[vertex][1] = {qy, qy};
    return value;
  };
  const auto first_accepted = triangle(-1, 0);
  const auto first_prepared = triangle(1, 0);
  const auto second_accepted = triangle(1, 0);
  const auto second_prepared = triangle(-1, 0);
  sct::NonlinearSeparationResult reference;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result = sct::CertifyQuadraticFacetSeparation(
        first_accepted, first_prepared, coefficients(-8), .01,
        second_accepted, second_prepared, coefficients(8), .01,
        1, 4095, 20);
    EXPECT_EQ(result.status,
              sct::NonlinearSeparationStatus::CertifiedSeparated);
    if (!repeat) reference = result;
    EXPECT_EQ(result.status, reference.status);
    EXPECT_EQ(result.work, reference.work);
    EXPECT_EQ(result.deepest, reference.deepest);
  }

  const auto fixed = triangle(0, 0);
  const auto moving = triangle(0, 0);
  auto elevated = moving;
  for (auto& vertex : elevated.vertices) vertex.z = 1;
  auto separated_q = coefficients(0);
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    separated_q.q[vertex][2] = {4, 4};
  const auto linear_quadratic =
      sct::CertifyQuadraticFacetSeparation(
          fixed, fixed, coefficients(0), .01,
          elevated, elevated, separated_q, .01,
          1, 4095, 20);
  EXPECT_EQ(linear_quadratic.status,
            sct::NonlinearSeparationStatus::CertifiedSeparated);
  EXPECT_EQ(linear_quadratic.work, 3u);

  auto contact_q = coefficients(0);
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    contact_q.q[vertex][2] = {8, 8};
  EXPECT_NE(sct::CertifyQuadraticFacetSeparation(
                fixed, fixed, coefficients(0), .01,
                elevated, elevated, contact_q, .01,
                1, 4095, 20).status,
            sct::NonlinearSeparationStatus::CertifiedSeparated);
}

TEST(SelfContactTransactionCuda,
     QuadraticResidualCertificateIsBitwiseRepeatable) {
  const auto triangle = [](std::uint64_t eid, double z) {
    c::CurrentFixedTriangle value;
    value.key = {17, eid, 0, 0};
    value.vertices[0] = {0, 0, z};
    value.vertices[1] = {2, 0, z};
    value.vertices[2] = {0, 2, z};
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      value.vertex_keys[vertex].source_instance_id = 17;
      value.vertex_keys[vertex].first =
          10 * eid + vertex;
      value.vertex_keys[vertex].denominator = 1;
    }
    for (unsigned edge = 0; edge < 3; ++edge) {
      auto first_key = value.vertex_keys[edge];
      auto second_key = value.vertex_keys[(edge + 1) % 3];
      if (c::fixed_triangle_features::Compare(
              second_key, first_key) < 0)
        std::swap(first_key, second_key);
      value.edge_keys[edge].parent_boundary = true;
      value.edge_keys[edge].endpoints[0] = first_key;
      value.edge_keys[edge].endpoints[1] = second_key;
    }
    return value;
  };
  const auto quadratic = [](double z) {
    sct::FacetQuadraticCoefficients value;
    value.complete = true;
    for (unsigned vertex = 0; vertex < 3; ++vertex)
      value.q[vertex][2] = {z, z};
    return value;
  };
  const auto first = triangle(10, 0);
  const auto second = triangle(20, 1);
  std::array<c::FixedTriangleFeatureCandidate, 15> features;
  c::fixed_triangle_features::PairFeatureResult geometry;
  ASSERT_EQ(
      c::fixed_triangle_features::EvaluatePairFeaturesOnce(
          first, second, features.data(), features.size(),
          &geometry),
      c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_EQ(geometry.feature_count, features.size());

  sct::LinearResidualSeparationResult reference;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result =
        sct::CertifyQuadraticResidualSeparation(
            first, first, quadratic(1), .1,
            second, second, quadratic(0), .1, 1,
            {features.data(), features.size(), true},
            {nullptr, 0, true});
    ASSERT_EQ(
        result.status,
        sct::LinearResidualSeparationStatus::
            CertifiedSeparated);
    if (!repeat) reference = result;
    EXPECT_EQ(std::memcmp(
                  &result, &reference, sizeof(result)),
              0);
  }
  EXPECT_EQ(
      sct::CertifyQuadraticResidualSeparation(
          first, first, quadratic(8), .1,
          second, second, quadratic(0), .1, 1,
          {features.data(), features.size(), true},
          {nullptr, 0, true}).status,
      sct::LinearResidualSeparationStatus::PotentialContact);
}

TEST(SelfContactTransactionCuda,
     CommonTranslationCertificateIsDeterministicAtMinimalCap) {
  const auto vertex = [](std::uint64_t id) {
    c::FacetVertexKey result;
    result.source_instance_id = 17;
    result.first = id;
    return result;
  };
  const auto path = [&](std::uint64_t eid,
                        const std::array<c::Vec3, 3>& base,
                        const std::array<c::Vec3, 3>& current,
                        std::uint64_t vertex_base) {
    c::RepresentedTrianglePath result;
    result.key = {17, eid, 0, 0};
    for (unsigned i = 0; i < 3; ++i) {
      result.vertices[i].key = vertex(vertex_base + i);
      result.vertices[i].endpoint[0] = base[i];
      result.vertices[i].endpoint[1] = current[i];
    }
    for (unsigned i = 0; i < 3; ++i) {
      auto& edge = result.edge_keys[i];
      edge.parent_boundary = true;
      edge.endpoints[0] = result.vertices[i].key;
      edge.endpoints[1] = result.vertices[(i + 1) % 3].key;
      if (edge.endpoints[1].first < edge.endpoints[0].first)
        std::swap(edge.endpoints[0], edge.endpoints[1]);
    }
    return result;
  };
  const auto translate = [](std::array<c::Vec3, 3> value) {
    for (auto& point : value) {
      point.x += 4;
      point.y -= 3;
      point.z += 2;
    }
    return value;
  };
  const std::array<c::Vec3, 3> first{{
      {0, 0, 0}, {2, 0, 0}, {0, 2, 0}}};
  const std::array<c::Vec3, 3> second{{
      {1.5, 1.5, 0}, {3.5, 1.5, 0}, {1.5, 3.5, 0}}};
  const std::array<c::RepresentedTrianglePath, 2> paths{
      path(10, first, translate(first), 100),
      path(20, second, translate(second), 200)};
  const c::RepresentedTrianglePair pair{0, 1};
  c::RepresentedIntervalResult reference;
  bool have_reference = false;
  for (unsigned workers : {1u, 4u}) {
    c::RepresentedIntervalLimits limits;
    limits.max_paths = 2;
    limits.max_input_pairs = 1;
    limits.max_results = 1;
    limits.max_work_per_pair = 1;
    limits.max_total_work = 1;
    limits.max_depth = 20;
    limits.worker_count = workers;
    c::RepresentedIntervalCrossing crossing;
    ASSERT_EQ(crossing.Initialize(limits).status,
              c::RepresentedIntervalStatus::Ok);
    for (unsigned repeat = 0; repeat < 16; ++repeat) {
      const auto report =
          crossing.Certify(paths.data(), paths.size(), &pair, 1);
      ASSERT_EQ(report.status, c::RepresentedIntervalStatus::Ok);
      ASSERT_EQ(crossing.results().count, 1u);
      const auto result = crossing.results().data[0];
      EXPECT_EQ(
          result.classification,
          c::RepresentedIntervalClassification::CertifiedSeparated);
      EXPECT_EQ(result.reason, c::RepresentedIntervalReason::None);
      EXPECT_EQ(result.work, 1u);
      if (!have_reference) {
        reference = result;
        have_reference = true;
      } else {
        EXPECT_EQ(std::memcmp(
                      &result, &reference, sizeof(result)),
                  0);
      }
    }
  }
}

TEST(SelfContactTransactionCuda,
     ResidualTranslationCertificateIsDeterministic) {
  const auto vertex = [](std::uint64_t id) {
    c::FacetVertexKey result;
    result.source_instance_id = 17;
    result.first = id;
    return result;
  };
  const auto triangle = [&](std::uint64_t eid,
                            std::uint64_t first_vertex,
                            double z) {
    c::CurrentFixedTriangle result;
    result.key = {17, eid, 0, 0};
    const c::Vec3 points[3]{
        {0, 0, z}, {2, 0, z}, {0, 2, z}};
    for (unsigned i = 0; i < 3; ++i) {
      result.vertices[i] = points[i];
      result.vertex_keys[i] = vertex(first_vertex + i);
    }
    for (unsigned i = 0; i < 3; ++i) {
      auto a = result.vertex_keys[i];
      auto b = result.vertex_keys[(i + 1) % 3];
      if (c::fixed_triangle_features::Compare(b, a) < 0)
        std::swap(a, b);
      result.edge_keys[i].parent_boundary = true;
      result.edge_keys[i].endpoints[0] = a;
      result.edge_keys[i].endpoints[1] = b;
    }
    return result;
  };
  const auto first_base = triangle(10, 100, 0);
  const auto second_base = triangle(20, 200, 1);
  auto first_prepared = first_base;
  auto second_prepared = second_base;
  const double translation = std::ldexp(1.0, -52);
  for (auto* value : {&first_prepared, &second_prepared})
    for (auto& point : value->vertices)
      point.x += translation;

  std::array<c::FixedTriangleFeatureCandidate, 15> features;
  c::fixed_triangle_features::PairFeatureResult feature_result;
  ASSERT_EQ(
      c::fixed_triangle_features::EvaluatePairFeaturesOnce(
          first_prepared, second_prepared,
          features.data(), features.size(), &feature_result),
      c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_EQ(feature_result.feature_count, 15u);
  bool intersects = false;
  c::FixedTriangleIntersection intersection;
  ASSERT_EQ(
      c::fixed_triangle_features::ClassifyPairIntersection(
          first_prepared, second_prepared,
          &intersection, &intersects),
      c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_FALSE(intersects);

  sct::LinearResidualSeparationResult reference;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result =
        sct::CertifyLinearResidualSeparation(
            first_base, first_prepared, .1,
            second_base, second_prepared, .1,
            {features.data(), feature_result.feature_count, true},
            {nullptr, 0, true});
    ASSERT_EQ(
        result.status,
        sct::LinearResidualSeparationStatus::
            CertifiedSeparated);
    ASSERT_FALSE(result.exact_common_translation);
    if (!repeat)
      reference = result;
    EXPECT_EQ(result.reference_translation.x,
              reference.reference_translation.x);
    EXPECT_EQ(result.first_residual_upper_m,
              reference.first_residual_upper_m);
    EXPECT_EQ(result.second_residual_upper_m,
              reference.second_residual_upper_m);
    EXPECT_EQ(result.prepared_distance_lower_m,
              reference.prepared_distance_lower_m);
    EXPECT_EQ(result.strict_gap_lower_m,
              reference.strict_gap_lower_m);
  }

  const auto close_base = triangle(20, 200, .15);
  auto close_prepared = close_base;
  for (auto& point : close_prepared.vertices)
    point.x += translation;
  std::array<c::FixedTriangleFeatureCandidate, 15>
      close_features;
  c::fixed_triangle_features::PairFeatureResult
      close_feature_result;
  ASSERT_EQ(
      c::fixed_triangle_features::EvaluatePairFeaturesOnce(
          first_prepared, close_prepared,
          close_features.data(), close_features.size(),
          &close_feature_result),
      c::FixedTriangleDiscoveryStatus::Ok);
  const auto persistent_found = std::find_if(
      close_features.begin(),
      close_features.begin() + close_feature_result.feature_count,
      [](const auto& feature) {
        return feature.key.kind ==
            c::FixedTriangleCandidateKind::EdgeEdge;
      });
  ASSERT_NE(persistent_found,
            close_features.begin() +
                close_feature_result.feature_count);
  const auto persistent_feature = *persistent_found;
  sct::AcceptedEventCertificate accepted;
  accepted.kind = sct::AcceptedEventCertificateKind::EdgeEdge;
  accepted.discovery = persistent_feature;
  accepted.event.feature = persistent_feature.key;
  accepted.event.source_order = 0;
  accepted.event.edge_use[0] = 1;
  accepted.event.edge_use[1] = 2;
  accepted.edge_facet[0] = 1;
  accepted.edge_facet[1] = 2;
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    accepted.event.endpoints[endpoint].count = 3;
    accepted.event.endpoints[endpoint].nodes[0] =
        3 * endpoint;
    accepted.event.endpoints[endpoint].nodes[1] =
        3 * endpoint + 1;
    accepted.event.endpoints[endpoint].nodes[2] =
        3 * endpoint + 2;
    accepted.event.endpoints[endpoint].weights[0] = .5;
    accepted.event.endpoints[endpoint].weights[1] = .5;
  }
  accepted.event.classification.kind =
      c::SelfContactPairKind::EdgeEdge;
  accepted.event.classification.status =
      c::SelfContactPairStatus::AdmittedEdgeEdge;
  accepted.event.classification.active[0] = true;
  accepted.event.classification.active[1] = true;
  accepted.event.classification.reference_half_thickness_m[0] =
      .1;
  accepted.event.classification.reference_half_thickness_m[1] =
      .1;
  accepted.event.classification.candidate_directed_area_m2 =
      {1, 1, 1, 0};
  accepted.event.classification.admitted_force_area_m2 =
      {1, 1, 1, 0};
  sct::PersistentLinearContactResult persistent_reference;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result = sct::CertifyPersistentLinearContact(
        first_base, first_prepared, .1,
        close_base, close_prepared, .1,
        {&persistent_feature, 1, true}, &accepted, 1);
    ASSERT_EQ(
        result.status,
        sct::PersistentLinearContactStatus::CertifiedContact);
    ASSERT_FALSE(result.exact_common_translation);
    if (!repeat)
      persistent_reference = result;
    EXPECT_EQ(result.feature.kind,
              persistent_reference.feature.kind);
    EXPECT_EQ(result.accepted_certificate,
              persistent_reference.accepted_certificate);
    EXPECT_EQ(result.strict_thickness_margin_lower_m,
              persistent_reference.strict_thickness_margin_lower_m);
  }
  sct::FacetQuadraticCoefficients zero_quadratic;
  zero_quadratic.complete = true;
  sct::NonlinearSeparationResult coverage_reference;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result = sct::CertifyQuadraticFacetCoverage(
        first_base, first_prepared, zero_quadratic, .1,
        close_base, close_prepared, zero_quadratic, .1,
        1, &accepted, 1, 4095, 20);
    ASSERT_EQ(
        result.status,
        sct::NonlinearSeparationStatus::
            CertifiedAcceptedCoverage);
    if (!repeat) coverage_reference = result;
    EXPECT_EQ(result.work, coverage_reference.work);
    EXPECT_EQ(result.deepest, coverage_reference.deepest);
    EXPECT_EQ(
        result.accepted_source_order,
        coverage_reference.accepted_source_order);
    EXPECT_EQ(
        result.proof_digest,
        coverage_reference.proof_digest);
  }
  const sct::AcceptedFeatureExclusionCertificate exclusion{
      persistent_feature, 7};
  sct::NonlinearSeparationResult exclusion_reference;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result =
        sct::CertifyQuadraticFacetPolicyCoverage(
            first_base, first_prepared, zero_quadratic, .1,
            close_base, close_prepared, zero_quadratic, .1,
            1, nullptr, 0, &exclusion, 1, 4095, 20);
    ASSERT_EQ(
        result.status,
        sct::NonlinearSeparationStatus::
            CertifiedExactExclusion);
    if (!repeat) exclusion_reference = result;
    EXPECT_EQ(result.work, exclusion_reference.work);
    EXPECT_EQ(
        result.excluded_rigid_group,
        exclusion_reference.excluded_rigid_group);
    EXPECT_EQ(
        result.proof_digest,
        exclusion_reference.proof_digest);
  }
  accepted.discovery.triangles[0].parent_eid = 9;
  accepted.discovery.triangles[1].parent_eid = 19;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result = sct::CertifyPersistentLinearContact(
        first_base, first_prepared, .1,
        close_base, close_prepared, .1,
        {&persistent_feature, 1, true}, &accepted, 1);
    ASSERT_EQ(
        result.status,
        sct::PersistentLinearContactStatus::CertifiedContact);
    EXPECT_EQ(result.accepted_certificate, 0u);
    EXPECT_EQ(result.feature.kind,
              c::RepresentedFeatureKind::EdgeEdge);
  }

  c::FixedTriangleFeatureCandidate face_feature;
  face_feature.key.vertex_face.vertex =
      first_prepared.vertex_keys[0];
  face_feature.key.vertex_face.target.SetFace(
      close_prepared.key);
  face_feature.triangles[0] = first_prepared.key;
  face_feature.triangles[1] = close_prepared.key;
  face_feature.local_features[0] = 0;
  face_feature.local_features[1] = 3;
  face_feature.points[0] = first_prepared.vertices[0];
  face_feature.points[1] = close_prepared.vertices[0];
  face_feature.face_weights[0] = 1;
  face_feature.face_weights[1] = 0;
  face_feature.face_weights[2] =
      std::numeric_limits<double>::denorm_min();
  face_feature.distance_m = .15;
  sct::AcceptedEventCertificate face_accepted = accepted;
  face_accepted.kind =
      sct::AcceptedEventCertificateKind::VertexFace;
  face_accepted.discovery = face_feature;
  face_accepted.event.feature = face_feature.key;
  face_accepted.event.vertex_use = 1;
  face_accepted.event.facet_use = 2;
  face_accepted.event.edge_use[0] = UINT32_MAX;
  face_accepted.event.edge_use[1] = UINT32_MAX;
  face_accepted.vertex_facet = 1;
  face_accepted.target_facet = 2;
  face_accepted.edge_facet[0] = UINT32_MAX;
  face_accepted.edge_facet[1] = UINT32_MAX;
  face_accepted.event.classification.kind =
      c::SelfContactPairKind::VertexFace;
  face_accepted.event.classification.status =
      c::SelfContactPairStatus::AdmittedVertexFace;
  sct::PersistentLinearContactResult face_reference;
  for (unsigned repeat = 0; repeat < 64; ++repeat) {
    const auto result = sct::CertifyPersistentLinearContact(
        first_base, first_prepared, .1,
        close_base, close_prepared, .1,
        {&face_feature, 1, true}, &face_accepted, 1);
    ASSERT_EQ(
        result.status,
        sct::PersistentLinearContactStatus::CertifiedContact);
    ASSERT_GT(result.face_weight_normalization_upper_m, 0);
    if (!repeat)
      face_reference = result;
    EXPECT_EQ(result.face_weight_normalization_upper_m,
              face_reference.face_weight_normalization_upper_m);
    EXPECT_EQ(result.strict_thickness_margin_lower_m,
              face_reference.strict_thickness_margin_lower_m);
  }
}

TEST(SelfContactTransactionCuda,
     ExactLocalIntersectionPrecedesOnlyUnsupportedMotion) {
  const c::RepresentedIntervalPairKey pair{{
      {17, 10, 0, 0}, {17, 20, 0, 0}}};
  c::RepresentedIntervalResult crossing;
  crossing.key = pair;
  crossing.classification =
      c::RepresentedIntervalClassification::Unresolved;
  crossing.reason = c::RepresentedIntervalReason::UnsupportedMotion;
  c::FixedTriangleIntersection intersection;
  intersection.triangles[0] = {17, 10, 0, 0};
  intersection.triangles[1] = {17, 20, 0, 0};
  intersection.kind =
      c::FixedTriangleIntersectionKind::Transverse;
  intersection.local_exclusion =
      c::FixedTriangleLocalExclusion::SharedVertexOnly;
  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t outcome_count = 0;
  sct::CandidateValidationInput input;
  input.canonical_pairs = &pair;
  input.pair_count = 1;
  input.features.complete = true;
  input.intersections = {&intersection, 1, true};
  input.crossings = {&crossing, 1, true};
  input.outcomes = &outcome;
  input.outcome_capacity = 1;
  input.outcome_count = &outcome_count;
  ASSERT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome_count, 1u);
  EXPECT_EQ(outcome.disposition,
            c::SelfContactCandidateDisposition::
                ExcludedLocalIntersection);

  crossing.reason = c::RepresentedIntervalReason::WorkExhausted;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::UnresolvedCandidate);
  crossing.reason = c::RepresentedIntervalReason::UnsupportedMotion;
  intersection.local_exclusion =
      c::FixedTriangleLocalExclusion::None;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::UnresolvedCandidate);
}

TEST(SelfContactTransactionCuda,
     ExactForecastCapMinusOneAndRosterEntryAreStable) {
  Fixture fixture;
  ASSERT_TRUE(fixture.InitializeInfrastructure());
  fixture.config.force.owner = fixture.rig.owner.accepted();
  fixture.config.force.stiffness_per_area_n_m3 = 2e9;
  fixture.config.force.event_capacity = 512;
  fixture.config.force.configuration_id = p::Configuration;
  fixture.config.force.qualification_id = p::Qualification;
  fixture.config.source_id = p::SelfContactSource;
  auto limits = c::SelfContactTransactionLimits{};
  const auto exact = c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits);
  ASSERT_TRUE(Good(exact.report));
  EXPECT_EQ(exact.forecast.nonlinear_subdivision_work_per_pair,
            limits.max_nonlinear_subdivision_work_per_pair);
  EXPECT_EQ(exact.forecast.nonlinear_subdivision_work_per_chunk,
            limits.max_nonlinear_subdivision_work_per_chunk);
  EXPECT_EQ(
      exact.forecast.complete_nonlinear_subdivision_work_capacity,
      limits.max_stream_nonlinear_subdivision_work);
  EXPECT_EQ(exact.forecast.nonlinear_subdivision_depth,
            limits.max_nonlinear_subdivision_depth);
  EXPECT_EQ(exact.forecast.shared_backing_discount_bytes,
            exact.forecast.broadphase.retained_source_bytes +
                exact.forecast.force.retained_active_use_bytes);
  limits.max_host_bytes = exact.forecast.owned_host_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.max_host_bytes;
  limits.activity.max_host_bytes =
      exact.forecast.activity.owned_host_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.activity.max_host_bytes;
  limits.max_device_bytes = exact.forecast.device_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.max_device_bytes;
  limits.max_startup_host_bytes =
      exact.forecast.startup_host_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.max_startup_host_bytes;
  cudaStream_t owner_stream = nullptr;
  ASSERT_EQ(fixture.rig.owner.BorrowOwnerStream(&owner_stream).status,
            fe::NodalStatus::Ok);
  EXPECT_EQ(fixture.rig.owner.ValidateOwnerStream(nullptr).status,
            fe::NodalStatus::InvalidInput);
  EXPECT_EQ(fixture.rig.owner.ValidateOwnerStream(
                cudaStreamLegacy).status,
            fe::NodalStatus::InvalidInput);
  EXPECT_EQ(fixture.rig.owner.ValidateOwnerStream(owner_stream).status,
            fe::NodalStatus::Ok);
  ASSERT_TRUE(Good(fixture.transaction.Initialize(
      fixture.config, fixture.uses, fixture.rig.owner,
      fixture.rig.publication, fixture.physical,
      fixture.rig.Participants(), fixture.rig.fixture.Identity(),
      owner_stream, limits)));
  const auto entry = fixture.transaction.roster_entry();
  EXPECT_NE(entry.issuer, nullptr);
  EXPECT_EQ(entry.source_id, p::SelfContactSource);
  EXPECT_EQ(fixture.transaction.allocations().device.device_allocations, 2u);
  EXPECT_EQ(fixture.transaction.allocations().device.device_bytes,
            exact.forecast.device_bytes);
  EXPECT_EQ(fixture.transaction.allocations().activity.host_bytes,
            exact.forecast.activity.arena_bytes);
}

TEST(SelfContactTransactionCuda,
     SingleParentZeroPairZeroEventStillParticipatesAndCommits) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  EXPECT_EQ(accepted.broadphase_pairs(), 0u);
  EXPECT_EQ(accepted.facet_pairs(), 0u);
  EXPECT_EQ(accepted.potential_tasks(), 0u);
  EXPECT_EQ(accepted.local_masked_tasks(), 0u);
  EXPECT_EQ(accepted.exact_executed_tasks(), 0u);
  EXPECT_EQ(accepted.diagnostics().event_count, 0u);

  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  c::SelfContactTransactionReceipt receipt;
  ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, accepted, &receipt)));
  EXPECT_EQ(receipt.broadphase_pairs(), 0u);
  EXPECT_EQ(receipt.facet_pairs(), 0u);
  EXPECT_EQ(receipt.potential_tasks(), 0u);
  EXPECT_EQ(receipt.local_masked_tasks(), 0u);
  EXPECT_EQ(receipt.exact_executed_tasks(), 0u);
  EXPECT_EQ(receipt.policy_outcomes(), 0u);
  const auto policy = fixture.transaction.policy_outcomes();
  EXPECT_TRUE(policy.complete);
  EXPECT_EQ(policy.count, 0u);
  EXPECT_EQ(policy.data, nullptr);
  ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);
}

TEST(SelfContactTransactionCuda,
     AcceptedInteriorEeForceCandidateRetryAndRollbackKeepForceSti) {
  // Level one keeps the physical fixture small while producing one mixed
  // candidate stream: exact contact pairs and strict swept-box separations.
  Fixture fixture(false, false, 2.5,
                  p::ContactConstraintLayout::Legacy, 1);
  ASSERT_TRUE(fixture.Initialize());
  const auto node = fixture.ProbeNode();
  ASSERT_NE(node, UINT32_MAX);
  const auto allocation = fixture.transaction.allocations();
  {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactTransactionReceipt unchanged;
    EXPECT_NE(fixture.transaction.SealCandidate(
        fixture.rig.owner, token, {}, {}, {}, &unchanged).status,
        c::SelfContactTransactionStatus::Ok);
    EXPECT_FALSE(unchanged.valid());
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  }
  {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    EXPECT_EQ(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly,
        reinterpret_cast<c::SelfContactAcceptedAssemblyReceipt*>(
            &assembly)).status,
        c::SelfContactTransactionStatus::InvalidInput);
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  }
  for (unsigned interval = 0; interval < 2; ++interval) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token, &cin)));
    double before_sti = 0, after_sti = 0;
    ASSERT_EQ(cudaMemcpyAsync(
        &before_sti, cin.translational_stiffness + node,
        sizeof(before_sti), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    EXPECT_GT(accepted.broadphase_pairs(), 0u);
    EXPECT_GT(accepted.facet_pairs(), 0u);
    EXPECT_EQ(accepted.potential_tasks(),
              accepted.local_masked_tasks() +
                  accepted.exact_executed_tasks());
    EXPECT_GT(accepted.diagnostics().event_count, 0u);
    EXPECT_GT(accepted.diagnostics().vertex_face_event_count,0u);
    EXPECT_GT(
        accepted.diagnostics().boundary_vertex_edge_event_count,0u);
    EXPECT_GT(accepted.diagnostics().edge_edge_event_count,0u);
    EXPECT_EQ(accepted.diagnostics().vertex_face_event_count+
              accepted.diagnostics().edge_edge_event_count,
              accepted.diagnostics().event_count);
    EXPECT_GT(accepted.diagnostics().active_count, 0u);
    EXPECT_EQ(accepted.diagnostics().first_source_order, 0u);
    EXPECT_EQ(accepted.diagnostics().last_source_order,
              accepted.diagnostics().event_count - 1);
    EXPECT_GT(
        accepted.diagnostics().maximum_represented_stiffness_n_m, 0);
    ASSERT_EQ(cudaMemcpyAsync(
        &after_sti, cin.translational_stiffness + node,
        sizeof(after_sti), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    EXPECT_GT(accepted.diagnostics().maximum_force_norm_n, 0);
    EXPECT_GT(accepted.diagnostics().maximum_sti_diagonal_n_m, 0);
    EXPECT_GE(after_sti, before_sti);

    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    if (interval == 0) {
      EXPECT_EQ(fixture.rig.publication.CommitPhysical(
          fixture.rig.owner, token, common,
          {prepared.owner_id, prepared.kinematics.base_epoch,
           prepared.attempt, p::Qualification, true}).status,
          fe::ShellPublicationStatus::ParticipationFailure);
      EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);

      ASSERT_TRUE(fixture.rig.Begin(token, assembly));
      ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
          fixture.rig.owner, token, assembly, &accepted)));
      ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
      c::SelfContactTransactionReceipt unchanged;
      EXPECT_NE(fixture.transaction.SealCandidate(
          fixture.rig.owner, token, common, {}, accepted,
          &unchanged).status,
          c::SelfContactTransactionStatus::Ok);
      EXPECT_FALSE(unchanged.valid());
      EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);

      ASSERT_TRUE(fixture.rig.Begin(token, assembly));
      ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
          fixture.rig.owner, token, assembly, &accepted)));
      ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    }
    c::SelfContactTransactionReceipt receipt;
    ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &receipt)));
    ASSERT_TRUE(receipt.valid());
    EXPECT_EQ(receipt.potential_tasks(),
              receipt.local_masked_tasks() +
                  receipt.exact_executed_tasks());
    const auto& policy = receipt.policy_summary();
    EXPECT_TRUE(policy.complete);
    EXPECT_GT(policy.motion_certified_linear_separated,0u);
    EXPECT_LE(policy.axis_certified_linear_separated,
              policy.motion_certified_linear_separated);
    EXPECT_LE(policy.edge_axis_certified_linear_separated,
              policy.axis_certified_linear_separated);
    EXPECT_LE(policy.vertex_edge_axis_separated,
              policy.axis_certified_linear_separated -
                  policy.edge_axis_certified_linear_separated);
    EXPECT_LE(policy.vertex_vertex_axis_separated,
              policy.axis_certified_linear_separated -
                  policy.edge_axis_certified_linear_separated -
                  policy.vertex_edge_axis_separated);
    EXPECT_GT(policy.exact_crossing_pairs,0u);
    EXPECT_GT(policy.represented_by_accepted_ee, 0u);
    EXPECT_LT(policy.exact_crossing_pairs,policy.outcomes);
    EXPECT_EQ(
        policy.outcomes,
        policy.motion_certified_linear_separated +
            policy.motion_excluded_same_rigid_group +
            policy.exact_crossing_pairs);
    ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
    EXPECT_EQ(fixture.rig.owner.accepted().epoch,
              static_cast<std::uint64_t>(interval + 1));
    EXPECT_EQ(p::Bits(fixture.rig.owner.accepted().reaction_kick_dt),
              p::Bits(interval ? p::H : .5 * p::H));
    EXPECT_EQ(fixture.transaction.allocations().device.device_bytes,
              allocation.device.device_bytes);
    EXPECT_EQ(fixture.transaction.allocations().device.device_allocations,
              allocation.device.device_allocations);
    EXPECT_EQ(fixture.transaction.allocations().activity.host_bytes,
              allocation.activity.host_bytes);
  }
}

TEST(SelfContactTransactionCuda,
     ActualMergedRigidBodyExcludesDiscoveredVfBeforeForceOrSti) {
  Fixture fixture(false,false,2.5,
      p::ContactConstraintLayout::SameMergedParts);
  ASSERT_TRUE(fixture.Initialize());
  ASSERT_EQ(fixture.rig.fixture.rigid.groups().size(),1u);
  EXPECT_EQ(fixture.rig.fixture.rigid.groups()[0].source_kind,
            fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(fixture.rig.fixture.topology.part_count(),2u);

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(fixture.rig.Begin(token,assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token,&cin)));
  AssemblyFields before(fixture.rig.fixture.domain.node_count()),after(before.nodes);
  ASSERT_TRUE(before.Read(assembly,cin));
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner,token,assembly,&accepted)));
  EXPECT_GT(accepted.broadphase_pairs(),0u);
  EXPECT_GT(accepted.facet_pairs(),0u);
  EXPECT_EQ(accepted.discovered_features(),0u);
  EXPECT_EQ(accepted.diagnostics().event_count,0u);
  EXPECT_EQ(accepted.diagnostics().active_count,0u);
  EXPECT_EQ(accepted.diagnostics().maximum_force_norm_n,0);
  EXPECT_EQ(accepted.diagnostics().maximum_sti_diagonal_n_m,0);
  ASSERT_TRUE(after.Read(assembly,cin));
  EXPECT_EQ(after.values,before.values);

  fixture.Discard();
  EXPECT_EQ(fixture.rig.owner.accepted().epoch,0u);
}

TEST(SelfContactTransactionCuda,
     ActualMergedPartAndPlainBodiesUseMergedWrenchesBeforeInverseResponse) {
  Fixture fixture(false,false,2.5,
      p::ContactConstraintLayout::MergedPartAndPlain);
  ASSERT_TRUE(fixture.Initialize());
  const auto& binding=fixture.rig.fixture.rigid;
  ASSERT_EQ(binding.groups().size(),2u);
  ASSERT_EQ(fixture.rig.fixture.topology.part_count(),2u);
  EXPECT_EQ(binding.groups()[0].source_kind,fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(binding.groups()[1].source_kind,
            fe::RigidBindingSourceKind::NodalGroup);
  EXPECT_EQ(binding.groups()[0].source_id,binding.groups()[1].source_id);
  ASSERT_NE(fixture.Parent(102),SIZE_MAX);
  ASSERT_NE(fixture.Parent(104),SIZE_MAX);

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(fixture.rig.Begin(token,assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token,&cin)));
  AssemblyFields before(fixture.rig.fixture.domain.node_count()),after(before.nodes);
  ASSERT_TRUE(before.Read(assembly,cin));
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner,token,assembly,&accepted)));
  ASSERT_GT(accepted.diagnostics().event_count,0u);
  ASSERT_GT(accepted.diagnostics().active_count,0u);
  ASSERT_TRUE(after.Read(assembly,cin));

  std::vector<double> positions(3*after.nodes);
  ASSERT_EQ(cudaMemcpyAsync(positions.data(),assembly.accepted.position_xyz,
      positions.size()*sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),
      cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
  std::array<c::Vec3,2> expected_acceleration{},expected_angular{};
  c::Vec3 contact_resultant,contact_moment;
  std::array<std::size_t,2> contacted_nodes{};
  std::array<c::Vec3,2> endpoint_inverse_sum{},merged_contact_acceleration{};
  for (std::size_t g=0;g<binding.groups().size();++g) {
    const auto& group=binding.groups()[g];
    c::Vec3 force,moment,contact_force;
    for (std::size_t slot=0;slot<group.member_count;++slot) {
      const auto& member=
          binding.members()[group.member_offset+slot];
      const auto node=member.domain_node;
      const c::Vec3 x{positions[3*node],positions[3*node+1],
                      positions[3*node+2]};
      const auto node_force=after.Vector(0,node);
      const auto node_couple=after.Vector(3,node);
      force=c::Add(force,node_force);
      moment=c::Add(moment,c::Add(node_couple,
          Cross(c::Subtract(x,{group.center.x,group.center.y,group.center.z}),
                node_force)));
      const auto increment=c::Subtract(node_force,before.Vector(0,node));
      contact_force=c::Add(contact_force,increment);
      contact_resultant=c::Add(contact_resultant,increment);
      contact_moment=c::Add(contact_moment,Cross(x,increment));
      if (Norm(increment)>0) {
        ASSERT_GT(member.mass_kg,0);
        ++contacted_nodes[g];
        endpoint_inverse_sum[g]=c::Add(endpoint_inverse_sum[g],
            c::Scale(increment,1/member.mass_kg));
      }
    }
    expected_acceleration[g]=c::Scale(force,1/group.mass_kg);
    expected_angular[g]=DenseAngularAcceleration(group,moment);
    merged_contact_acceleration[g]=c::Scale(contact_force,1/group.mass_kg);
  }
  // The fully rigid T3 face contributes multiple weighted PART nodes. The
  // opposite ordinary T3 contributes its contacting vertex to the partial
  // plain group; its third node remains ordinary, so this is not an
  // unsupported complete plain-rigid shell skin.
  EXPECT_GE(contacted_nodes[0],2u);
  EXPECT_GT(Norm(merged_contact_acceleration[0]),0);
  EXPECT_GT(Norm(c::Subtract(endpoint_inverse_sum[0],
                         merged_contact_acceleration[0])),
            1e-6*Norm(merged_contact_acceleration[0]));
  const double contact_scale=accepted.diagnostics().maximum_force_norm_n;
  Near(contact_resultant,accepted.diagnostics().equal_opposite_residual_n,
       contact_scale);
  Near(contact_moment,accepted.diagnostics().global_moment_n_m,
       contact_scale);
  Near(contact_resultant,{},contact_scale);

  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token,assembly,prepared,common));
  std::vector<double> force_stage(6*after.nodes);
  std::array<fe::NodalRigidGroupAccelerationSnapshot,2> actual;
  fe::NodalPreparedView captured;
  ASSERT_EQ(fixture.rig.owner.CopyPreparedForceStage(token,
      {force_stage.data(),force_stage.data()+3*after.nodes,after.nodes,
       actual.data(),actual.size()},&captured).status,fe::NodalStatus::Ok);
  EXPECT_EQ(captured.attempt,prepared.attempt);
  for (std::size_t g=0;g<actual.size();++g) {
    EXPECT_EQ(actual[g].source_kind,binding.groups()[g].source_kind);
    EXPECT_EQ(actual[g].source_group_id,binding.groups()[g].source_id);
    Near({actual[g].acceleration.x,actual[g].acceleration.y,
          actual[g].acceleration.z},expected_acceleration[g]);
    Near({actual[g].angular_acceleration.x,actual[g].angular_acceleration.y,
          actual[g].angular_acceleration.z},expected_angular[g]);
  }
  c::SelfContactTransactionReceipt completed;
  const auto rigid_interval=fixture.transaction.SealCandidate(
      fixture.rig.owner,token,common,prepared,accepted,&completed);
  EXPECT_EQ(rigid_interval.status,
            c::SelfContactTransactionStatus::Ok);
  EXPECT_TRUE(completed.valid());
  fixture.Discard();
  EXPECT_EQ(fixture.rig.owner.accepted().epoch,0u);
}

TEST(SelfContactTransactionCuda,
     ActualT3RemovalFiltersCandidateAndLongInactiveRetryCommits) {
  Fixture fixture(false, false, 1.e-12);
  ASSERT_TRUE(fixture.DriveT3Removal());
  ASSERT_TRUE(fixture.Initialize());
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  auto forged = common;
  ++forged.t3.attempt;
  c::SelfContactTransactionReceipt unchanged;
  const auto rejected = fixture.transaction.SealCandidate(
      fixture.rig.owner, token, forged, prepared, accepted, &unchanged);
  EXPECT_EQ(rejected.status,
            c::SelfContactTransactionStatus::ActivityFailure);
  EXPECT_FALSE(unchanged.valid());
  EXPECT_FALSE(accepted.valid());
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  ASSERT_TRUE(fixture.rig.Read(after));
  p::Exact(before, after);

  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  std::uint8_t prepared_t3_activity = 1;
  ASSERT_TRUE(p::Good(fixture.rig.t3.CopyPreparedParentActivity(
      fixture.rig.owner, token, common.t3,
      &prepared_t3_activity, 1)));
  ASSERT_EQ(prepared_t3_activity, 0u);
  c::SelfContactTransactionReceipt removal;
  ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, accepted, &removal)));
  EXPECT_FALSE(accepted.valid());
  EXPECT_EQ(removal.active_parents(), 1u);
  EXPECT_EQ(removal.removing_parents(), 1u);
  EXPECT_EQ(removal.skipped_parents(), 0u);
  EXPECT_EQ(removal.facet_pairs(), 0u);
  EXPECT_EQ(removal.policy_outcomes(), 0u);
  const auto expired = accepted;
  ASSERT_TRUE(fixture.Commit(token, prepared, common, removal));
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);

  c::SelfContactAcceptedAssemblyReceipt inactive;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &inactive)));
  EXPECT_EQ(inactive.facet_pairs(), 0u);
  EXPECT_EQ(inactive.diagnostics().event_count, 0u);
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  EXPECT_NE(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, expired,
      &unchanged).status, c::SelfContactTransactionStatus::Ok);
  EXPECT_FALSE(unchanged.valid());
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);

  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &inactive)));
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  c::SelfContactTransactionReceipt long_inactive;
  ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, inactive,
      &long_inactive)));
  EXPECT_EQ(long_inactive.active_parents(), 1u);
  EXPECT_EQ(long_inactive.removing_parents(), 0u);
  EXPECT_EQ(long_inactive.skipped_parents(), 1u);
  EXPECT_EQ(long_inactive.facet_pairs(), 0u);
  EXPECT_EQ(long_inactive.policy_outcomes(), 0u);
  ASSERT_TRUE(fixture.Commit(
      token, prepared, common, long_inactive));
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 2u);
}

TEST(SelfContactTransactionCuda,
     AlgorithmicDeterminismAcrossSchedulingWorkersAndLifetimes) {
  using Storage = std::aligned_storage_t<
      sizeof(Fixture), alignof(Fixture)>;
  Storage fixture_storage;
  DeterminismObservation reference;
  bool have_reference = false;
  constexpr unsigned Workers[]{1, 2, 4};

  for (unsigned repetition = 0; repetition < 32; ++repetition) {
    SCOPED_TRACE(repetition);
    // Reuse the exact same enclosing address while reconstructing every owner,
    // transaction, worker pool and CUDA allocation on every repetition.
    auto* fixture = new (&fixture_storage) Fixture(
        false, false, 2.5, p::ContactConstraintLayout::Legacy,
        1, repetition);
    struct DestroyFixture {
      Fixture* value;
      ~DestroyFixture() { value->~Fixture(); }
    } destroy{fixture};

    const auto workers = Workers[repetition % 3];
    c::SelfContactTransactionLimits limits;
    limits.accepted_discovery.worker_count = workers;
    limits.candidate_discovery.worker_count = workers;
    limits.crossing.worker_count = workers;
    fixture->config.broadphase_axis = repetition % 3;
    ASSERT_TRUE(fixture->Initialize(limits));
    EXPECT_LT(fixture->transaction.forecast().owned_host_bytes,
              512u << 20);

    p::Snapshot initial, rolled_back;
    ASSERT_TRUE(fixture->rig.Read(initial));
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture->rig.Begin(token, assembly));
    PriorStreamWork first_perturbation;
    ASSERT_TRUE(first_perturbation.Queue(
        assembly.stream, 2 * repetition));
    c::SelfContactAcceptedAssemblyReceipt rolled_back_receipt;
    ASSERT_TRUE(Good(fixture->transaction.AssembleAccepted(
        fixture->rig.owner, token, assembly,
        &rolled_back_receipt)));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture->Prepare(
        token, assembly, prepared, common));
    EXPECT_EQ(fixture->rig.publication.CommitPhysical(
        fixture->rig.owner, token, common,
        {prepared.owner_id, prepared.kinematics.base_epoch,
         prepared.attempt, p::Qualification, true}).status,
        fe::ShellPublicationStatus::ParticipationFailure);
    ASSERT_TRUE(fixture->rig.Read(rolled_back));
    p::Exact(initial, rolled_back);

    ASSERT_TRUE(fixture->rig.Begin(token, assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(
        fixture->rig.owner.BorrowCinAssembly(token, &cin)));
    AssemblyFields before(fixture->rig.fixture.domain.node_count()),
                   after(before.nodes);
    ASSERT_TRUE(before.Read(assembly, cin));
    PriorStreamWork retry_perturbation;
    ASSERT_TRUE(retry_perturbation.Queue(
        assembly.stream, 2 * repetition + 1));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture->transaction.AssembleAccepted(
        fixture->rig.owner, token, assembly, &accepted)));
    ASSERT_TRUE(after.Read(assembly, cin));
    EXPECT_EQ(accepted.diagnostics().owner_id, assembly.owner_id);
    EXPECT_EQ(accepted.diagnostics().active_use_identity,
              fixture->uses.identity());
    ASSERT_GT(accepted.diagnostics().event_count, 0u);
    EXPECT_EQ(accepted.diagnostics().first_source_order, 0u);
    EXPECT_EQ(accepted.diagnostics().last_source_order,
              accepted.diagnostics().event_count - 1);

    ASSERT_TRUE(fixture->Prepare(
        token, assembly, prepared, common));
    c::SelfContactTransactionReceipt receipt;
    ASSERT_TRUE(Good(fixture->transaction.SealCandidate(
        fixture->rig.owner, token, common, prepared, accepted,
        &receipt)));
    const auto summary = fixture->transaction.policy_summary();
    const auto outcomes = fixture->transaction.policy_outcomes();
    ASSERT_TRUE(summary.complete);
    ASSERT_TRUE(summary.detailed_publication);
    ASSERT_TRUE(outcomes.complete);
    ASSERT_EQ(outcomes.count, summary.outcomes);
    ASSERT_GT(outcomes.count, 0u);

    DeterminismObservation observation;
    observation.fields = FieldBits(after);
    observation.rollback_receipt =
        AcceptedReceiptBits(rolled_back_receipt);
    observation.accepted_receipt = AcceptedReceiptBits(accepted);
    observation.transaction_receipt =
        TransactionReceiptBits(receipt);
    observation.policy_summary = PolicySummaryBits(summary);
    observation.policy_outcomes = PolicyOutcomeBits(
        outcomes, &observation.canonical_event_order);
    // A candidate may certify every pair separated even though accepted
    // assembly contained contact events.  The resulting empty reference
    // vector is still part of the bitwise repeated observation.
    for (std::size_t i = 0;
         i < observation.canonical_event_order.size(); i += 2) {
      EXPECT_LT(observation.canonical_event_order[i],
                accepted.diagnostics().event_count);
      EXPECT_LT(observation.canonical_event_order[i + 1],
                accepted.diagnostics().event_count);
    }

    ASSERT_TRUE(fixture->Commit(
        token, prepared, common, receipt));
    p::Snapshot final;
    ASSERT_TRUE(fixture->rig.Read(final));
    EXPECT_NE(final.stamp.owner_id, 0u);
    EXPECT_EQ(final.stamp.epoch, 1u);
    observation.accepted_state = final.values;
    observation.final_stamp = StampBits(final.stamp);

    if (!have_reference) {
      reference = observation;
      have_reference = true;
    } else {
      EXPECT_EQ(observation.fields, reference.fields);
      EXPECT_EQ(observation.rollback_receipt,
                reference.rollback_receipt);
      EXPECT_EQ(observation.accepted_receipt,
                reference.accepted_receipt);
      EXPECT_EQ(observation.transaction_receipt,
                reference.transaction_receipt);
      EXPECT_EQ(observation.policy_summary,
                reference.policy_summary);
      EXPECT_EQ(observation.policy_outcomes,
                reference.policy_outcomes);
      EXPECT_EQ(observation.canonical_event_order,
                reference.canonical_event_order);
      EXPECT_EQ(observation.accepted_state,
                reference.accepted_state);
      EXPECT_EQ(observation.final_stamp,
                reference.final_stamp);
    }
  }
}

TEST(SelfContactTransactionCuda,
     ExactPassThroughUnresolvedReasonRollsBackAndRetriesExactly) {
  Fixture fixture(false, true);
  auto limits = c::SelfContactTransactionLimits{};
  limits.crossing.max_depth = 4;
  limits.crossing.max_work_per_pair = 31;
  limits.crossing.max_total_work = 31 * 64;
  ASSERT_TRUE(fixture.Initialize(limits));
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));

  std::size_t failed_pair = SIZE_MAX;
  for (unsigned retry = 0; retry < 2; ++retry) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    ASSERT_GT(accepted.diagnostics().active_count, 0u);

    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(
        token, assembly, prepared, common));
    c::SelfContactTransactionReceipt unchanged;
    const auto report = fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &unchanged);
    EXPECT_EQ(report.status,
              c::SelfContactTransactionStatus::UnresolvedCandidate);
    EXPECT_EQ(report.crossing_reason,
              c::RepresentedIntervalReason::WorkExhausted);
    EXPECT_FALSE(unchanged.valid());
    EXPECT_NE(report.pair, SIZE_MAX);
    if (!retry)
      failed_pair = report.pair;
    else
      EXPECT_EQ(report.pair, failed_pair);
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
    ASSERT_TRUE(fixture.rig.Read(after));
    p::Exact(before, after);
  }
}

}  // namespace self_contact_transaction_cuda_test
