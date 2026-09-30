// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_mapped/Fixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_src/elements/qeph/mapped/Stiffness.h"
#include "lib_src/elements/qeph/QephBatch.h"
#include "lib_src/elements/t3/T3Batch.h"
#include "lib_src/elements/t3/mapped/Stiffness.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/ShellBatchFailure.h"
#include "TypedValues.h"

namespace qt_mapped_test {
namespace fe=tl::fea;
struct SkinScope {
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidAssemblyBinding rigid;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::ShellPhysicalBinding physical;
};
struct Fixture : qbat_mapped_test::Fixture {
  std::optional<SkinScope> skin;
  const fe::ShellPhysicalBinding& Physical() const { return skin ? skin->physical : physical; }
  const fe::ShellBatchPlasticityBinding& Catalog() const { return skin ? skin->catalog : catalog; }
  const fe::NodalRigidAssemblyBinding& Rigid() const { return skin ? skin->rigid : rigid; }
};
inline fe::NodalReport InitializeOwner(Fixture& fixture,fe::FENodalState& owner) {
  const auto cin=fixture.mechanics.Cin();
  return owner.Initialize(fixture.mechanics.Config(),fixture.mechanics.Kinematics(),
      fixture.mechanics.im.data(),fixture.mechanics.Dofs(),fixture.Rigid(),&cin);
}
using qbat_binding_test::Bytes;
template<class T> std::vector<unsigned char> PayloadBytes(const std::vector<T>& values) {
  const auto* first=reinterpret_cast<const unsigned char*>(values.data());
  return {first,first+values.size()*sizeof(T)};
}
inline fe::ShellSectionLaw Law(const fe::ShellBatchPlasticityBinding& catalog,fe::ShellBindingFamily family,std::size_t row) {
  fe::ShellSectionLaw law=fe::ShellSectionLaw::Unspecified;
  EXPECT_TRUE(catalog.Law(family,row,&law));
  return law;
}
struct Quad {
  using Batch=fe::qeph::QephBatch;
  using Config=fe::qeph::QephBatchConfig;
  using Result=fe::qeph::ForceTrial;
  using Diagnostics=fe::qeph::BatchDiagnostics;
  static constexpr auto Family=fe::ShellBindingFamily::Qeph;
  static constexpr auto Success=fe::qeph::BatchStatus::Success;
  static constexpr unsigned Slots=4;
  static std::size_t Count(const Fixture& f) { return f.Physical().shells()->qeph_count(); }
  static const auto& Reference(const Fixture& f,std::size_t i) { return f.Physical().shells()->qeph_reference(i); }
  static bool Stiffness(const Fixture& f,std::size_t i,fe::shell_nodal_stiffness::Packet<4>& output) {
    return fe::qeph::mapped::InitialStiffness(Reference(f,i),Law(f.Catalog(),Family,i),output);
  }
};
struct Triangle {
  using Batch=fe::t3::T3Batch;
  using Config=fe::t3::T3BatchConfig;
  using Result=fe::t3::ForceTrial;
  using Diagnostics=fe::t3::BatchDiagnostics;
  static constexpr auto Family=fe::ShellBindingFamily::T3;
  static constexpr auto Success=fe::t3::BatchStatus::Success;
  static constexpr unsigned Slots=3;
  static std::size_t Count(const Fixture& f) { return f.Physical().shells()->t3_count(); }
  static const auto& Reference(const Fixture& f,std::size_t i) { return f.Physical().shells()->t3_reference(i); }
  static bool Stiffness(const Fixture& f,std::size_t i,fe::shell_nodal_stiffness::Packet<3>& output) {
    return fe::t3::mapped::InitialStiffness(Reference(f,i),Law(f.Catalog(),Family,i),output);
  }
};
template<class Family> typename Family::Config Config(const Fixture& f,const fe::NodalStamp& stamp) {
  typename Family::Config value;
  value.owner=stamp;
  value.configuration_id=81;
  value.qualification_id=82;
  value.element_count=Family::Count(f);
  value.usage=decltype(value.usage)::CoupledForces;
  return value;
}
// This private owner fixture supplies loads/stiffness only to the disjoint CIN
// patch, as in the QBAT qualifier. It does not stand in for another participant.
bool PrepareOwner(Fixture&,fe::FENodalState&,const fe::NodalTrialToken&,
                  fe::NodalAssemblyView&,fe::NodalPreparedView&);
void MakeSkinFixture(Fixture&,bool triangle=false);
template<class Family> struct Rig {
  Fixture fixture;
  fe::FENodalState owner;
  typename Family::Batch batch;
  typename Family::Config config;
  bool Initialize(bool skin=false) {
    if (skin) MakeSkinFixture(fixture,Family::Slots==3);
    const auto initialized=InitializeOwner(fixture,owner);
    EXPECT_EQ(initialized.status,fe::NodalStatus::Ok)<<initialized.message;
    if (initialized.status!=fe::NodalStatus::Ok) return false;
    config=Config<Family>(fixture,owner.accepted());
    const auto result=batch.InitializeMapped(config,fixture.Physical(),owner,fixture.Witnesses());
    EXPECT_EQ(result.status,Family::Success)<<result.message;
    return result.status==Family::Success;
  }
  bool Begin(fe::NodalTrialToken& token,fe::NodalAssemblyView& assembly) {
    const auto begun=owner.BeginTrial(&token,&assembly);
    EXPECT_EQ(begun.status,fe::NodalStatus::Ok)<<begun.message;
    if (begun.status!=fe::NodalStatus::Ok) return false;
    const auto report=batch.AssembleMappedAccepted(owner,token,assembly);
    EXPECT_EQ(report.status,Family::Success)<<report.message;
    return report.status==Family::Success;
  }
  bool Prepare(fe::NodalTrialToken& token,fe::NodalAssemblyView& assembly,fe::NodalPreparedView& view) {
    return Begin(token,assembly) && PrepareOwner(fixture,owner,token,assembly,view);
  }
  StateBits AcceptedState() {
    std::vector<fe::ShellBatchLayeredSection> sections(config.element_count);
    std::vector<fe::ShellBatchFailureState> failures(config.element_count);
    typename Family::Diagnostics diagnostics;
    EXPECT_EQ(batch.CopyAcceptedLayeredSectionHistory(owner.accepted(),sections.data(),sections.size(),&diagnostics).status,Family::Success);
    const bool one_point=std::any_of(sections.begin(),sections.end(),[](const auto& row) { return row.one_point()!=nullptr; });
    const auto old_failure=PayloadBytes(failures);
    const auto old_diagnostics=Bytes(diagnostics);
    const auto failure_report=batch.CopyAcceptedFailureHistory(owner.accepted(),failures.data(),failures.size(),&diagnostics);
    if (one_point) {
      // The true point owns its failure payload; no NIP3 history is available.
      EXPECT_NE(failure_report.status,Family::Success);
      EXPECT_EQ(PayloadBytes(failures),old_failure);
      EXPECT_EQ(Bytes(diagnostics),old_diagnostics);
    } else EXPECT_EQ(failure_report.status,Family::Success);
    StateBits values;
    for (std::size_t row=0;row<sections.size();++row) {
      Add(values,sections[row]);
      if (!one_point) Add(values,failures[row]);
    }
    return values;
  }
  std::vector<typename Family::Result> Accepted() {
    std::vector<typename Family::Result> output(config.element_count);
    typename Family::Diagnostics diagnostics;
    EXPECT_EQ(batch.CopyAcceptedResults(owner.accepted(),output.data(),output.size(),&diagnostics).status,Family::Success);
    return output;
  }
};
// Compare named scalar records only; no padding-byte serialization promise.
template<class Result> std::vector<double> Values(const std::vector<Result>& values) {
  std::vector<double> output;
  for (const auto& value:values) {
    const auto& h=value.proposed_history.data();
    output.insert(output.end(),{value.proposed_history.stamp().time,double(value.proposed_history.stamp().sample_index),h.active,h.thickness});
    for (double v:h.stress) output.push_back(v);
    for (double v:h.material_stress) output.push_back(v);
    for (double v:h.bending_stress) output.push_back(v);
    for (double v:h.strain_curvature) output.push_back(v);
    if constexpr (std::is_same_v<Result,fe::qeph::ForceTrial>) {
      for (double v:h.stabilization) output.push_back(v);
      output.push_back(h.hourglass_viscous_work);
    } else output.push_back(h.equivalent_strain_rate);
    for (double v:h.internal_work) output.push_back(v);
    for (const auto& f:value.internal_force) output.insert(output.end(),{f.x,f.y,f.z});
    for (const auto& f:value.internal_couple) output.insert(output.end(),{f.x,f.y,f.z});
  }
  return output;
}
} // namespace qt_mapped_test
