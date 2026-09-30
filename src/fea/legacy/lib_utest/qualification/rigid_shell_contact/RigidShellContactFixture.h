#pragma once
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"
#include "lib_utest/qualification/t3/mixed_binding/ShellBatchBindingFixture.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/constraints/NodalRigidGroupModel.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
#include "lib_src/collision/NodalWallContactDevice.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include "lib_src/collision/T3ContactIntegration.h"

namespace rigid_shell_contact_test {
namespace fe=tl::fea;
namespace q=fe::qeph;
namespace t=fe::t3;
namespace sc=tlfea::contact;
namespace nt=tl_test::nodal_temporal;
using Cuda=nt::NodalTemporalCuda;
using shell_binding_test::Bytes;
constexpr unsigned Nodes=5;
constexpr double H=1./8192;
constexpr std::uint64_t Source=1001,Qualification=1002,Configuration=1003;
struct Fixture {
  fe::ShellBatchBinding binding;
  nt::Initial initial;
  fe::NodalRigidGroupModel groups;
  sc::NodalWallWeights weights;
  std::array<sc::PlanarWallVertex,4> wall_vertices{};
  std::array<sc::PlanarWallTriangle,2> wall_faces{};
  // The coverage API takes a projected Y/Z box at the exact wall X.
  // Physical X penetration is bounded separately by WallConfig::law.
  sc::PlanarWallBox motion{{0,-.5,-.5},{0,2.5,1.5}};
  bool Prepare(std::uint64_t source=Source,double speed=0);
  fe::NodalReport Owner(fe::FENodalState&) const;
  q::QephBatchConfig QConfig(fe::NodalStamp) const;
  t::T3BatchConfig TConfig(fe::NodalStamp) const;
  sc::NodalWallDeviceConfig WallConfig(fe::NodalStamp) const;
  sc::PlanarWallView Wall() const { return {wall_vertices.data(),4,wall_faces.data(),2}; }
};
struct Rig {
  Fixture source;
  fe::FENodalState owner;
  q::QephBatch qeph;
  t::T3Batch t3;
  sc::NodalWallContactDevice contact;
  fe::ShellBatchPublication publication;
  bool Initialize(std::uint64_t source_id=Source,double speed=0,bool bind=true);
  bool Bind();
  void Discard() { publication.DiscardTrial(); qeph.DiscardTrial(); t3.DiscardTrial(); contact.DiscardTrial(); owner.Discard(); }
};
struct Prepared {
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView view;
  sc::NodalWallDiagnostics wall_base,wall_candidate;
  fe::ShellBatchDiagnostics shells;
  q::ForceTrial quad;
  t::ForceTrial triangle;
};
bool Assemble(Rig&,Prepared&,bool load=true);
bool Advance(Rig&,Prepared&);
bool Shells(Rig&,Prepared&);
bool Prepare(Rig&,Prepared&,bool load=true);
bool Commit(Rig&,const Prepared&,bool valid=true);
nt::Snapshot ReadPrepared(const fe::NodalPreparedView&);
std::array<double,6*Nodes> ReadLoads(const fe::NodalAssemblyView&);
fe::NodalRigidGroupSnapshot ReadGroup(fe::FENodalState&);
void SameGroup(const fe::NodalRigidGroupSnapshot&,const fe::NodalRigidGroupSnapshot&);
void CheckForceHistory(Rig&,const Prepared&);
} // namespace rigid_shell_contact_test
