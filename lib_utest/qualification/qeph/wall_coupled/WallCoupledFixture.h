#pragma once
// Qualification composition only. Frozen operands and scopes: README.md.
#include "../coupled/QephCoupledLedger.h"
#include "lib_src/collision/NodalWallContactDevice.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include "lib_utest/q4_planar_geometry_fixture.h"

namespace qeph_wall_test {
using namespace qeph_coupled_test;
namespace sc=tlfea::contact;
constexpr std::uint64_t WallQualification=0x4357305052454631ULL;
constexpr std::uint64_t ShellConfiguration=0x4357305348454c31ULL;
constexpr std::uint64_t WallConfiguration=0x43573057414c4c31ULL;
constexpr std::uint64_t WallBinding=0x43573046494e4931ULL;
constexpr double Slope=1./128,Preload=0x1p-12,Kappa=4e5,DepthCap=.0005;
constexpr double ForceError=5e-7,EnergyError=1.2500000000000005e-12;
struct WallRig {
  Rig shell;
  sc::Q4ParametricReference reference;
  sc::NodalWallWeights weights;
  std::array<sc::SurfaceQ4,2> parents{};
  q4_planar_test::Wall mesh=q4_planar_test::Square();
  sc::PlanarWallBox motion{{0,-.05,-.05},{0,.05,.05}};
  sc::NodalWallContactDevice wall;
  explicit WallRig(unsigned cells):shell(cells) {}
  bool Initialize(double h);
  sc::NodalWallConfig Law() const { return {0,Kappa,DepthCap,ForceError,EnergyError}; }
  sc::NodalWallResult Host(const Snapshot&,std::uint64_t epoch,std::uint64_t attempt) const;
  void Discard() { shell.owner.Discard(); shell.batch.DiscardTrial(); wall.DiscardTrial(); }
};
struct Trial {
  Prepared nodal;
  sc::NodalWallDiagnostics contact_base;
  sc::NodalWallDeviceResults contact_base_result;
};
struct Staged {
  q::BatchDiagnostics shell;
  PortResults elements{};
  sc::NodalWallDiagnostics contact;
  sc::NodalWallDeviceResults contact_result;
};
Loads ContactLoads(const sc::NodalWallResult&);
bool Begin(WallRig&,Trial&);
bool Evaluate(WallRig&,const Trial&,Staged&,bool contact_first=false);
void Identity(const WallRig&,const Trial&,const Staged&);
void Agreement(const WallRig&,const Trial&,const Staged&,const NativeProposal&,double,std::uint64_t);
bool Publish(WallRig&,const Trial&,const Staged&,sc::NodalWallDeviceResults& published);
void ContactAgreement(const sc::NodalWallDeviceResults&,const sc::NodalWallResult&);
void Property(const std::string&,double);
struct ContactEvidence {
  double work_ratio=0,impulse_ratio=0,defect_ratio=0;
  double omitted_contact=0,omitted_shell=0,early_endpoint=0;
  unsigned omitted_contact_epoch=0,omitted_shell_epoch=0;
};
void ContactLedgers(const WallRig&,const Snapshot&,const Trial&,const Staged&,ContactEvidence&);
void Controls(const WallRig&,const Snapshot&,const Trial&,const Staged&,const Loads&,ContactEvidence&);
void Prefix(unsigned cells);
} // namespace qeph_wall_test
