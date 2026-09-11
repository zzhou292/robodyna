// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/qeph/QephForce.h"
#include "lib_src/elements/qeph/mapped/Incidence.h"
#include "lib_src/elements/qeph/mapped/AssemblyValues.h"
#include "lib_src/elements/ShellMixedSectionArenaLayout.h"
#include "lib_src/solvers/NodalCinRuntime.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>

namespace qeph_gather_test {
namespace fe=tl::fea;
namespace q=fe::qeph;
namespace b=q::batch_detail;
namespace m=q::mapped;
constexpr std::size_t Parents=4,Nodes=6;
inline std::uint64_t Bits(double value) { std::uint64_t bits;std::memcpy(&bits,&value,8);return bits; }
struct Inputs {
  double position[3*Nodes]{},velocity[3*Nodes]{},omega[3*Nodes]{},orientation[4*Nodes]{};
  double inverse[Nodes]{},values[8][Nodes]{};
  std::uint8_t fixed[Nodes]{};
  fe::NodalAssemblyResult result;
  fe::stability::RowBounds bounds;
  fe::ShellSectionLaw law[Parents]{};
  fe::shell_batch_plasticity_detail::MixedDeviceStorage mixed;
  fe::NodalAssemblyView View(std::uint64_t epoch) {
    fe::NodalAssemblyView view;
    view.accepted={position,velocity,omega,Nodes,epoch,orientation};
    view.mass.inverse_mass=inverse;view.mass.fixed=fixed;view.mass.node_count=Nodes;
    view.inverse_inertia=inverse;view.translation_fixed_bits=fixed;view.rotation_fixed=fixed;
    view.forces={values[0],values[1],values[2],values[3],values[4],values[5],Nodes,epoch};
    view.bounds=&bounds;view.result=&result;view.attempt=3;view.position_time=epoch*1e-6;
    return view;
  }
  fe::NodalCinAssemblyView Cin() {
    fe::NodalCinAssemblyView cin;cin.translational_stiffness=values[6];
    cin.rotational_stiffness=values[7];cin.node_count=Nodes;return cin;
  }
  void Reset(std::uint64_t epoch) {
    result={};result.base_epoch=epoch;result.attempt=3;
    bounds={};bounds.base_epoch=epoch;bounds.attempt=3;bounds.initialized=bounds.valid=true;
    mixed.law=law;
    for (unsigned n=0;n<Nodes;++n) {
      inverse[n]=1;orientation[4*n]=1;
      for (unsigned channel=0;channel<8;++channel) values[channel][n]=channel<6?-0.:.125*(1+n+channel);
    }
    // A nonzero prior producer's force must participate before every QEPH term.
    values[0][0]=0x1p54;values[1][2]=-.7;values[3][1]=3.25;
  }
};
struct Fixture {
  b::Layout layout;
  tl::util::HostArena arena;
  b::Storage* host=nullptr;
  Inputs input;
  Fixture() {
    EXPECT_TRUE(layout.InitializeMapped(Parents,Nodes,1u<<20));
    EXPECT_TRUE(arena.Initialize(layout.bytes));host=layout.Construct(arena);EXPECT_NE(host,nullptr);
    if (!host) return;
    auto& model=host->model;model.config.element_count=Parents;model.config.owner.node_count=Nodes;
    model.joined=model.mapped=true;
    const q::Vec3 positions[]{{0,0,0},{.04,0,0},{.04,.02,0},{0,.02,0},{.06,0,0},{.06,.02,0}};
    const std::size_t connectivity[Parents][4]{{2,3,0,1},{0,1,2,3},{1,4,5,2},{3,0,1,2}};
    for (unsigned n=0;n<Nodes;++n) {
      model.initial_position[n]=positions[n];
      input.position[3*n]=positions[n].x;input.position[3*n+1]=positions[n].y;
    }
    for (unsigned parent=0;parent<Parents;++parent) {
      auto& element=model.element[parent];q::ReferenceInput reference;
      for (unsigned slot=0;slot<4;++slot) {
        element.nodes[slot]=connectivity[parent][slot];
        reference.position[slot]=positions[element.nodes[slot]];
        reference.node_ids[slot]=10+element.nodes[slot];
      }
      EXPECT_EQ(q::InitializeReference(reference,element.reference),q::Status::kSuccess);
      input.law[parent]=parent==3?fe::ShellSectionLaw::RigidSkin:fe::ShellSectionLaw::LayeredLaw44Nip3;
    }
    EXPECT_TRUE(m::BuildIncidence(model.element,Parents,Nodes,host->assembly.offsets,Nodes+1,
        host->assembly.incidence,4*Parents));
    Prepare(0);
  }
  void Prepare(unsigned epoch) {
    input.Reset(epoch);
    for (unsigned parent=0;parent<Parents;++parent) {
      auto& result=host->slab[0].element[parent];result={};
      const auto& reference=host->model.element[parent].reference;
      EXPECT_EQ(q::InitializeHistory(reference,{epoch*1e-6,epoch},result.proposed_history),q::Status::kSuccess);
      if (epoch && input.law[parent]!=fe::ShellSectionLaw::RigidSkin) {
        // A valid retained packet with deliberate cancelling nodal loads. The
        // gather test does not claim these prescribed loads are a force solver.
        result.kinematics.area=reference.area;result.kinematics.sample_index=epoch;
        result.kinematics.base_time=(epoch-1)*1e-6;result.kinematics.dt=1e-6;
        result.kinematics.nodal_factors[0]=.7;result.kinematics.nodal_factors[1]=1;
        result.diagnostics.translational_stiffness=37;result.diagnostics.rotational_stiffness=13;
        for (unsigned slot=0;slot<4;++slot) {
          const double force=parent==0?0x1p54:parent==1?-1:-0x1p54;
          result.internal_force[slot]={force,-0.,.125*(1+slot)};
          result.internal_couple[slot]={-force,.03125*(slot+1),0.};
        }
      }
      EXPECT_TRUE(m::ValidResult(reference,result,epoch*1e-6,epoch,input.law[parent]==fe::ShellSectionLaw::RigidSkin));
    }
  }
};
inline void Compare(const Inputs& actual,const Inputs& expected) {
  for (unsigned channel=0;channel<8;++channel) for (unsigned n=0;n<Nodes;++n) {
    EXPECT_EQ(Bits(actual.values[channel][n]),Bits(expected.values[channel][n]))<<channel<<":"<<n;
  }
}
} // namespace qeph_gather_test
