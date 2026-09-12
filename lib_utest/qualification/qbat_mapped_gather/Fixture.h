// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_force/Fixture.h"
#include "lib_src/elements/qbat/QbatBatchStorage.h"
#include "lib_src/elements/qbat/QbatBatchAdvance.h"
#include "lib_src/elements/qbat/mapped/AssemblyValues.h"
#include "lib_src/elements/qbat/mapped/Measurement.h"
#include "lib_src/elements/mapped_shell/Incidence.h"
#include <array>
#include <cstring>
namespace qbat_gather_test {
namespace fe=tl::fea;
namespace q=fe::qbat;
namespace b=q::batch_detail;
namespace m=q::mapped;
constexpr std::size_t Parents=4,Nodes=139;
constexpr double Dt=qbat_force_test::Dt;
inline std::uint64_t Bits(double x) { std::uint64_t out;std::memcpy(&out,&x,8);return out; }
struct Inputs {
  double position[3*Nodes]{},endpoint[3*Nodes]{},velocity[3*Nodes]{},omega[3*Nodes]{},orientation[4*Nodes]{};
  double inverse[Nodes]{},values[8][Nodes]{};
  std::uint8_t fixed[Nodes]{};
  fe::NodalAssemblyResult result;
  fe::stability::RowBounds bounds;
  void Reset(unsigned epoch) {
    result={};result.base_epoch=epoch;result.attempt=3;
    bounds={};bounds.base_epoch=epoch;bounds.attempt=3;bounds.initialized=bounds.valid=true;
    for(unsigned node=0;node<Nodes;++node) {
      inverse[node]=1;orientation[4*node]=1;
      for(unsigned channel=0;channel<8;++channel) values[channel][node]=channel<6?-0.:.125*(1+node+channel);
      for(unsigned axis=0;axis<3;++axis) endpoint[3*node+axis]=position[3*node+axis]+.0001*(node%7)*(axis+1);
    }
    values[0][0]=0x1p54;values[1][2]=-.7;values[3][1]=3.25;
  }
  fe::NodalAssemblyView View(unsigned epoch) {
    fe::NodalAssemblyView view;
    view.accepted={position,velocity,omega,Nodes,epoch,orientation};
    view.mass.inverse_mass=inverse;view.mass.fixed=fixed;view.mass.node_count=Nodes;
    view.inverse_inertia=inverse;view.translation_fixed_bits=fixed;view.rotation_fixed=fixed;
    view.forces={values[0],values[1],values[2],values[3],values[4],values[5],Nodes,epoch};
    view.result=&result;view.bounds=&bounds;view.attempt=3;view.position_time=epoch*Dt;
    return view;
  }
  fe::NodalCinAssemblyView Cin() {
    fe::NodalCinAssemblyView out;out.translational_stiffness=values[6];out.rotational_stiffness=values[7];out.node_count=Nodes;return out;
  }
  fe::NodalPreparedView Prepared(unsigned epoch) {
    fe::NodalPreparedView out;
    out.base_kinematics={position,velocity,omega,Nodes,epoch,orientation};
    out.kinematics={endpoint,velocity,omega,Nodes,epoch,orientation};
    out.base_time=epoch*Dt;out.proposed_time=(epoch+1)*Dt;out.attempt=3;out.kick_dt=epoch?Dt:.5*Dt;
    return out;
  }
};
struct Fixture {
  b::Layout layout;
  tl::util::HostArena arena;
  b::Storage* host=nullptr;
  Inputs input;
  Fixture() {
    EXPECT_TRUE(layout.InitializeMapped(Parents,Nodes,0,1u<<20));
    EXPECT_TRUE(arena.Initialize(layout.bytes));host=layout.Construct(arena);EXPECT_NE(host,nullptr);
    if(!host) return;
    auto& model=host->model;model.mapped=model.joined=true;
    model.config.element_count=Parents;model.config.owner.node_count=Nodes;
    model.config.owner.fixed_dt=Dt;model.config.usage=q::BatchUsage::CoupledForces;
    const q::Vec3 positions[6]{{0,0,0},{.04,0,0},{.04,.02,0},{0,.02,0},{.08,0,0},{.08,.02,0}};
    for(unsigned node=0;node<Nodes;++node) {
      const auto x=node<6?positions[node]:q::Vec3{};
      model.initial_position[node]=x;
      input.position[3*node]=x.x;input.position[3*node+1]=x.y;input.position[3*node+2]=x.z;
    }
    const std::size_t nodes[Parents][4]{{2,3,0,1},{0,1,2,3},{1,4,5,2},{0,1,2,3}};
    qbat_force_test::Fixture original(false);
    for(unsigned parent=0;parent<Parents;++parent) {
      auto& element=model.element[parent];auto reference=original.input;
      for(unsigned slot=0;slot<4;++slot) {
        element.nodes[slot]=nodes[parent][slot];reference.quadrilateral.position[slot]=positions[element.nodes[slot]];
      }
      EXPECT_EQ(q::InitializeReference(reference,element.reference),q::Status::kSuccess);
      element.material=original.material;element.failure=original.failure;element.source_parent_id=200+parent;
    }
    EXPECT_TRUE(fe::mapped_shell::BuildIncidence<4>(model.element,Parents,Nodes,
        host->assembly.offsets,Nodes+1,host->assembly.incidence,4*Parents));
    Prepare(0);
  }
  void Prepare(unsigned epoch,bool prescribed_loads=true) {
    input.Reset(epoch);
    for(unsigned parent=0;parent<Parents;++parent) {
      const auto& element=host->model.element[parent];auto& accepted=host->slab[0].element[parent];
      EXPECT_EQ(b::InitializeResult(element,accepted),q::Status::kSuccess);
      qbat_force_test::Fixture path(false);path.input=element.reference.input();path.reference=element.reference;
      for(unsigned step=0;step<epoch;++step) {
        q::BatchResult next;
        EXPECT_EQ(b::Advance(element,accepted,qbat_force_test::Path(path,step),next),q::Status::kSuccess);
        accepted=next;
      }
      EXPECT_EQ(b::Advance(element,accepted,qbat_force_test::Path(path,epoch),host->slab[1].element[parent]),q::Status::kSuccess);
      host->candidate_status[parent]=q::Status::kSuccess;
      if(epoch && prescribed_loads) for(unsigned slot=0;slot<4;++slot) {
        const double force=parent==0?0x1p54:parent==1?-1:-0x1p54;
        accepted.internal_force_n[slot]={force,-0.,.125*(1+slot)};
        accepted.internal_couple_nm[slot]={-0.,0.,-0.};
      }
      EXPECT_TRUE(b::ValidResult(accepted,element.material,epoch*Dt,epoch));
    }
  }
};
inline q::BatchDiagnostics Identity(unsigned epoch) {
  q::BatchDiagnostics out;out.time=(epoch+1)*Dt;out.epoch=epoch+1;out.base_epoch=epoch;
  out.attempt=3;out.phase=q::BatchPhase::Prepared;out.has_completed_interval=true;return out;
}
inline void Remove(q::BatchResult& result) {
  result.history.element_active=false;
  for(unsigned slot=0;slot<4;++slot) {
    auto& point=result.history.point[slot];point.surface_active=false;point.failure.point_active=false;
    point.failure.damage=1;point.failure.failure_time_s=result.stamp.time;
    for(auto& stress:point.material.stress) stress=0;
    result.internal_force_n[slot]={-0.,0.,-0.};
    result.internal_couple_nm[slot]={0.,-0.,0.};
  }
  result.diagnostics.translation_stiffness_n_m=0;
}
inline void Compare(const Inputs& a,const Inputs& b) {
  for(unsigned channel=0;channel<8;++channel) for(unsigned node=0;node<Nodes;++node)
    EXPECT_EQ(Bits(a.values[channel][node]),Bits(b.values[channel][node]))<<channel<<":"<<node;
}
} // namespace qbat_gather_test
