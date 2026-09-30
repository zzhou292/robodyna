#pragma once
#include "../vehicle_wall_device/VehicleWallFixture.h"
#include "lib_src/collision/nodal_wall_mapped/Evaluation.cuh"
#include <cstring>
namespace wall_evaluation_test {
namespace c=tlfea::contact;
namespace d=c::nodal_wall_device_detail;
namespace m=c::nodal_wall_mapped;
namespace v=vehicle_wall_device_test;
constexpr unsigned Nodes=137,Quads=129,Triangles=65,Parents=Quads+Triangles;
struct Inputs {
  double x[3*Nodes]{},velocity[3*Nodes]{};
  std::uint8_t activity[Parents]{};
  m::Summary summary;
};
struct Fixture {
  v::Fixture source{Quads,Triangles,Nodes};
  d::PreparedModel model;
  d::Storage* storage=nullptr;
  Inputs* input=nullptr;
  Fixture() {
    EXPECT_TRUE(source.Prepare());
    EXPECT_EQ(d::PreparePhysicalModel(source.Config(),source.wall.view(),source.weights,source.Positions(),
        source.motion,&model).status,c::NodalWallDeviceStatus::Ok);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&storage),model.layout().bytes),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&input),sizeof(Inputs)),cudaSuccess);
    Restore();
  }
  ~Fixture() {cudaFree(input);cudaFree(storage);}
  void Restore() {
    std::memcpy(storage,model.data(),model.layout().bytes);
    *storage=model.Rebase(storage);
    *input={};
    std::copy(source.x.begin(),source.x.end(),input->x);
    std::copy(source.v.begin(),source.v.end(),input->velocity);
    std::fill(std::begin(input->activity),std::end(input->activity),1);
    input->summary.points_admitted=true;
  }
  c::NodalWallDiagnostics Identity() const {
    c::NodalWallDiagnostics id;
    id.owner_id=77;id.base_epoch=2;id.attempt=5;id.configuration_id=991;id.wall_binding_id=771;
    id.time=id.base_time=2*v::H;id.velocity_time=id.base_velocity_time=1.5*v::H;
    id.phase=c::NodalWallDevicePhase::AcceptedBase;
    return id;
  }
  tl::fea::DeviceNodalKinematicsView Kinematics() const {
    tl::fea::DeviceNodalKinematicsView k;
    k.position_xyz=input->x;k.velocity_xyz=input->velocity;k.node_count=Nodes;k.base_epoch=2;
    return k;
  }
  m::Sidecar Side() const {m::Sidecar s;s.accepted=input->activity;s.summary=&input->summary;return s;}
  v::Results Read(const d::ActiveResults& active) const {
    v::Results result(source);
    result.diagnostics=active.diagnostics;
    std::copy_n(active.parents,Parents,result.parents.begin());
    std::copy_n(active.nodes,Nodes,result.nodes.begin());
    std::copy_n(active.wall_face,Nodes,result.faces.begin());
    return result;
  }
  v::Results Read() const {return Read(storage->result);}
};
} // namespace wall_evaluation_test
