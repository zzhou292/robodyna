// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Support.h"
#include <cuda_runtime.h>
namespace beam18_read_tile_test {
struct Device {
  d::Storage* state=nullptr;b::Parent* parents=nullptr;b::Material* material=nullptr;b::ForceTrial* old=nullptr; b::ForceTrial* now=nullptr;int* status=nullptr;
  double* curve=nullptr;double* fields=nullptr;std::size_t count=0,capacity=0;fe::NodalPreparedView view;
  template<class T>bool Allocate(T*& p,std::size_t n){return cudaMallocManaged(&p,n*sizeof(T))==cudaSuccess;}
  bool Initialize(std::size_t n) {
    count=n;capacity=std::max<std::size_t>(n,1);
    if(!Allocate(state,1)||!Allocate(parents,capacity)||!Allocate(material,1)||!Allocate(old,capacity)||!Allocate(now,capacity)||!Allocate(status,capacity)||!Allocate(curve,2*law44_solid_test::Count)||!Allocate(fields,26*2*capacity))return false;
    *state={};state->count=count;state->parents=parents;state->materials=material;state->slab[0]=old;state->slab[1]=now;state->status=status;
    state->source_instance_id=101;state->config.owner.owner_id=103;state->config.configuration_id=107;state->config.qualification_id=109;state->config.owner.fixed_dt=1.;
    for(std::size_t p=0;p<law44_solid_test::Count;++p){curve[p]=law44_solid_test::X[p];curve[law44_solid_test::Count+p]=law44_solid_test::Y[p];}
    const auto ref=beam18_force_test::Reference();*material=beam18_force_test::Material(ref);material->curve.plastic_strain=curve;material->curve.yield_stress_pa=curve+law44_solid_test::Count;
    const auto nodes=2*capacity;view={};view.kick_dt=1.;view.base_kinematics.node_count=view.kinematics.node_count=nodes;
    view.base_kinematics.position_xyz=fields;view.kinematics.position_xyz=fields+3*nodes;
    view.base_kinematics.velocity_xyz=fields+6*nodes;view.kinematics.velocity_xyz=fields+9*nodes;
    view.base_kinematics.angular_velocity_xyz=fields+12*nodes;view.kinematics.angular_velocity_xyz=fields+15*nodes;
    view.base_kinematics.orientation_wxyz=fields+18*nodes;view.kinematics.orientation_wxyz=fields+22*nodes;
    return Reset();
  }
  bool Reset(){
    std::fill(fields,fields+26*2*capacity,0.);const auto ref=beam18_force_test::Reference();b::ForceTrial base;
    if(b::InitializeForce(ref,*material,{},base)!=b::Status::Success)return false;
    const double terms[]{0x1p54,1,-0x1p54,-0.,0x1p-1022,-0x1p-1022,.25,-.25};
    for(std::size_t p=0;p<capacity;++p){parents[p]={};parents[p].reference=ref;parents[p].material_index=0;parents[p].domain_nodes[0]=2*p;parents[p].domain_nodes[1]=2*p+1;status[p]=0;old[p]=now[p]=base;
      now[p].diagnostics.internal_work_increment_j[0]=terms[p%8];now[p].diagnostics.internal_work_increment_j[1]=terms[(p+1)%8];now[p].diagnostics.plastic_work_increment_j=0.;
      for(unsigned n=0;n<2;++n){const auto node=2*p+n;const_cast<double*>(view.base_kinematics.position_xyz)[3*node]=n*.1;const_cast<double*>(view.kinematics.position_xyz)[3*node]=n*.1+1.;
        const_cast<double*>(view.base_kinematics.velocity_xyz)[3*node]=const_cast<double*>(view.kinematics.velocity_xyz)[3*node]=1.;
        const_cast<double*>(view.base_kinematics.orientation_wxyz)[4*node]=const_cast<double*>(view.kinematics.orientation_wxyz)[4*node]=1.;}}
    if(!d::ValidResult(parents[0],material[parents[0].material_index],now[0],0,0)){ADD_FAILURE()<<"Reset result is invalid";return false;}
    double kick=0,drift=0;
    if(!fe::beam_endpoint::AccumulateRhsWork(parents[0].domain_nodes,old[0].rhs_force_n,old[0].rhs_couple_nm,view,state->config.owner.fixed_dt,kick,drift)){ADD_FAILURE()<<"Reset endpoint view is invalid";return false;}
    return true;
  }
  ~Device(){cudaFree(fields);cudaFree(curve);cudaFree(status);cudaFree(now);cudaFree(old);cudaFree(material);cudaFree(parents);cudaFree(state);}
};
}
