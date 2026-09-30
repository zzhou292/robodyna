// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include <cuda_runtime.h>
#include <cfloat>

namespace heph_test {
struct DeviceState {
  s::Reference reference;
  s::Material material;
  s::History history;
  s::History previous;
  s::ForceTrial trial;
  s::ForceStatus status=s::ForceStatus::InvalidInput;
};
static __global__ void InitializeDevice(const s::ReferenceInput* input,DeviceState* output,unsigned count) {
  const unsigned index=blockIdx.x*blockDim.x+threadIdx.x;
  if(index>=count)return;
  DeviceState state;
  if(s::InitializeReference(input[index],state.reference)!=s::Status::Success) {
    output[index]=state;
    return;
  }
  if(tl::material::law42::Prepare(24e6,.463,input[index].density_kg_m3,1e26,state.material)!=
      tl::material::law42::Status::Ok) {
    output[index]=state;
    return;
  }
  state.status=s::InitializeHistory(state.reference,state.material,state.history);
  output[index]=state;
}
static __global__ void AdvanceDevice(DeviceState* states,const s::PrescribedInterval* input,unsigned count) {
  const unsigned index=blockIdx.x*blockDim.x+threadIdx.x;
  if(index>=count)return;
  auto& state=states[index];
  auto interval=input[index];
  interval.base_time_s=state.history.stamp().time_s;
  state.status=s::EvaluateForce(state.reference,state.history,interval,state.material,state.trial);
  if(state.status==s::ForceStatus::Success) {
    state.previous=state.history;
    state.history=state.trial.proposed_history;
  }
}
struct DeviceRejections {
  s::ForceTrial overflow,identity,cutoff,retry;
  s::ForceStatus overflow_status,identity_status,cutoff_status,retry_status;
};
static __global__ void RejectAndRetry(DeviceState* state,s::PrescribedInterval good,DeviceRejections* out) {
  good.base_time_s=state->previous.stamp().time_s;
  auto bad=good;
  bad.velocity_m_s[7].z=DBL_MAX;
  out->overflow_status=s::EvaluateForce(state->reference,state->previous,bad,state->material,state->trial);
  out->overflow=state->trial;
  auto changed_input=state->reference.input();
  ++changed_input.source_material_id;
  s::Reference changed;
  s::InitializeReference(changed_input,changed);
  out->identity_status=s::EvaluateForce(changed,state->previous,good,state->material,state->trial);
  out->identity=state->trial;
  auto cutoff=state->material;
  cutoff.tension_cutoff_pa=1;
  s::History cutoff_history;
  s::InitializeHistory(state->reference,cutoff,cutoff_history);
  out->cutoff_status=s::EvaluateForce(state->reference,cutoff_history,good,cutoff,state->trial);
  out->cutoff=state->trial;
  out->retry_status=s::EvaluateForce(state->reference,state->previous,good,state->material,state->trial);
  out->retry=state->trial;
}
// Test-only owning allocation. Production receives no device allocation helper.
template<class T> struct DeviceArray {
  T* data=nullptr;
  explicit DeviceArray(std::size_t count=1) {
    if(cudaMalloc(&data,count*sizeof(T))!=cudaSuccess)throw std::runtime_error("CUDA test allocation failed");
  }
  ~DeviceArray(){cudaFree(data);}
  DeviceArray(const DeviceArray&)=delete;
  DeviceArray& operator=(const DeviceArray&)=delete;
};
} // namespace heph_test
