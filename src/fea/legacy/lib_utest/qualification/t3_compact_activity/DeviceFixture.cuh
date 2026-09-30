// SPDX-License-Identifier: MIT
#pragma once
#include "Flow.h"
#include <cuda_runtime.h>
#include <stdexcept>
#include <memory>
namespace t3_compact_test {
inline void Check(cudaError_t error){if(error!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(error));}
struct DeviceFixture {
  struct Delete { void operator()(void* p) const noexcept { cudaFree(p); } };
  std::vector<std::unique_ptr<void,Delete>> allocations;
  t::batch_detail::Storage* storage=nullptr;
  sp::MixedDeviceStorage* mixed=nullptr;sp::FailureDeviceStorage* failure=nullptr;sp::OnePointDeviceStorage* point=nullptr;
  m::ActivityQuery query;
  template<class T>T* Copy(const T* input,std::size_t count) {
    T* out=nullptr;Check(cudaMallocManaged(&out,count*sizeof(T)));allocations.emplace_back(out);
    for(std::size_t i=0;i<count;++i)new(out+i)T(input?input[i]:T{});
    return out;
  }
  explicit DeviceFixture(const Fixture& f) {
    storage=Copy<t::batch_detail::Storage>(nullptr,1);
    mixed=Copy<sp::MixedDeviceStorage>(nullptr,1);failure=Copy<sp::FailureDeviceStorage>(nullptr,1);
    point=f.has_point?Copy<sp::OnePointDeviceStorage>(nullptr,1):nullptr;
    storage->model.element=Copy(f.elements.data(),f.size());storage->model.config.element_count=f.size();
    storage->slab[0].element=Copy(f.forces.data(),f.size());storage->slab[1].element=storage->slab[0].element;
    storage->assembly.activity.first_invalid=Copy<std::uint32_t>(nullptr,1);
    storage->assembly.activity.active=Copy<std::uint8_t>(nullptr,f.size());
    mixed->law=Copy(f.laws.data(),f.size());mixed->plastic.parameters=Copy(f.parameters.data(),f.size());
    for(std::size_t i=0;i<f.size();++i)if(f.parameters[i].curve.count) {
      // Test-only relocation of the fixture's immutable material tables.
      auto& curve=mixed->plastic.parameters[i].curve;
      curve.plastic_strain=Copy(f.parameters[i].curve.plastic_strain,curve.count);
      curve.yield_stress_pa=Copy(f.parameters[i].curve.yield_stress_pa,curve.count);
    }
    mixed->plastic.section[0]=Copy(f.plastic.data(),f.size());mixed->plastic.section[1]=mixed->plastic.section[0];
    mixed->elastic_section[0]=Copy(f.elastic.data(),f.size());mixed->elastic_section[1]=mixed->elastic_section[0];
    failure->policy=Copy(f.policies.data(),f.size());failure->state[0]=Copy(f.failure.data(),f.size());failure->state[1]=failure->state[0];
    if(point){point->section[0]=Copy(f.points.data(),f.size());point->section[1]=point->section[0];}
    query={storage,mixed,failure,point,f.size(),0,f.time,f.force_time,f.epoch,f.force_epoch,f.execution};
  }
  ~DeviceFixture()=default;
  std::uint32_t Read(Phase phase,std::vector<std::uint8_t>& output) {
    *storage->assembly.activity.first_invalid=m::NoActivityFailure;
    std::fill_n(storage->assembly.activity.active,query.parents,19);
    Check(m::LaunchActivity(query,phase,nullptr));Check(cudaDeviceSynchronize());
    output.assign(storage->assembly.activity.active,storage->assembly.activity.active+query.parents);
    return *storage->assembly.activity.first_invalid;
  }
};
}
