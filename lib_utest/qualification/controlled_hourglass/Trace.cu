// SPDX-License-Identifier: AGPL-3.0-or-later
// Diagnostic only: emit all independently carried native/GPU packets; no tolerance.
#include "TestSupport.h"
#include <cuda_runtime.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace controlled_test {
struct TraceState { Case value; c::Result result; c::Status status; };
__global__ void TraceEvaluate(TraceState* rows) {
  const unsigned n=threadIdx.x;if(n>=8)return;auto& row=rows[n];
  row.status=c::EvaluateLaw42(row.value.input,row.value.state,row.result);
  if(row.status==c::Status::Success){row.value.state=row.result.proposed_state;
    row.value.input.internal_energy_density_j_m3=row.result.internal_energy_density_j_m3;}
}
void Cuda(cudaError_t status) {if(status!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(status));}
template<class Sequence> void Array(const Sequence& values) {
  std::cout<<'[';bool first=true;for(double v:values){if(!first)std::cout<<',';first=false;std::cout<<v;}std::cout<<']';
}
void Input(const Case& value) {
  const auto& p=value.input;std::vector<double> values{p.mu_pa,p.poisson_ratio,p.density_kg_m3,
    p.material_sound_speed_m_s,p.dt_s,p.current_volume_m3,p.reference_volume_m3,
    p.internal_energy_density_j_m3,p.raw_stiffness_n_m};
  for(const auto& v:p.local_velocity_m_s)for(unsigned k=0;k<3;++k)values.push_back(b::Component(v,k));
  for(const auto& v:p.incoming_local_force_n)for(unsigned k=0;k<3;++k)values.push_back(b::Component(v,k));
  for(const auto& row:p.projection)for(double v:row)values.push_back(v);
  for(const auto& row:value.state.force_n)for(double v:row)values.push_back(v);
  Array(values);
}
} // namespace controlled_test
int main() {
  using namespace controlled_test;TraceState* device=nullptr;
  try {
    Cuda(cudaSetDevice(0));Cuda(cudaMalloc(reinterpret_cast<void**>(&device),8*sizeof(TraceState)));
    std::array<TraceState,8> rows{};std::array<Case,8> native{};
    for(unsigned n=0;n<8;++n){rows[n].value=Moving();rows[n].value.input.material_sound_speed_m_s=250+120*n;native[n]=rows[n].value;}
    std::cout<<std::setprecision(std::numeric_limits<double>::max_digits10);
    for(unsigned step=0;step<32;++step) {
      if(step)for(unsigned n=0;n<8;++n)for(unsigned i=0;i<8;++i) {
        rows[n].value.input.local_velocity_m_s[i].z=(step%2?-1.:1.)*(i%2?.7:-.7);
        native[n].input.local_velocity_m_s[i].z=rows[n].value.input.local_velocity_m_s[i].z;
      }
      const auto before=rows;
      Cuda(cudaMemcpy(device,rows.data(),sizeof(rows),cudaMemcpyHostToDevice));
      TraceEvaluate<<<1,8>>>(device);Cuda(cudaGetLastError());
      Cuda(cudaMemcpy(rows.data(),device,sizeof(rows),cudaMemcpyDeviceToHost));
      for(unsigned n=0;n<8;++n) {
        if(rows[n].status!=c::Status::Success)throw std::runtime_error("GPU leaf rejected trace packet");
        const auto expected=Native(native[n]);
        std::cout<<"{\"step\":"<<step<<",\"packet\":"<<n<<",\"gpu_input\":";Input(before[n].value);
        std::cout<<",\"native_input\":";Input(native[n]);std::cout<<",\"gpu_output\":";Array(Values(rows[n].result));
        std::cout<<",\"native_output\":";Array(expected);std::cout<<"}\n";
        AcceptNative(native[n],expected);
      }
    }
    Cuda(cudaFree(device));return 0;
  } catch(const std::exception& error) {if(device)cudaFree(device);std::cerr<<error.what()<<'\n';return 1;}
}
