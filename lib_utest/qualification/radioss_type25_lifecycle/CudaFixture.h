// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Assertions.h"
#include <cuda_runtime.h>
#include <cstring>
#include <memory>
#include <stdexcept>
namespace type25_lifecycle_test::device {
constexpr unsigned Rows=4,Nodes=32,Mains=8,Normals=32,Raw=128,Entries=256,Slots=128,Sliding=256;
// Qualification-owned bounded source copies only. Production runtime borrows
// its one physical owner's accepted X/V and authenticates source separately.
struct Image {
  l::Input input;
  l::Node nodes[Nodes];double positions[3*Nodes],velocities[3*Nodes];
  l::Main mains[Mains];l::Secondary secondary[Rows];l::NormalReference normals[Normals];
  n::StoredNormal current_face_normals[4*Mains];l::NormalReference current_references[Normals];
  n::NativeGeometryHistory accepted[Rows];l::SpatialOccurrence spatial[Raw];
  std::uint32_t no[Normals+1],ne[Entries],ro[Rows+1],re[Entries],so[Rows+1],se[Raw];
};
struct Work {
  l::Report report;
  l::PreparedRow prepared[Rows];
  l::OptimizedRow optimized[Rows];
  l::Occurrence occurrences[Rows*Slots];n::NativeRawGeometryResult geometry[Rows*Slots];
  int sliding[Rows*Sliding];
};
__device__ inline l::Input Bind(const Image& x) {
  auto in=x.input;in.source.nodes=x.nodes;in.source.mains=x.mains;
  in.source.secondary=x.secondary;in.source.normals=in.source.normals?x.normals:nullptr;
  if(in.current_normals.face_normals)in.current_normals.face_normals=x.current_face_normals;
  if(in.current_normals.references)in.current_normals.references=x.current_references;
  in.source.normal_to_main.offsets=x.no;in.source.normal_to_main.entries=x.ne;
  in.source.removed_main_by_secondary.offsets=x.ro;in.source.removed_main_by_secondary.entries=x.re;
  in.current.positions.data=x.positions;in.current.velocities.data=x.velocities;
  in.accepted_rows=x.accepted;in.spatial=x.spatial;
  in.spatial_by_secondary.offsets=x.so;in.spatial_by_secondary.entries=x.se;return in;
}
__device__ inline l::RowScratch Scratch(Work& out,unsigned row) {
  return {out.occurrences+row*Slots,out.geometry+row*Slots,Slots,
          out.sliding+row*Sliding,Sliding,row*Slots};
}
__global__ void AdmitSource(const Image* image,Work* out) {
  if(blockIdx.x||threadIdx.x)return;
  out->report={};out->report.status=l::detail::Validate(Bind(*image));
}
__global__ void PrepareRows(const Image* image,Work* out,bool reverse) {
  if(out->report.status!=n::selection::Status::Ok)return;
  const auto in=Bind(*image);n::units_detail::Factors units;l::detail::Factors(in.current,units);
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<in.source.secondary_count;i+=blockDim.x*gridDim.x) {
    const unsigned row=reverse?unsigned(in.source.secondary_count)-1-i:i;
    const auto needs=l::detail::Requirements(in,row);
    if(needs.status!=n::selection::Status::Ok||needs.sliding>Sliding) {
      out->prepared[row]={};out->prepared[row].stage.report.status=n::selection::Status::CapacityExceeded;
    } else out->prepared[row]=l::detail::PrepareRow(in,row,Scratch(*out,row),units);
  }
}
// Separate launches prove retained preparation can resume after a stream
// barrier; the normal producer itself is deliberately not simulated here.
__global__ void BeforeNormalRows(const Image* image,Work* out,bool reverse) {
  if(out->report.status!=n::selection::Status::Ok)return;
  const auto in=Bind(*image);n::units_detail::Factors units;l::detail::Factors(in.current,units);
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<in.source.secondary_count;i+=blockDim.x*gridDim.x) {
    const unsigned row=reverse?unsigned(in.source.secondary_count)-1-i:i;
    out->optimized[row]=l::detail::PrepareRowBeforeNormals(in,row,units);
  }
}
__global__ void AfterNormalRows(const Image* image,Work* out,bool reverse) {
  if(out->report.status!=n::selection::Status::Ok)return;
  const auto in=Bind(*image);n::units_detail::Factors units;l::detail::Factors(in.current,units);
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<in.source.secondary_count;i+=blockDim.x*gridDim.x) {
    const unsigned row=reverse?unsigned(in.source.secondary_count)-1-i:i;
    const auto needs=l::detail::Requirements(in,row);
    if(needs.status!=n::selection::Status::Ok||needs.sliding>Sliding) {
      out->prepared[row]={};out->prepared[row].stage.report.status=n::selection::Status::CapacityExceeded;
    } else out->prepared[row]=l::detail::PrepareRowAfterNormals(in,row,Scratch(*out,row),units,out->optimized[row]);
  }
}
__global__ void AdmitCount(const Image* image,Work* out,std::size_t capacity) {
  if(blockIdx.x||threadIdx.x||out->report.status!=n::selection::Status::Ok)return;
  std::size_t count=0;
  for(unsigned row=0;row<image->input.source.secondary_count;++row) {
    const auto& r=out->prepared[row].stage.report;
    if(r.status!=n::selection::Status::Ok){out->report=r;return;}
    count+=r.required_candidates;
  }
  out->report.required_candidates=count;out->report.count_complete=true;
  if(count>capacity)out->report.status=n::selection::Status::CapacityExceeded;
}
__global__ void CompleteRows(const Image* image,Work* out,bool reverse) {
  if(out->report.status!=n::selection::Status::Ok)return;
  const auto in=Bind(*image);n::units_detail::Factors units;l::detail::Factors(in.current,units);
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<in.source.secondary_count;i+=blockDim.x*gridDim.x) {
    const unsigned row=reverse?unsigned(in.source.secondary_count)-1-i:i;
    out->prepared[row].stage=l::detail::CompleteRow(in,row,Scratch(*out,row),units,out->prepared[row]);
  }
}
// Test-only invalid writable span names a real live int subobject. The new
// read/write-disjoint admission must reject it before changing source fields.
__global__ void AliasedNormalScratch(Image* image,Work* out) {
  if(blockIdx.x||threadIdx.x||out->report.status!=n::selection::Status::Ok)return;
  const auto in=Bind(*image);n::units_detail::Factors units;l::detail::Factors(in.current,units);
  auto scratch=Scratch(*out,0);scratch.sliding_mains=&image->current_references[0].boundary;
  scratch.sliding_capacity=1;
  out->prepared[0]=l::detail::PrepareRow(in,0,scratch,units);
}
__global__ void FinishRows(const n::NativeGeometryHistory* in,n::NativeGeometryHistory* out,
    n::selection::Status* status,unsigned count) {
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    status[i]=l::FinishNativeRow(in[i],out+i);
}
inline void Check(cudaError_t error) {
  if(error!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(error));
}
template<class T,std::size_t N> inline void Copy(T(&out)[N],const std::vector<T>& in) {
  if(in.size()>N)throw std::runtime_error("Qualification packet bound exceeded");
  std::copy(in.begin(),in.end(),out);
}
class Device {
  cudaStream_t stream=nullptr;Image* image=nullptr;Work* work=nullptr;
 public:
  Device() {
    try {Check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));
      Check(cudaMalloc(&image,sizeof(Image)));Check(cudaMalloc(&work,sizeof(Work)));
    } catch(...) {Release();throw;}
  }
  ~Device(){Release();}
  void Release() {
    if(stream)cudaStreamSynchronize(stream);
    if(work)cudaFree(work);if(image)cudaFree(image);if(stream)cudaStreamDestroy(stream);
    work=nullptr;image=nullptr;stream=nullptr;
  }
  Device(const Device&)=delete;Device& operator=(const Device&)=delete;
  std::vector<l::RowResult> Finish(const std::vector<l::RowResult>& rows) {
    if(rows.size()>Rows)throw std::runtime_error("Finish fixture bound exceeded");
    std::vector<n::NativeGeometryHistory> input;for(const auto& row:rows)input.push_back(row.history);
    std::vector<n::NativeGeometryHistory> actual(rows.size());
    std::vector<n::selection::Status> status(rows.size());
    struct Drain {cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{stream};
    // Qualification scratch fields are disjoint and sufficiently aligned.
    auto* in=reinterpret_cast<n::NativeGeometryHistory*>(image);
    auto* out=reinterpret_cast<n::NativeGeometryHistory*>(work);
    auto* codes=reinterpret_cast<n::selection::Status*>(out+Rows);
    Check(cudaMemcpyAsync(in,input.data(),input.size()*sizeof(input[0]),cudaMemcpyHostToDevice,stream));
    FinishRows<<<1,32,0,stream>>>(in,out,codes,unsigned(rows.size()));Check(cudaGetLastError());
    Check(cudaMemcpyAsync(actual.data(),out,actual.size()*sizeof(actual[0]),cudaMemcpyDeviceToHost,stream));
    Check(cudaMemcpyAsync(status.data(),codes,status.size()*sizeof(status[0]),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));auto result=rows;
    for(std::size_t i=0;i<rows.size();++i) {
      EXPECT_EQ(status[i],n::selection::Status::Ok);result[i].history=actual[i];
    }
    return result;
  }
  l::Report Evaluate(Fixture& f,l::HostResult& result,bool reverse=false,
      std::size_t capacity=Rows*Slots,unsigned threads=32,bool separate_phases=false,
      const l::CurrentNormalView* current=nullptr,bool omit_legacy_references=false,
      bool alias_current_reference=false) {
    auto host=std::make_unique<Image>();host->input=f.Input();
    if(current)host->input.current_normals=*current;
    if(omit_legacy_references)host->input.source.normals=nullptr;
    const auto& view=host->input.current_normals;
    // Bounds on the fixture payload are independent of deliberately malformed
    // descriptor counts; do not dereference an overflow-sized declared span.
    if(f.mains.size()>Mains||f.normals.size()>Normals)throw std::runtime_error("Normal fixture bound exceeded");
    if(view.face_normals)std::copy_n(view.face_normals,std::min(view.normal_count,4*f.mains.size()),host->current_face_normals);
    if(view.references)std::copy_n(view.references,std::min(view.reference_count,f.normals.size()),host->current_references);
    Copy(host->nodes,f.nodes);Copy(host->positions,f.positions);Copy(host->velocities,f.velocities);
    Copy(host->mains,f.mains);Copy(host->secondary,f.secondary);Copy(host->normals,f.normals);
    Copy(host->accepted,f.accepted);Copy(host->spatial,f.spatial);
    Copy(host->no,f.normal_offsets);Copy(host->ne,f.normal_entries);
    Copy(host->ro,f.removed_offsets);Copy(host->re,f.removed_entries);
    Copy(host->so,f.spatial_offsets);Copy(host->se,f.spatial_entries);
    auto actual=std::make_unique<Work>();auto after=std::make_unique<Image>();
    // Drain is declared after every host transfer buffer, so any thrown error
    // drains queued copies before their input/output lifetimes end.
    struct Drain {cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{stream};
    Check(cudaMemcpyAsync(image,host.get(),sizeof(Image),cudaMemcpyHostToDevice,stream));
    Check(cudaMemsetAsync(work,0,sizeof(Work),stream));
    AdmitSource<<<1,1,0,stream>>>(image,work);Check(cudaGetLastError());
    if(alias_current_reference) {
      AliasedNormalScratch<<<1,1,0,stream>>>(image,work);Check(cudaGetLastError());
    } else if(separate_phases) {
      BeforeNormalRows<<<2,threads,0,stream>>>(image,work,reverse);Check(cudaGetLastError());
      AfterNormalRows<<<2,threads,0,stream>>>(image,work,reverse);Check(cudaGetLastError());
    } else {PrepareRows<<<2,threads,0,stream>>>(image,work,reverse);Check(cudaGetLastError());}
    AdmitCount<<<1,1,0,stream>>>(image,work,capacity);Check(cudaGetLastError());
    CompleteRows<<<2,threads,0,stream>>>(image,work,reverse);Check(cudaGetLastError());
    Check(cudaMemcpyAsync(actual.get(),work,sizeof(Work),cudaMemcpyDeviceToHost,stream));
    Check(cudaMemcpyAsync(after.get(),image,sizeof(Image),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));
    EXPECT_EQ(std::memcmp(host.get(),after.get(),sizeof(Image)),0); // Exact borrowed bytes stay untouched.
    auto report=actual->report;if(report.status!=n::selection::Status::Ok)return report;
    for(unsigned row=0;row<f.secondary.size();++row)
      if(actual->prepared[row].stage.report.status!=n::selection::Status::Ok) {
        auto failed=actual->prepared[row].stage.report;
        failed.required_candidates=report.required_candidates;failed.count_complete=true;return failed;
      }
    // Qualification readback reorders only integer occurrence ordinals; every
    // numerical row, cache, raw geometry and history above was computed on GPU.
    std::vector<l::detail::OrderedSlot> order;
    l::HostResult output;output.rows.resize(f.secondary.size());
    for(unsigned row=0;row<f.secondary.size();++row) {
      const auto& staged=actual->prepared[row].stage;output.rows[row]=staged.value;
      for(unsigned i=0;i<staged.occurrence_count;++i)order.push_back({row,row*Slots+i});
    }
    std::sort(order.begin(),order.end(),[&](auto a,auto b){
      return l::detail::Before(actual->occurrences[a.slot],a.row,actual->occurrences[b.slot],b.row);
    });
    for(const auto& entry:order) {
      auto occurrence=actual->occurrences[entry.slot];occurrence.cache.occurrence=output.occurrences.size();
      output.occurrences.push_back(occurrence);output.geometry.push_back(actual->geometry[entry.slot]);
    }
    result=std::move(output);return report;
  }
};
} // namespace type25_lifecycle_test::device
