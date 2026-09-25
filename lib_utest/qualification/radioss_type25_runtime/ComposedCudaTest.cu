// SPDX-License-Identifier: AGPL-3.0-or-later
// Numerical pipeline qualification. This intentionally calls private numerical
// staging; physical publication and actual moving-owner tests are separate.
#include "lib_src/collision/radioss_type25/runtime/Launch.h"
#include "lib_src/collision/RadiossType25AssemblyDevice.h"
#include "../radioss_type25_lifecycle/Assertions.h"
#include "../radioss_type25_lifecycle/NativeOracle.h"
#include "../radioss_type25_local_geometry/NativeOracle.h"
#include "../radioss_type25_friction/NativeOracle.h"
#include "../radioss_type25_friction/Assertions.h"
#include "../radioss_type25_assembly/NativeOracle.h"
#include <cuda_runtime.h>
#include <stdexcept>
namespace {
namespace n=tlfea::contact::radioss_type25;namespace rd=n::runtime_detail;
namespace l=n::lifecycle;namespace a=n::assembly;namespace fe=tl::fea;
using Fixture=type25_lifecycle_test::Fixture;
void Check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
struct NumericalRig {
  cudaStream_t stream=nullptr;void* arena=nullptr;double* motion=nullptr;double* forces=nullptr;
  rd::Layout layout;rd::Device d;rd::Control control;n::TransactionConfig config;
  n::TransactionLimits limits;n::FixedMainSource source;n::units_detail::Factors units;
  l::Input input;std::vector<double> mass;
  std::unique_ptr<a::DeviceIncidenceBuilder> incidence;
  explicit NumericalRig(const Fixture& fixture,unsigned packet) {
    try {
      input=fixture.Input();source.selection=input.source;source.primary_main_count=1;source.force_packet_size=packet;
      limits.inventory.max_pairs=128;limits.optimized_candidates=128;limits.sliding_entries=1024;limits.max_device_bytes=16u<<20;
      config.units={1,1,1};config.lifecycle=input.profile;config.normal.engine={0,0,0};
      config.friction={2,10,0,1,0,0,1};config.friction_coefficients={.1,{0,0,0,0,.1,-.001}};
      config.assembly={0,0,0,0,0,{0,0,0}};n::units_detail::Make(config.units,units);
      Check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));std::size_t cub=0;
      Check(rd::QueryScratch(input.source.secondary_count,limits.optimized_candidates,cub));
      if(!rd::MakeLayout(source,limits,cub,layout))throw std::runtime_error("Layout failed");
      Check(cudaMalloc(&arena,layout.bytes));d=rd::Bind(arena,layout,source,limits);
      Check(cudaMalloc(&motion,6*input.source.node_count*sizeof(double)));
      Check(cudaMalloc(&forces,4*input.source.node_count*sizeof(double)));
      // All host transfer lifetimes are retained as fixture/this members through drain.
      Copy(fixture.nodes.data(),layout.nodes);Copy(fixture.mains.data(),layout.mains);Copy(fixture.normals.data(),layout.normals);
      Copy(fixture.normal_offsets.data(),layout.normal_offsets);Copy(fixture.normal_entries.data(),layout.normal_entries);
      Copy(fixture.removed_offsets.data(),layout.removed_offsets);Copy(fixture.removed_entries.data(),layout.removed_entries);
      for(unsigned slab=0;slab<2;++slab){Copy(fixture.secondary.data(),layout.secondary[slab]);Copy(fixture.accepted.data(),layout.history[slab]);}
      Copy(fixture.spatial.data(),{layout.spatial.offset,fixture.spatial.size(),fixture.spatial.size()*sizeof(l::SpatialOccurrence)});
      Copy(fixture.spatial_offsets.data(),layout.spatial_offsets);
      Copy(fixture.spatial_entries.data(),{layout.spatial_entries.offset,fixture.spatial_entries.size(),fixture.spatial_entries.size()*sizeof(std::uint32_t)});
      mass.resize(input.source.node_count);for(std::size_t i=0;i<mass.size();++i)mass[i]=.5+double(i)*.125;
      Copy(mass.data(),layout.native_mass);Copy(&control,layout.control);
      Check(cudaMemcpyAsync(motion,fixture.positions.data(),fixture.positions.size()*sizeof(double),cudaMemcpyHostToDevice,stream));
      Check(cudaMemcpyAsync(motion+fixture.positions.size(),fixture.velocities.data(),fixture.velocities.size()*sizeof(double),cudaMemcpyHostToDevice,stream));
      Check(cudaStreamSynchronize(stream));
      input.source=d.source;input.source.secondary=d.secondary[0];input.accepted_rows=d.history[0];
      input.current.positions.data=motion;input.current.velocities.data=motion+fixture.positions.size();
      input.spatial=d.spatial;input.spatial_by_secondary.offsets=d.spatial_offsets;input.spatial_by_secondary.entries=d.spatial_entries;
      incidence=std::make_unique<a::DeviceIncidenceBuilder>();
      if(incidence->Initialize({128,input.source.node_count,128,16u<<20},stream)!=a::IncidenceStatus::Ok)
        throw std::runtime_error("Incidence initialization failed");
    } catch(...){Release();throw;}
  }
  ~NumericalRig(){Release();}
  void Release(){if(stream)cudaStreamSynchronize(stream);incidence.reset();if(forces)cudaFree(forces);if(motion)cudaFree(motion);
    if(arena)cudaFree(arena);if(stream)cudaStreamDestroy(stream);forces=motion=nullptr;arena=nullptr;stream=nullptr;}
  void Copy(const void* source,tl::util::ArenaRegion r){if(r.bytes)Check(cudaMemcpyAsync(tl::util::ArenaPointer<std::byte>(arena,r),source,r.bytes,cudaMemcpyHostToDevice,stream));}
  void Fence(cudaError_t error){Check(error);Check(cudaMemcpyAsync(&control,d.control,sizeof(control),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));if(control.failure!=~0ull)throw std::runtime_error("Runtime device failure "+std::to_string(control.failure));}
  template<class T> std::vector<T> Read(const T* from,std::size_t count) {
    std::vector<T> out(count);if(count)Check(cudaMemcpyAsync(out.data(),from,count*sizeof(T),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));return out;
  }
  std::size_t Classify() {
    Fence(rd::Prepare(d,input,units,stream));if(control.required_sliding>limits.sliding_entries)throw std::runtime_error("Sliding cap");
    Fence(rd::CountCandidates(d,input,units,stream));const auto count=std::size_t(control.required_candidates);
    if(count>limits.optimized_candidates)throw std::runtime_error("Candidate cap");
    Fence(rd::Complete(d,input,units,count,stream));Fence(rd::Order(d,count,stream));return count;
  }
  l::HostResult Classification(std::size_t count) {
    l::HostResult out;const auto rows=Read(d.row_results,input.source.secondary_count);
    for(const auto& row:rows)out.rows.push_back(row.value);
    const auto raw=Read(d.occurrences,count);const auto order=Read(d.sorted_slots,count);
    const auto geometry=Read(d.geometry,count);
    for(auto slot:order){out.occurrences.push_back(raw[slot]);out.geometry.push_back(geometry[slot]);}return out;
  }
  std::vector<a::SiNodalValue> Assemble(const std::vector<a::NativeNodalValue>& incoming) {
    const auto kept=std::size_t(control.kept),cohorts=kept/source.force_packet_size+(kept%source.force_packet_size!=0);
    const a::Schedule schedule{cohorts?d.cohort_ends:nullptr,cohorts,kept};
    if(incidence->Stage({kept?d.force_connectivity:nullptr,schedule,input.source.node_count,{1,1,1,1,1}})!=a::IncidenceStatus::Ok)
      throw std::runtime_error("Incidence stage failed");
    const auto nodes=input.source.node_count;std::vector<double> values(4*nodes);
    for(std::size_t i=0;i<nodes;++i){values[i]=incoming[i].force.x;values[nodes+i]=incoming[i].force.y;
      values[2*nodes+i]=incoming[i].force.z;values[3*nodes+i]=incoming[i].stiffness;}
    Check(cudaMemcpyAsync(forces,values.data(),values.size()*sizeof(double),cudaMemcpyHostToDevice,stream));
    fe::NodalAssemblyView view;view.forces.force_x=forces;view.forces.force_y=forces+nodes;view.forces.force_z=forces+2*nodes;
    fe::NodalCinAssemblyView cin;cin.translational_stiffness=forces+3*nodes;
    Fence(rd::Gather(d,schedule,incidence->view().incidence(),view,cin,stream));
    return Read(d.nodal_output,nodes);
  }
};
// Independently assembled FOR3 inputs from source arrays; this reference does
// not call runtime::ForceInput or its packet/offset response controller.
n::NativeFrictionInput Arguments(const Fixture& f,const l::Occurrence& o,
    const n::NativeGeometryFinalResult& g,const std::vector<double>& mass,double kick) {
  n::NativeFrictionInput in;const auto row=o.selected.key.history_index;const auto node=f.secondary[row].node;
  in.normal={};in.normal.penetration=g.penetration;in.normal.stiffness=g.geometry.incoming_stiffness;
  in.normal.time=f.step.time;in.normal.dt=f.step.previous_dt;in.dt12=kick;in.normal.secondary_mass=mass[node];
  in.normal_axis=g.geometry.normal;in.relative_velocity={f.velocities[3*node],f.velocities[3*node+1],f.velocities[3*node+2]};
  for(unsigned k=0;k<4;++k){const auto main=f.mains[o.selected.local_main-1].nodes[k];
    const auto h=g.geometry.weights[k];in.normal.weights[k]=h;in.normal.main_mass[k]=mass[main];
    in.main_vertices[k]={f.positions[3*main],f.positions[3*main+1],f.positions[3*main+2]};
    in.relative_velocity.x-=h*f.velocities[3*main];in.relative_velocity.y-=h*f.velocities[3*main+1];
    in.relative_velocity.z-=h*f.velocities[3*main+2];}
  in.normal.normal_velocity=in.normal_axis.x*in.relative_velocity.x+in.normal_axis.y*in.relative_velocity.y+
      in.normal_axis.z*in.relative_velocity.z;return in;
}
void CompareComplete(Fixture f,unsigned packet) {
  const auto reference=type25_lifecycle_test::OracleLifecycle(f.Input());NumericalRig gpu(f,packet);
  const auto count=gpu.Classify();const auto classified=gpu.Classification(count);
  type25_lifecycle_test::Same(classified,reference);
  const double kick=.0005;gpu.Fence(rd::Respond(gpu.d,gpu.input,gpu.config,gpu.units,kick,1,count,gpu.control.kept,gpu.stream));
  auto rows=reference.rows;std::vector<n::NativeGeometryHistory> histories;
  for(const auto& row:rows)histories.push_back(row.history);
  std::vector<std::size_t> kept;for(std::size_t i=0;i<count;++i)if(reference.occurrences[i].secondary>0)kept.push_back(i);
  std::vector<n::NativeFrictionResult> responses(count),packed;
  std::vector<a::Connectivity> connectivity;std::vector<std::uint32_t> ends;
  for(std::size_t begin=0;begin<kept.size();begin+=packet) {
    const auto end=std::min(kept.size(),begin+packet);std::vector<n::NativeRawGeometryResult> geometry;
    for(std::size_t j=begin;j<end;++j)geometry.push_back(reference.geometry[kept[j]]);
    const auto finalized=type25_geometry_test::OracleHistory(f.profile.geometry,f.step.time,geometry,histories);histories=finalized.rows;
    for(std::size_t j=begin;j<end;++j) {
      const auto i=kept[j];const auto& occurrence=reference.occurrences[i];const auto row=occurrence.selected.key.history_index;
      type25_friction_test::Case input;input.normal_config=gpu.config.normal;input.controls=gpu.config.friction;
      input.coefficients=gpu.config.friction_coefficients;input.input=Arguments(f,occurrence,finalized.results[j-begin],gpu.mass,kick);
      input.history=histories[row].row.history;responses[i]=type25_friction_test::Oracle(input);histories[row].row.history=responses[i].history;
      packed.push_back(responses[i]);a::Connectivity c;for(unsigned k=0;k<4;++k)c.main[k]=f.mains[occurrence.local_main-1].nodes[k];
      c.secondary=f.secondary[row].node;connectivity.push_back(c);
    }
    ends.push_back(std::uint32_t(end));
  }
  for(std::size_t row=0;row<rows.size();++row)rows[row].history=histories[row];rows=type25_lifecycle_test::OracleFinish(rows);
  const auto actual_history=gpu.Read(gpu.d.history[1],rows.size());
  for(std::size_t row=0;row<rows.size();++row)type25_geometry_test::Same(actual_history[row],rows[row].history,false);
  const auto actual_responses=gpu.Read(gpu.d.responses,count);const auto order=gpu.Read(gpu.d.sorted_slots,count);
  for(std::size_t i=0;i<count;++i)if(reference.occurrences[i].secondary>0)
    type25_friction_test::Same(actual_responses[order[i]],responses[i]);
  std::vector<a::NativeNodalValue> incoming(f.nodes.size());
  for(std::size_t i=0;i<incoming.size();++i)incoming[i]={{double(i)+.25,-double(i)-.5,double(i)*.125},double(i)+.75};
  const auto native=type25_assembly_test::NativeAssemble(connectivity,packed,ends,incoming);const auto actual=gpu.Assemble(incoming);
  for(std::size_t i=0;i<actual.size();++i){SCOPED_TRACE(i);type25_geometry_test::Number(actual[i].force.x,native[i].force.x,false);
    type25_geometry_test::Number(actual[i].force.y,native[i].force.y,false);type25_geometry_test::Number(actual[i].force.z,native[i].force.z,false);
    type25_geometry_test::Number(actual[i].stiffness,native[i].stiffness,false);}
}
TEST(NativeType25RuntimeCuda,CompleteUnseededNewImpactPipelineMatchesIndependentNative) {
  Fixture f;f.AddSecondary(6,1,.2);f.spatial={{2,3},{1,1},{1,3}};f.Rebuild();CompareComplete(f,128);
}
TEST(NativeType25RuntimeCuda,RetainedRowsRespectSourceForcePacketBoundaries) {
  for(unsigned packet:{1u,2u,128u}) {SCOPED_TRACE(packet);Fixture f;f.Retained();f.step.time=.01;
    f.spatial={{1,1},{1,3}};f.AddSecondary(6,1,.2);f.accepted[1]=f.accepted[0];
    f.accepted[1].secondary_source_id=f.nodes[f.secondary[1].node].source_id;
    f.accepted[1].row.irtlm[0]=33;f.accepted[1].row.irtlm[2]=3;
    f.Rebuild();CompareComplete(f,packet);}
}
TEST(NativeType25RuntimeCuda,EmptyOptimizedSetPreservesIncomingNodalValues) {
  Fixture f;f.positions[20]=100;f.spatial.clear();f.Rebuild();CompareComplete(f,2);
}
} // namespace
