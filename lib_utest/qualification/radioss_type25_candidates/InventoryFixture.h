#pragma once
#include "PackingFixture.h"
#include "lib_src/collision/RadiossType25Candidates.h"
#include <cuda_runtime.h>
#include <stdexcept>
#include <tuple>
namespace candidate_test {
extern "C" void rd_pen3_inventory(const int*,const double*,const double*,const double*,const int*,const int*,double*);
inline double NativeClearance(const c::LocalRow& row) {
  const auto packed=NativePack(row);double xyz[30]{},gaps[]{packed.gap,0.},result[2]{};int flags[12]{};
  for(unsigned i=0;i<4;++i) {
    xyz[3*i]=row.screen.vertices[i].x;xyz[3*i+1]=row.screen.vertices[i].y;xyz[3*i+2]=row.screen.vertices[i].z;
    flags[i]=int(row.nodes[i]);
  }
  xyz[12]=row.screen.secondary.x;xyz[13]=row.screen.secondary.y;xyz[14]=row.screen.secondary.z;
  flags[4]=row.segment_type;flags[5]=packed.symmetry;const int count=1;
  rd_pen3_inventory(&count,xyz,gaps,&row.screen.margin,flags,&row.main_count,result);return result[0];
}
template<class T> struct Managed {
  T* pointer=nullptr;std::size_t size=0;
  Managed()=default;Managed(const Managed&)=delete;Managed& operator=(const Managed&)=delete;
  ~Managed(){if(pointer)cudaFree(pointer);}
  void Set(const std::vector<T>& source) {
    if(pointer)throw std::logic_error("fixture resize forbidden");size=source.size();
    if(size&&cudaMallocManaged(&pointer,size*sizeof(T))!=cudaSuccess)throw std::runtime_error("managed allocation");
    if(size)std::copy(source.begin(),source.end(),pointer);
  }
  T& operator[](std::size_t i){return pointer[i];}
};
struct Scene {
  std::vector<std::uint64_t> ids,removal_offsets;
  std::vector<int> codes;
  std::vector<std::uint32_t> secondaries,removals;
  std::vector<c::Main> mains;
  Managed<double> positions,velocities,secondary_stiffness,secondary_gaps,main_stiffness,main_gaps,curvature;
  cudaStream_t stream=nullptr;
  explicit Scene(unsigned main_count=6,unsigned secondary_count=18) {
    if(cudaStreamCreate(&stream)!=cudaSuccess)throw std::runtime_error("stream");
    std::vector<double> xyz,velocity;
    for(unsigned m=0;m<main_count;++m) {
      c::Main main;main.source_id=100+(main_count-m)/2;main.segment_type=m%3==0?0:1;
      const double origin=double(m%3)*3.;
      double points[12]={origin-1,-1,0,origin+1,-1,0,origin+1,1,.0625*double(m%2),origin-1,1,0};
      for(unsigned j=0;j<4;++j) {
        main.nodes[j]=ids.size();ids.push_back(1000+ids.size());codes.push_back(0);
        for(unsigned a=0;a<3;++a){xyz.push_back(points[3*j+a]);velocity.push_back(.125*double((m+j+a)%3));}
      }
      if(m%2)main.nodes[3]=main.nodes[2];mains.push_back(main);
    }
    for(unsigned i=0;i<secondary_count;++i) {
      secondaries.push_back(ids.size());ids.push_back(1000+ids.size());codes.push_back(0);
      xyz.insert(xyz.end(),{double(i%3)*3.,double(i%2)*.125,.125*double(1+i%3)});
      velocity.insert(velocity.end(),{-.125,.25,-.5});
    }
    if(secondary_count>3&&main_count)secondaries[2]=mains[0].nodes[0];
    if(secondary_count>4)secondaries[3]=secondaries[0]; // repeated native role occurrence
    removal_offsets.push_back(0);
    for(unsigned m=0;m<main_count;++m) {
      if(secondary_count>5&&m%2==0){removals.push_back(secondaries[5]);removals.push_back(secondaries[4]);}
      removal_offsets.push_back(removals.size());
    }
    if(ids.empty()){ids.push_back(1);codes.push_back(0);xyz={0,0,0};velocity={0,0,0};}
    positions.Set(xyz);velocities.Set(velocity);
    secondary_stiffness.Set(std::vector<double>(secondary_count,1.));secondary_gaps.Set(std::vector<double>(secondary_count,.125));
    main_stiffness.Set(std::vector<double>(main_count,1.));main_gaps.Set(std::vector<double>(main_count,.125));
    curvature.Set(std::vector<double>(main_count,.03125));
  }
  ~Scene(){if(stream)cudaStreamDestroy(stream);}
  c::Source Source(c::InputUnits mode=c::InputUnits::Native) const {
    c::Source s;s.stamp={71,81};s.units={.001,1000.,1.};s.input_units=mode;
    s.physical_nodes=ids.size();s.secondaries=secondaries.size();s.mains=mains.size();s.removals=removals.size();
    s.node_ids=ids.data();s.constraint_codes=codes.data();s.secondary_nodes=secondaries.empty()?nullptr:secondaries.data();
    s.main=mains.empty()?nullptr:mains.data();s.removal_offsets=removal_offsets.data();
    s.removal_nodes=removals.empty()?nullptr:removals.data();s.native_main_count=std::max<std::size_t>(1,mains.size());return s;
  }
  c::Current Current() const {
    c::Current x;x.stamp={{71,81},1,1,1,1,1};
    x.positions={positions.pointer,std::uint32_t(ids.size()),3,1};x.velocities={velocities.pointer,std::uint32_t(ids.size()),3,1};
    x.secondary_stiffness=secondary_stiffness.pointer;x.secondary_gaps=secondary_gaps.pointer;
    x.main_stiffness=main_stiffness.pointer;x.main_gaps=main_gaps.pointer;x.main_curvature=curvature.pointer;
    x.domain={{-100,-100,-100},{100,100,100}};x.margin=.25;x.stored_motion=.0625;x.previous_dt=.125;return x;
  }
};
inline c::Limits Limits(std::size_t pairs=4096,std::size_t tasks=512) {
  c::Limits limits;limits.max_pairs=pairs;limits.max_tasks=tasks;limits.max_device_bytes=32u<<20;return limits;
}
inline c::Vector Read(const c::Current& in,std::uint32_t node,bool velocity=false) {
  const auto x=(velocity?in.velocities:in.positions).at(node);return {x.x,x.y,x.z};
}
inline std::vector<c::Pair> Reference(const Scene& scene,const c::Current& in) {
  std::vector<c::Pair> result;
  for(std::size_t s=0;s<scene.secondaries.size();++s) {
    if(in.secondary_stiffness[s]<=0.)continue;const auto node=scene.secondaries[s];const auto point=Read(in,node);
    const auto lo=in.domain.minimum,hi=in.domain.maximum;
    if(point.x<lo.x||point.x>hi.x||point.y<lo.y||point.y>hi.y||point.z<lo.z||point.z>hi.z)continue;
    for(std::size_t m=0;m<scene.mains.size();++m) {
      if(in.main_stiffness[m]<=0.)continue;const auto& main=scene.mains[m];bool excluded=false;
      for(auto own:main.nodes)if(node==own)excluded=true;
      for(auto k=scene.removal_offsets[m];k<scene.removal_offsets[m+1];++k)if(scene.removals[k]==node)excluded=true;
      if(excluded)continue;
      c::LocalRow row;row.secondary_node=scene.ids[node];row.screen.secondary=point;row.screen.margin=in.margin;
      row.screen.secondary_gap=in.secondary_gaps[s];row.screen.main_gap=in.main_gaps[m];row.screen.curvature=in.main_curvature[m];
      row.screen.gap_load=in.gap_load;row.screen.drad=in.drad;row.screen.stored_motion=in.stored_motion;
      for(unsigned j=0;j<4;++j)row.screen.vertices[j]=Read(in,main.nodes[j]);
      if(!NativeScreen(row.screen))continue;
      row.previous_dt=in.previous_dt;row.secondary_velocity=Read(in,node,true);row.constraint_codes[4]=scene.codes[node];
      row.segment_type=main.segment_type;row.main_count=scene.Source().native_main_count;
      for(unsigned j=0;j<4;++j){row.nodes[j]=scene.ids[main.nodes[j]];row.main_velocities[j]=Read(in,main.nodes[j],true);row.constraint_codes[j]=scene.codes[main.nodes[j]];}
      if(NativeClearance(row)!=0.)result.push_back({std::uint32_t(s),std::uint32_t(m)});
    }
  }
  std::sort(result.begin(),result.end(),[&](c::Pair a,c::Pair b) {
    return std::tie(a.secondary_row,scene.mains[a.main_occurrence].source_id,a.main_occurrence)<
        std::tie(b.secondary_row,scene.mains[b.main_occurrence].source_id,b.main_occurrence);
  });return result;
}
} // namespace candidate_test
