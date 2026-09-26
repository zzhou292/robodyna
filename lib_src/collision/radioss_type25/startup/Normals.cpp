// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FloatNormals.h"
#include <initializer_list>
namespace tlfea::contact::radioss_type25::startup::detail {
Report StarterNormals(const Vector* points,Data data,std::size_t p,std::size_t g,
    std::size_t references,StoredNormal* previous,const PostGapmTopology* post) noexcept {
  const float floor=fp::StarterFloor();auto report=fp::Primary(points,data,p,floor,post);
  if(report.status!=Status::Ok)return report;
  report=fp::FreeEdges(points,data,g,floor,post);if(report.status!=Status::Ok)return report;
  for(std::size_t m=0;m<g;++m) {
    if(post && post->final_support[m].second_solid_source_id)continue;
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(main.neighbors[k] || (k==2 && main.nodes[2]==main.nodes[3]))continue;
      for(unsigned endpoint:{k,(k+1)%4}) {
        auto& ref=data.references[std::size_t(main.normal_reference[endpoint]-1)];
        ref.boundary=1; // I25NEIGH's boolean boundary, not NORMP's later count.
        if(fp::Zero(ref.bisector[0]))ref.bisector[0]=data.normals[4*m+k];
        else ref.bisector[1]=data.normals[4*m+k];
      }
    }
  }
  (void)references;
  return fp::AverageNeighbors(data,g,floor,previous,post);
}
Report FixedNormals(const Vector* points,Data data,std::size_t p,std::size_t g,
    std::size_t references,StoredNormal* previous,int* first_slot,int* second_slot) noexcept {
  const float floor=fp::ReadyFloor();auto report=fp::Primary(points,data,p,floor);
  if(report.status!=Status::Ok)return report;
  // I25MAIN_NORM clears all VTX_BISECTOR slots before its FLAG1 NORMP call;
  // NORMP then clears LBOUND. Unassigned channels therefore retain positive zero.
  for(std::size_t r=0;r<references;++r)data.references[r]={};
  // I25FREE_BOUND visits increasing main IDs. The admitted all-positive fixed
  // profile keeps every free main. Retain each original FREE_BOUND insertion
  // slot BEFORE replacing the raw normal with its free-edge direction.
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      const auto index=4*m+k;first_slot[index]=second_slot[index]=3;
      if(main.neighbors[k] || (k==2 && main.nodes[2]==main.nodes[3]))continue;
      if(fp::Zero(data.normals[index]))continue;
      first_slot[index]=++data.references[std::size_t(main.normal_reference[k]-1)].boundary;
      second_slot[index]=++data.references[std::size_t(main.normal_reference[(k+1)%4]-1)].boundary;
    }
  }
  for(std::size_t r=0;r<references;++r)
    if(data.references[r].boundary>2)return {Status::UnsupportedTopology};
  report=fp::FreeEdges(points,data,g,floor);if(report.status!=Status::Ok)return report;
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(main.neighbors[k] || (k==2 && main.nodes[2]==main.nodes[3]))continue;
      const auto index=4*m+k;const auto value=data.normals[index];
      if(first_slot[index]<=2)
        data.references[std::size_t(main.normal_reference[k]-1)].bisector[first_slot[index]-1]=value;
      if(second_slot[index]<=2)
        data.references[std::size_t(main.normal_reference[(k+1)%4]-1)].bisector[second_slot[index]-1]=value;
    }
  }
  return fp::AverageNeighbors(data,g,floor,previous);
}
} // namespace tlfea::contact::radioss_type25::startup::detail
