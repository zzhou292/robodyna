// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 I25NEIGH/SEG_EN/SEG_E/SEG_OPP and ADD_ID.
// Serial main/edge order is intentional: reciprocal writes affect later choices.
#include "NeighborGeometry.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup::detail {
namespace {
bool Contains(const Main& face,std::uint32_t node) noexcept {
  for(const auto n:face.nodes)if(n==node)return true;
  return false;
}
bool SameSegment(const Main& first,const Main& second) noexcept {
  for(const auto node:second.nodes)if(!Contains(first,node))return false;
  return true;
}
unsigned Slot(const Main& face,std::uint32_t node) noexcept {
  for(unsigned k=0;k<4;++k)if(face.nodes[k]==node)return k;
  return 4;
}
bool Nok(const Main& self,const Main& other,std::uint32_t i1,std::uint32_t i2,
    unsigned j1,unsigned j2) noexcept {
  if(self.nodes[2]==self.nodes[3])
    for(unsigned j=0;j<4;++j)if(self.nodes[j]!=i1 && self.nodes[j]!=i2)
      for(unsigned k=0;k<4;++k)if(k!=j1 && k!=j2 && self.nodes[j]==other.nodes[k])return true;
  if(other.nodes[2]==other.nodes[3])
    for(unsigned j=0;j<4;++j)if(j!=j1 && j!=j2)
      for(unsigned k=0;k<4;++k)if(self.nodes[k]!=i1 && self.nodes[k]!=i2 && other.nodes[j]==self.nodes[k])return true;
  return false;
}
bool AddId(int* ice,std::size_t& count,std::size_t capacity,int id) noexcept {
  if(id)for(std::size_t i=0;i<count;++i)if(ice[i]==id)return true;
  if(count==capacity)return false;
  ice[count++]=id;return true;
}
void RotateMinimum(const Main& face,std::uint32_t (&out)[4]) noexcept {
  const unsigned count=face.nodes[2]==face.nodes[3]?3:4;
  unsigned minimum=0;
  for(unsigned k=0;k<count;++k)if(face.nodes[k]<=face.nodes[minimum])minimum=k;
  for(unsigned k=0;k<count;++k)out[k]=face.nodes[(minimum+k)%count];
  if(count==3)out[3]=out[2];
}
bool Opposite(const Main& first,const Main& second) noexcept {
  std::uint32_t a[4],b[4];RotateMinimum(first,a);RotateMinimum(second,b);
  if(a[2]!=a[3] && b[2]!=b[3])return a[0]==b[0]&&a[1]==b[3]&&a[2]==b[2]&&a[3]==b[1];
  if(a[2]==a[3] && b[2]==b[3])return a[0]==b[0]&&a[1]==b[2]&&a[2]==b[1];
  return false;
}
bool Collect(Data data,const Edge* edges,std::size_t count,std::size_t self,
    std::uint32_t i1,std::uint32_t i2,int* ice,std::size_t& ne,std::size_t g) noexcept {
  const auto before=ne;
  if(i1==i2)return AddId(ice,ne,g+4,0); // Repeated T3 edge, original zero convention.
  const auto low=std::min(i1,i2),high=std::max(i1,i2);
  const Edge key{low,high,0,0};
  const auto* first=std::lower_bound(edges,edges+count,key,[](const Edge& a,const Edge& b) {
    return a.low<b.low || (a.low==b.low && a.high<b.high);
  });
  const auto& main=data.mains[self];
  for(auto it=first;it!=edges+count && it->low==low && it->high==high;++it) {
    if(it->main==self)continue;
    const auto& other=data.mains[it->main];
    // A sorted bucket contains each actual edge once, ordered by native main ID.
    // This is exactly the relevant subsequence of EIDNOD at endpoint I2.
    if(other.nodes[it->slot]!=i2 || other.nodes[(it->slot+1)%4]!=i1 || SameSegment(main,other))continue;
    const auto role1=main.segment_type<0?-main.segment_type:main.segment_type;
    const auto role2=other.segment_type<0?-other.segment_type:other.segment_type;
    if((std::size_t(role1)>g)!=(std::size_t(role2)>g))continue;
    const auto j1=Slot(other,i1);auto j2=Slot(other,i2);
    if(j1==4 || j2==4)return false;
    if(other.nodes[2]==other.nodes[3] && j2==2 && j1==0)j2=3;
    if(!other.neighbors[j2] && !Nok(main,other,i1,i2,j1,j2))
      if(!AddId(ice,ne,g+4,int(it->main+1)))return false;
  }
  return before!=ne || AddId(ice,ne,g+4,0);
}
}
Report OrderedNeighbors(const Input& in,Data data,const Vector* points,Edge* edges,
    int* ice,double* angles,double* sides,const PostGapmTopology* post,int* solid_tags) noexcept {
  const auto g=data.main_count;
  const auto count=BuildEdges(data,g,edges,post,post?EdgePopulation::External:EdgePopulation::All);
  if(post)std::fill_n(solid_tags,g,0);
  Report report{Status::Ok};
  auto fail=[&](Status status,std::size_t m,std::size_t node=SIZE_MAX) {
    report.status=status;report.primary=data.expanded_to_primary[m];report.node=node;return report;
  };
  for(std::size_t m=0;m<g;++m) {
    if(post && post->final_support[m].second_solid_source_id) {
      // Original ordinary incidence/neighbor pass excludes internal segments.
      // IDEL1 later marks them for the additional solid-only union pass.
      solid_tags[m]=post->final_solid_erosion==SolidErosion::Enabled?1:0;
      continue;
    }
    auto& main=data.mains[m];std::size_t ne=0;
    for(unsigned edge=0;edge<4;++edge) {
      const auto before=ne;
      if(before!=edge)return fail(Status::UnsupportedTopology,m);
      // Native fast return advances NE only. ICE is NOT filled from MVOISIN.
      if(main.neighbors[before]){++ne;continue;}
      const auto i1=main.nodes[edge],i2=main.nodes[(edge+1)%4];
      if(!Collect(data,edges,count,m,i1,i2,ice,ne,g))return fail(Status::ResourceLimit,m,i1);
      const auto candidates=ne-before;
      if(candidates>1) {
        if(post && post->final_solid_erosion==SolidErosion::Enabled &&
            post->final_support[m].first.kind==PhysicalSupportKind::EightSlotSolid)solid_tags[m]=1;
        if(candidates>g)return fail(Status::ResourceLimit,m,i1);
        if(!neighbor_geometry::Scores(main,data.mains,points,i1,i2,ice+before,candidates,angles,sides))
          return fail(Status::NonfiniteResult,m,i1);
        const auto winner=neighbor_geometry::Winner(angles,sides,candidates);
        ice[before]=winner==SIZE_MAX?0:ice[before+winner];
        if(winner==SIZE_MAX) {
          if(!report.neighbor_warnings.count) {
            report.neighbor_warnings.first_main=m;report.neighbor_warnings.first_edge=edge;
          }
          ++report.neighbor_warnings.count;
        }
        std::fill_n(ice+before+1,candidates-1,0);ne=before+1;
      }
      if(ne==before || ne-before>1)return fail(Status::UnsupportedTopology,m,i1); // Native IRR12/undefined slot.
      if(ice[ne-1]>0 && Opposite(main,data.mains[ice[ne-1]-1]))ice[ne-1]=0;
      main.neighbors[before]=ice[before];
      if(main.neighbors[before]) {
        auto& other=data.mains[main.neighbors[before]-1];
        const auto j1=Slot(other,i1);auto j2=Slot(other,i2);
        if(j1==4||j2==4)return fail(Status::UnsupportedTopology,m,i1);
        if(other.nodes[2]==other.nodes[3]&&j2==2&&j1==0)j2=3;
        other.neighbors[j2]=int(m+1);
      }
    }
    // Native MVOI reset covers only the retained four-slot prefix. Discarded
    // multiple-candidate tail entries were cleared by REMOVEALLBUT1 above.
    std::fill_n(ice,ne,0);
  }
  // Original I25NEIGH derives EVOISIN from complete reciprocal reversed edges
  // only after every main's selection has finished.
  for(std::size_t m=0;m<g;++m)for(unsigned edge=0;edge<4;++edge) {
    auto& main=data.mains[m];if(!main.neighbors[edge])continue;
    const auto other=std::size_t(main.neighbors[edge]-1);
    unsigned found=4;
    for(unsigned k=0;k<4;++k) {
      const auto& next=data.mains[other];if(k==2 && next.nodes[2]==next.nodes[3])continue;
      if(next.nodes[k]==main.nodes[(edge+1)%4] && next.nodes[(k+1)%4]==main.nodes[edge])found=k;
    }
    if(found==4 || data.mains[other].neighbors[found]!=int(m+1))return fail(Status::UnsupportedTopology,m);
    main.neighbor_edges[edge]=int(found+1);
  }
  return report;
}
// Original IDEL1 MNEIGH_SOLID lists are pure functions of the now-immutable
// MVOISIN, solid-support incidence and face words. Reconstruct one complete
// main's four lists before its reference unions. Preserve cross-edge ADD_ID
// deduplication and each edge's original main-ID insertion order; no dense
// NMAX*4*NRTM_S allocation or scan of all G faces is needed.
bool SolidNeighborLists(Data data,const Edge* edges,std::size_t count,std::size_t main,
    int* ids,std::size_t (&offsets)[5]) noexcept {
  std::size_t used=0;offsets[0]=0;
  const auto& face=data.mains[main];
  for(unsigned edge=0;edge<4;++edge) {
    if(!Collect(data,edges,count,main,face.nodes[edge],face.nodes[(edge+1)%4],ids,used,data.main_count))return false;
    offsets[edge+1]=used;
  }
  return true;
}

} // namespace tlfea::contact::radioss_type25::startup::detail
