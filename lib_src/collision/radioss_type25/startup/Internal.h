// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::startup::detail {
struct Edge { std::uint32_t low=0,high=0,main=0,slot=0; };
struct FaceKey { std::uint32_t nodes[4]{},count=0,ordinal=0; };
struct Identity { std::uint64_t id=0; std::uint32_t ordinal=0; };
struct OutputLayout {
  tl::util::ArenaRegion mains,expanded_to_primary,primary_to_partner;
  tl::util::ArenaRegion normals,references,normal_offsets,normal_mains;
};
struct Layout {
  OutputLayout output;
  tl::util::ArenaRegion points,edges,face_keys,identities,parents,tags,node_references;
  tl::util::ArenaRegion candidate_ids,candidate_angles,candidate_sides;
  Forecast forecast;
};
struct Data {
  Main* mains=nullptr;
  std::uint32_t* expanded_to_primary=nullptr;
  std::uint32_t* primary_to_partner=nullptr;
  StoredNormal* normals=nullptr;
  NormalReference* references=nullptr;
  std::uint32_t* normal_offsets=nullptr;
  std::uint32_t* normal_mains=nullptr;
};
bool Disjoint(const void*,std::size_t,const void*,std::size_t) noexcept;
Report MakeLayout(std::size_t nodes,std::size_t primary,Limits,Layout&,
    TopologyPolicy=TopologyPolicy::ManifoldTwoSided) noexcept;
Data Construct(tl::util::HostArena&,const OutputLayout&) noexcept;
Report CheckInput(const Input&,const Layout&,const tl::util::HostArena&,
    const tl::util::HostArena&,const void*,std::size_t) noexcept;
Report CheckSnapshot(const Input&,const Snapshot&,const FixedMainInput&,
    const tl::util::HostArena&,const tl::util::HostArena&,const FixedMainView*) noexcept;
Report Expand(const Input&,Data,Vector*,Identity*,FaceKey*) noexcept;
std::size_t BuildEdges(Data,std::size_t mains,Edge*) noexcept;
Report Topology(const Input&,Data,Edge*,std::size_t&) noexcept;
Report OrderedNeighbors(const Input&,Data,const Vector*,Edge*,int*,double*,double*) noexcept;
Report References(const Input&,Data,int*,int*,std::uint32_t*,std::size_t&,std::size_t&) noexcept;
Report StarterNormals(const Vector*,Data,std::size_t,std::size_t,std::size_t,StoredNormal*) noexcept;
Report FixedNormals(const Vector*,Data,std::size_t,std::size_t,std::size_t,StoredNormal*,int*,int*) noexcept;
} // namespace tlfea::contact::radioss_type25::startup::detail
