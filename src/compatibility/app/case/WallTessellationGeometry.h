#pragma once

#include "WallTessellation.h"
#include <map>
#include <set>

namespace crash::case_data::wall_tessellation_detail {
using Edge=std::pair<std::uint32_t,std::uint32_t>;
using Edges=std::map<Edge,unsigned>;
Edge Key(std::uint32_t,std::uint32_t);
Edges CountEdges(tlfea::contact::PlanarWallView);
std::set<Edge> Boundary(const Edges&);
// False means the pair is not a resolved strictly convex four-node patch.
// Corner cycle follows the source -X winding and starts at its smallest index.
bool ConvexCycle(tlfea::contact::PlanarWallView,std::uint32_t first,std::uint32_t second,
                 double wall_tolerance,std::array<std::uint32_t,4>& cycle);
tlfea::contact::Vec3 Midpoint(tlfea::contact::Vec3,tlfea::contact::Vec3,WallMidpointSource& metadata);
// Exact original index boundary for original/flip. For subdivision, every
// exposed edge must be precisely split by its one shared midpoint; uncertainty
// only describes the declared midpoint coordinates on diagonal source edges.
void CheckBoundary(tlfea::contact::PlanarWallView original,tlfea::contact::PlanarWallView derived,
                   WallTessellationMetadata&,const std::map<Edge,std::uint32_t>& midpoints,double tolerance);
std::string MeshSha256(tlfea::contact::PlanarWallView,WallTessellationKind);
} // namespace crash::case_data::wall_tessellation_detail
