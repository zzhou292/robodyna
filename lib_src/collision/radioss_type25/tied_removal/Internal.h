// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RadiossType25TiedRemoval.h"
#include "../search/Ranges.h"
namespace tlfea::contact::radioss_type25::tied_removal::detail {
struct Relation { std::uint32_t interface=0,row=0; bool secondary=false; };
struct DataLayout {
  tl::util::ArenaRegion extent,main_offsets,secondary_offsets,nodes,mains,contact,history;
};
struct Layout {
  Forecast forecast;
  DataLayout output,staged;
  tl::util::ArenaRegion offsets,cursors,secondary,seen,discovered,relations,ids,auxiliary_ids;
};
struct Data {
  double* extent;
  std::uint32_t *main_offsets,*secondary_offsets,*nodes,*mains;
  int* contact;
  History* history;
};
struct Work {
  std::uint32_t *offsets,*cursors,*secondary,*seen,*discovered;
  Relation* relations;
  std::uint64_t* ids;
  std::uint64_t* auxiliary_ids;
};
Report Plan(const Input&,Limits,Layout&) noexcept;
Report Admit(const Input&,Limits,const Layout&,const tl::util::HostArena&,
    const tl::util::HostArena&,const Snapshot*) noexcept;
Report Prepare(const Input&,Work) noexcept;
Report Evaluate(const Input&,Limits,const Layout&,Work,Data) noexcept;
Data Construct(tl::util::HostArena&,const DataLayout&) noexcept;
Work ConstructWork(tl::util::HostArena&,const Layout&) noexcept;
inline unsigned Corners(const Main& main) noexcept {return main.nodes[2]==main.nodes[3]?3:4;}
} // namespace tlfea::contact::radioss_type25::tied_removal::detail
