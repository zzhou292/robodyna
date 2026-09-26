// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RadiossType25SurfaceSource.h"
#include "../source_nodal/HostRanges.h"
#include "../search/Ranges.h"
#include <algorithm>
#include <cstring>
namespace tlfea::contact::radioss_type25::source_surfaces::detail {
struct Layout {
  tl::util::ArenaRegion faces,flags;
  tl::util::ArenaRegion staged_faces,staged_flags,selected,face_mask,parts;
  tl::util::ArenaRegion solid_offsets,quad_offsets,triangle_offsets;
  tl::util::ArenaRegion solid_rows,quad_rows,triangle_rows,cursor;
  Forecast forecast;
};
struct Work {
  Face* faces=nullptr;
  std::uint8_t *surface_flags=nullptr,*selected=nullptr,*face_mask=nullptr;
  std::uint64_t* parts=nullptr;
  std::uint32_t *solid_offsets=nullptr,*quad_offsets=nullptr,*triangle_offsets=nullptr;
  std::uint32_t *solid_rows=nullptr,*quad_rows=nullptr,*triangle_rows=nullptr,*cursor=nullptr;
};
Report MakeLayout(const Input&,Limits,Layout&) noexcept;
Report Admit(const Input&,const Layout&,const tl::util::HostArena&,const tl::util::HostArena&,const Snapshot*) noexcept;
Work Borrow(tl::util::HostArena&,const Layout&) noexcept;
Report Prepare(const Input&,Work,Counts&) noexcept;
Report Extract(const Input&,Work,Counts&,std::size_t&) noexcept;
bool SelectedPart(const Input&,const Work&,std::uint64_t) noexcept;
inline constexpr unsigned Faces[6][4]{{3,2,1,0},{4,5,6,7},{0,1,5,4},
  {2,3,7,6},{1,2,6,5},{0,4,7,3}};
}
