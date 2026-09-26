// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RadiossType25InterfaceSurface.h"
#include "../source_surfaces/Internal.h"
#include "../startup/CoatingOrientation.h"
namespace tlfea::contact::radioss_type25::surface_interface::detail {
namespace source=source_surfaces::detail;
struct Key { std::uint32_t words[6]{},raw=0; };
struct OutputLayout {
  tl::util::ArenaRegion primary,identities,classifications,raw_to_primary,primary_to_raw,raw_origins,solid_flags;
};
struct Layout {
  OutputLayout output,staged;
  tl::util::ArenaRegion points,keys,flags,selected,face_mask,parts;
  tl::util::ArenaRegion solid_offsets,quad_offsets,triangle_offsets;
  tl::util::ArenaRegion solid_rows,quad_rows,triangle_rows,cursor;
  Forecast forecast;
};
struct Data {
  startup::PrimaryFace* primary=nullptr;
  startup::PrimaryFaceIdentity* identities=nullptr;
  RawClassification* classifications=nullptr;
  startup::PrimaryFaceIdentity* raw_origins=nullptr;
  std::uint8_t* solid_flags=nullptr;
  std::uint32_t *raw_to_primary=nullptr,*primary_to_raw=nullptr;
};
Report FromPhysical(source_surfaces::Report) noexcept;
Report MakeLayout(const Input&,Limits,Layout&) noexcept;
Report Admit(const Input&,const Layout&,const tl::util::HostArena&,const tl::util::HostArena&,const Snapshot*) noexcept;
Data Construct(tl::util::HostArena&,const OutputLayout&) noexcept;
source::Work Context(tl::util::HostArena&,const Layout&) noexcept;
Report Prepare(const Input&,source::Work,Vector*) noexcept;
Report Classify(const Input&,source::Work,const Vector*,Data) noexcept;
Report Filter(const Input&,Data,Key*,std::size_t&,std::size_t&) noexcept;
}
