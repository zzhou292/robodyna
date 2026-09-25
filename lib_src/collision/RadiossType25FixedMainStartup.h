// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/startup/Types.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::startup {
// Host startup path. The caller owns bounded arenas and all borrowed source
// lifetimes. No arena is allocated or grown by these operations; preflight
// reports aligned storage including private publication staging.
Forecast Preflight(std::size_t nodes,std::size_t primary_faces,Limits={}) noexcept;
// Count/profile-only forecast; does not dereference source spans. The old
// count overload remains unambiguous and preserves the legacy arena layout.
Forecast Preflight(const Input&,Limits={}) noexcept;
Report BuildStarter(const Input&,Limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot*) noexcept;
// Separate native fixed-main-ready stage. Snapshot must be an immutable result
// of BuildStarter for this same input/generation. Structural checks reject bad
// spans, ranges and reciprocal/reference endpoints; they are not authentication
// of a caller-fabricated or arbitrarily reordered CSR. The source owner retains
// that borrowed-result provenance and lifetime. It borrows actual resolved coefficients
// for source activity admission and preserves the Starter snapshot. Output,
// scratch, snapshot and source ranges must be disjoint. All caller-visible
// output bytes and view descriptors survive any rejected attempt unchanged.
Report BuildFixedMain(const Input&,const Snapshot&,const FixedMainInput&,Limits,
    tl::util::HostArena& output,tl::util::HostArena& scratch,FixedMainView*) noexcept;
} // namespace tlfea::contact::radioss_type25::startup
