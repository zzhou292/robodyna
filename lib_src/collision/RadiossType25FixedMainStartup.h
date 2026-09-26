// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/startup/Types.h"
#include "radioss_type25/startup/PostGapmTypes.h"
#include "radioss_type25/startup/CoatingOrientation.h"
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
// Complete mixed SH2 stage only. TrueG=P+shell_count; no solid partners.
// The separate Snapshot/BuildStarter interface remains closed for mixed
// inputs until post-I25GAPM internal-support/erosion semantics are supplied.
Forecast PreflightMixedSides(const Input&,Limits={}) noexcept;
Report BuildMixedSides(const Input&,Limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,MixedSidesSnapshot*) noexcept;
// Clone genuine sides, apply only primary permutations, then construct
// neighbors/reference CSR/Starter normals from authentic post-GAPM support.
// The original BuildStarter overload continues to reject mixed input.
Forecast PreflightMixedStarter(const Input&,const MixedSidesSnapshot&,
    const PostGapmTopology&,Limits={}) noexcept;
Report BuildStarter(const Input&,const MixedSidesSnapshot&,const PostGapmTopology&,
    Limits,tl::util::HostArena& output,tl::util::HostArena& scratch,Snapshot*) noexcept;
// Count-only aligned storage forecast. This performs no borrowed-source or
// prefix validation; call the typed preflight after real arrays are available.
Forecast ForecastMixedStarterStorage(std::size_t nodes,std::size_t primaries,
    std::size_t shell_primaries,std::size_t raw_origins,Limits={}) noexcept;
// Explicit combined-domain overload. It validates an unchanged original node
// prefix and genuine original sides before one shared topology build. Added
// IDs must be unique and all coordinates finite, as in the original admission.
// No contact face may reference the added suffix. Old overloads stay exact.
// Prefix preflight requires an explicit Limits argument, preserving legacy
// calls whose fourth argument is an empty Limits initializer.
Forecast PreflightMixedStarter(const Input&,const MixedSidesSnapshot&,
    const PostGapmTopology&,const NodePrefixExtension&,Limits) noexcept;
Report BuildStarter(const Input&,const MixedSidesSnapshot&,const PostGapmTopology&,
    const NodePrefixExtension&,Limits,tl::util::HostArena& output,
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
