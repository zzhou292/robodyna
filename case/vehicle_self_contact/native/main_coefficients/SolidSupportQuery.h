#pragma once
#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
enum class SolidSupportState { Absent, Resolved, NeedsNativeReaderOrder };
struct SolidSupportSelection {
    SolidSupportState state=SolidSupportState::Absent;
    // Complete unique matching physical indices in representative order; never
    // claimed to be native NOD2ELS order. Resolve<=2 by sourceEID and flag union.
    std::vector<std::size_t> matches;
    std::size_t first=SIZE_MAX,second=SIZE_MAX;
    bool exterior_orientation=false; // Effective native IC1, after flag filtering.
};
struct SolidSupportQueryIndex {
    std::vector<std::uint32_t> offsets,rows;
    std::vector<std::uint8_t> emitted;
    const coated::Solid* source_solids=nullptr;
    std::size_t solid_count=0,node_count=0;
};
// Inputs must remain alive/immutable for the index lifetime. Pointer/count
// binding does not detect mutation through aliases or authenticate a fake source.
std::size_t SolidSupportQueryBytes(std::size_t nodes,std::size_t solids);
SolidSupportQueryIndex PrepareSolidSupportQueries(const coated::Inputs&,
    tl::util::ConstView<std::uint8_t> complete_emitted_flags,
    std::size_t byte_cap=std::size_t{64}<<20);
SolidSupportSelection QuerySolidSupport(const coated::Inputs&,const SolidSupportQueryIndex&,
    const s::Main& post_sides_face);
}
