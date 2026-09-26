#pragma once
#include "../SolidSupportQuery.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
struct NativeSolidSupportResult {
    std::vector<std::size_t> native_matches;
    std::size_t chosen=SIZE_MAX;
    int raw_owner_words[2]{}; // Source row1-based; prior seed retained when no match.
    std::uint32_t final_nodes[4]{};
    unsigned effective_count=0,inversions=0;
    bool warning=false,geometry_defined=false;
    double area=0;
};
NativeSolidSupportResult NativeSolidSupport(const coated::Inputs&,
    const std::vector<std::uint8_t>& flags,const coated::s::Main&,
    const std::array<int,2>& prior={0,0});
}
