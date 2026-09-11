#pragma once
#include "../Internal.h"
#include "../../tied_post_kinchk/tests/Fixture.h"
namespace crash::cases::vehicle_startup::cin_test {
struct Fixture : post_kinchk_test::Fixture {
    native_search::PostKinChkResult post;
    tl::fea::NodalNodeDomain domain;
    Fixture() {
        const auto packed = post_kinchk_detail::Pack(classified,receipt);
        if (!native_search::PostKinChk(packed.View(classified,receipt),&post)) throw std::runtime_error("Tiny KINET failed");
        const auto& ids_array = tied::source::FindArray(canonical,"node_ids");
        const auto& x_array = tied::source::FindArray(canonical,"node_positions");
        const auto ids = output::arrays::Decode<tied::SourceId>(ids_array.descriptor,ids_array.bytes);
        const auto x = output::arrays::Decode<double>(x_array.descriptor,x_array.bytes);
        std::vector<tl::fea::NodalDomainNode> nodes;
        for (std::size_t row = 0; row < ids.size(); ++row)
            nodes.push_back({ids[row],{x[3*row],x[3*row+1],x[3*row+2]}});
        std::reverse(nodes.begin(),nodes.end());
        if (!domain.Initialize({post.source_instance_id(),nodes.data(),nodes.size()})) throw std::runtime_error("Tiny domain failed");
    }
    tied_cin_detail::Inputs Packed() const {
        return tied_cin_detail::Pack(canonical,declaration,packing,geometry,*finalized.data(),classified,post);
    }
};
}
