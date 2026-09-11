#include "Fields.h"
#include <algorithm>

namespace crash::modelio::vehicle::rigid_part::point_mass {
struct Source::Storage {
    explicit Storage(const RigidPartSource& value):source(value) {}
    RigidPartSource source;
    Data data;
};
std::size_t Source::Forecast(const RigidPartSource& source,Limits limits) {
    const Limits hard;
    output::Require(limits.records && limits.records<=hard.records &&
        limits.blocks && limits.blocks<=hard.blocks && limits.host_bytes && limits.host_bytes<=hard.host_bytes,
        "Invalid original rigid point-mass source capacity");
    const auto& input=source.data();
    output::Require(input.sources.size()<=limits.blocks,"Rigid source block cap exceeded");
    std::size_t cards=0;
    for(const auto& block:input.sources) if(block.block.keyword=="*ELEMENT_MASS") {
        output::Require(block.cards.size()<=limits.records-cards,"Rigid point-mass card cap exceeded");
        cards+=block.cards.size();
    }
    std::size_t bytes=input.startup_budget_bytes;
    auto add=[&](std::size_t count,std::size_t width) {
        output::Require(bytes<=limits.host_bytes && count<=(limits.host_bytes-bytes)/width,
                        "Complete retained rigid point-mass source exceeds cap");
        bytes+=count*width;
    };
    add(1,sizeof(Source)+sizeof(Storage)+64);
    add(cards,sizeof(Record)+sizeof(Consumed)+128); // Owned rows plus set/sort scratch.
    return bytes;
}
Source Source::Prepare(const RigidPartSource& source,Limits limits) {
    const auto bytes=Forecast(source,limits);
    auto next=std::make_shared<Storage>(source);
    auto& data=next->data;
    const auto scale=source.source().canonical().data().inputs.units.mass_to_kg;
    data.records=detail::Read(source.data().sources,scale,limits);
    data.consumed.reserve(data.records.size());
    std::size_t part_rows=0;
    for(std::size_t i=0;i<data.records.size();++i) {
        const auto node=data.records[i].value.source_node_id;
        const auto root=source.root_for_node(node);
        if(root==SIZE_MAX) {++data.outside_records;continue;}
        for(const auto& body:source.data().bodies)
            if(std::binary_search(body.part_nodes.begin(),body.part_nodes.end(),node)) ++part_rows;
        data.consumed.push_back({i,root});
    }
    // This factory's immutable source authority is the exact original source
    // profile, not a generic material/parameter importer. Check its full census.
    output::Require(data.records.size()==155 && data.consumed.size()==54 &&
        data.outside_records==101 && part_rows==0,
        "Original rigid point-mass coverage changed");
    data.startup_budget_bytes=bytes;
    return Source(std::move(next));
}
const RigidPartSource& Source::rigid_source() const noexcept {return storage_->source;}
const Data& Source::data() const noexcept {return storage_->data;}
} // namespace crash::modelio::vehicle::rigid_part::point_mass
