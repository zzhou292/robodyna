#include "Mapping.h"
#include "lib_utils/BoundedArena.h"
namespace crash::output::physical_frames {
struct Mapping::Data {
    Data(const Execution& e,source::PreparedSourceMapping m):execution(e),mapping(std::move(m)) {}
    Execution execution;
    source::PreparedSourceMapping mapping;
    std::vector<std::uint32_t> nodes;
    std::vector<ParentField> parents;
    std::size_t bytes=0;
};
Mapping Mapping::Prepare(const Execution& execution,std::size_t cap) {
    const auto& native=execution.execution();
    const auto& resolution=execution.resolution();
    const auto& canonical=resolution.source().canonical();
    const auto& canonical_nodes=resolution.source().canonical_nodes();
    const auto* map=execution.physical().mapping();
    Require(cap && cap<=512u<<20 && native.prepared() && map &&
        map->Matches(*execution.physical().shells(),*execution.physical().domain()) &&
        canonical_nodes.size()==map->shell_node_count() && native.parents().size()==resolution.parents().size(),
        "Physical accepted mapping has incomplete source scope");
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion unused;
    Require(budget.Append<std::byte>(sizeof(Data)+8192,unused) &&
        budget.Append<std::byte>(canonical.data().limits.host_bytes,unused) &&
        budget.Append<std::uint64_t>(canonical.data().canonical_nodes,unused) &&
        budget.Append<std::uint32_t>(canonical_nodes.size(),unused) &&
        budget.Append<ParentField>(native.parents().size(),unused) &&
        budget.Append<source::NativeParent>(native.parents().size(),unused),"Accepted mapping scratch exceeds cap");
    std::vector<source::NativeParent> rows;
    rows.reserve(native.parents().size());
    std::vector<ParentField> fields;
    fields.reserve(native.parents().size());
    for(std::size_t i=0;i<native.parents().size();++i) {
        const auto& role=native.parents()[i];
        const auto& raw=resolution.parents()[i];
        const auto* mapping=resolution.native_mapping(i);
        Require(mapping && mapping->family==role.source.family && mapping->family_index==role.source.family_index &&
            mapping->family_index<=UINT32_MAX && raw.source_parent_id==role.source.source_parent_id,
            "Accepted parent/source family identity differs");
        const auto family=Family(mapping->family);
        rows.push_back({raw.canonical_parent,family,static_cast<std::uint32_t>(mapping->family_index),
                        role.material_points,Plasticity(role.law)});
        fields.push_back({family,static_cast<std::uint32_t>(mapping->family_index),role.law});
    }
    auto prepared=source::PreparedSourceMapping::Prepare(canonical,
        {canonical_nodes.data(),canonical_nodes.size(),rows.data(),rows.size()});
    // The source mapping factory already enforces its complete child reservation
    // charged above; its temporary arrays retire before the node-ID decode.
    auto next=std::make_shared<Data>(execution,std::move(prepared));
    next->parents=std::move(fields);
    next->nodes.reserve(canonical_nodes.size());
    const auto& ids=source::FindArray(canonical.data(),"node_ids");
    const auto original=arrays::Decode<std::uint64_t>(ids.descriptor,ids.bytes);
    const auto shell_nodes=execution.physical().shells()->active_nodes();
    for(std::size_t i=0;i<canonical_nodes.size();++i) {
        const auto physical=map->owner_index(i);
        Require(canonical_nodes[i]<original.size() && physical<map->owner_node_count() && physical<=UINT32_MAX &&
            original[canonical_nodes[i]]==shell_nodes[i].source_id &&
            execution.physical().domain()->nodes()[physical].source_id==shell_nodes[i].source_id,
            "Original render node and physical source node identity differ");
        next->nodes.push_back(static_cast<std::uint32_t>(physical));
    }
    next->bytes=sizeof(Data)+next->nodes.capacity()*sizeof(std::uint32_t)+
        next->parents.capacity()*sizeof(ParentField)+next->mapping.payload_bytes()+
        next->mapping.parents().capacity()*sizeof(records::ParentPoints)+8192;
    Require(next->bytes<=cap,"Retained accepted source mapping exceeds cap");
    return Mapping(std::move(next));
}
const Execution& Mapping::execution() const noexcept {return data_->execution;}
const source::PreparedSourceMapping& Mapping::source_mapping() const noexcept {return data_->mapping;}
const std::vector<std::uint32_t>& Mapping::physical_nodes() const noexcept {return data_->nodes;}
const std::vector<ParentField>& Mapping::parents() const noexcept {return data_->parents;}
std::size_t Mapping::physical_node_count() const noexcept {return data_->execution.physical().domain()->node_count();}
std::size_t Mapping::payload_bytes() const noexcept {return data_->bytes;}
} // namespace crash::output::physical_frames
