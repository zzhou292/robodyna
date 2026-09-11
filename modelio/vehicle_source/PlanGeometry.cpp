#include "Internal.h"
#include <algorithm>

namespace crash::modelio::vehicle::detail {
namespace {
template<class T> std::vector<T> Decode(const source::CanonicalData& d,const char* name) {
    const auto& array=source::FindArray(d,name);return output::arrays::Decode<T>(array.descriptor,array.bytes);
}
}
Geometry ReadGeometry(const source::CanonicalData& d,const std::vector<PartDisposition>& parts) {
    const auto records=Decode<std::uint64_t>(d,"shells_records");
    const auto conn=Decode<std::uint32_t>(d,"shells_node_indices");
    const auto ids=Decode<std::uint64_t>(d,"node_ids");
    Geometry next;next.parents.reserve(d.retained_shells);next.nodes.reserve(d.retained_nodes);
    std::vector<std::size_t> count(parts.size(),0);std::vector<unsigned char> used(d.canonical_nodes,0);
    for(std::size_t i=0;i<d.canonical_shells;++i) {
        const auto pid=records[6*i+1];
        const auto found=std::lower_bound(parts.begin(),parts.end(),pid,[](const auto& p,std::uint64_t id){return p.part_id<id;});
        if(found==parts.end()||found->part_id!=pid)continue; // Exactly the source's explicit tire exclusion.
        const auto p=static_cast<std::size_t>(found-parts.begin());++count[p];
        next.parents.push_back({static_cast<std::uint32_t>(i),static_cast<std::uint32_t>(p)});
        const bool quad=records[6*i+4]!=records[6*i+5];++(quad?next.counts.q4:next.counts.t3);
        if(found->status==Disposition::SupportedDeclaration)++next.counts.supported_parents;
        for(unsigned local=0;local<4;++local)used[conn[4*i+local]]=1;
    }
    for(std::size_t p=0;p<parts.size();++p) {
        Require(count[p]==parts[p].shell_count,"Late vehicle parent coverage mismatch");
        if(parts[p].status==Disposition::SupportedDeclaration)++next.counts.supported_parts;
    }
    for(std::size_t n=0;n<used.size();++n)if(used[n])next.nodes.push_back(static_cast<std::uint32_t>(n));
    std::sort(next.nodes.begin(),next.nodes.end(),[&](auto a,auto b){return ids[a]<ids[b];});
    next.counts.parts=parts.size();next.counts.parents=next.parents.size();next.counts.nodes=next.nodes.size();
    Require(next.counts.parents==d.retained_shells&&next.counts.nodes==d.retained_nodes&&
        next.counts.q4==d.retained_q4&&next.counts.t3==d.retained_t3,"Complete vehicle canonical coverage changed");
    return next;
}
} // namespace crash::modelio::vehicle::detail
