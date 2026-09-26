#include "SolidSupportQuery.h"
#include <algorithm>
#include <climits>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
namespace {
struct Incidence { std::uint32_t node,solid; };
struct Identity { std::uint64_t id; std::uint32_t row; };
}
std::size_t SolidSupportQueryBytes(std::size_t nodes,std::size_t solids) {
    if (!nodes || nodes>524288 || solids>16384)
        Reject(Status::ResourceLimit,"Solid support index extent exceeds source profile");
    // All simultaneous vectors, plus one query's complete matching roster.
    return 2*(nodes+1)*sizeof(std::uint32_t)+16*solids*(sizeof(Incidence)+sizeof(std::uint32_t))+
        2*solids*(sizeof(Identity)+sizeof(std::uint8_t)+sizeof(std::size_t))+sizeof(SolidSupportQueryIndex)+4096;
}
SolidSupportQueryIndex PrepareSolidSupportQueries(const coated::Inputs& input,
    tl::util::ConstView<std::uint8_t> emitted,std::size_t byte_cap) {
    const auto required=SolidSupportQueryBytes(input.nodes.size(),input.solids.size());
    if (!byte_cap || byte_cap>(std::size_t{64}<<20) || required>byte_cap)
        Reject(Status::ResourceLimit,"Solid support index exceeds admitted byte cap");
    Require(emitted.size()==input.solids.size() && (!emitted.size() || emitted.data()),
        "Solid support requires complete emitted-solid flags");
    std::vector<Incidence> incidence;
    std::vector<Identity> ids;
    incidence.reserve(8*input.solids.size());
    ids.reserve(input.solids.size());
    SolidSupportQueryIndex result;
    result.source_solids=input.solids.data();
    result.solid_count=input.solids.size();
    result.node_count=input.nodes.size();
    result.emitted.reserve(input.solids.size());
    for (std::size_t i=0;i<input.solids.size();++i) {
        const auto& row=input.solids[i];
        Require(row.phase==coated::PacketPhase::ReaderBeforeInitia && row.source_id && row.source_id<=INT_MAX &&
            (row.kind==coated::ReaderKind::Hex8 || row.kind==coated::ReaderKind::DeclaredPenta6),
            "Solid support requires native integer sourceEID and reader EightSlot packet");
        Require(emitted[i]<=1,"Solid support flag union must contain exact zero/one values");
        ids.push_back({row.source_id,std::uint32_t(i)});
        result.emitted.push_back(emitted[i]);
        if (row.kind==coated::ReaderKind::DeclaredPenta6) {
            Require(row.nodes[3]==row.nodes[0] && row.nodes[7]==row.nodes[4],
                "Solid support PENTA packet differs from reader raw8 phase");
            constexpr unsigned active[]{0,1,2,4,5,6};
            for (unsigned a=0;a<6;++a)
                for (unsigned b=0;b<a;++b)
                    Require(row.nodes[active[a]]!=row.nodes[active[b]],"Solid support PENTA repeats an active source slot");
        }
        for (unsigned k=0;k<8;++k) {
            const auto node=row.nodes[k];
            Require(node<input.nodes.size(),"Solid support raw node is outside source domain");
            // INSOL3D's TAGELEMS deduplicates a solid reached at several raw
            // corners. Retain raw8 source rows, index only that unique membership.
            if (std::find(row.nodes.begin(),row.nodes.begin()+k,node)==row.nodes.begin()+k)
                incidence.push_back({node,std::uint32_t(i)});
        }
    }
    std::sort(ids.begin(),ids.end(),[](auto a,auto b){return a.id<b.id;});
    for (std::size_t i=1;i<ids.size();++i)
        Require(ids[i-1].id!=ids[i].id,"Solid support source EIDs are not unique");
    std::sort(incidence.begin(),incidence.end(),[](auto a,auto b){
        return a.node<b.node || (a.node==b.node && a.solid<b.solid);
    });
    result.offsets.assign(input.nodes.size()+1,0);
    result.rows.reserve(incidence.size());
    for (auto value:incidence) {
        ++result.offsets[value.node+1];
        result.rows.push_back(value.solid);
    }
    for (std::size_t i=1;i<result.offsets.size();++i) result.offsets[i]+=result.offsets[i-1];
    if (ids.capacity()>2*input.solids.size() || incidence.capacity()>16*input.solids.size() ||
        result.emitted.capacity()>2*input.solids.size() || result.rows.capacity()>16*input.solids.size() ||
        result.offsets.capacity()>2*(input.nodes.size()+1))
        Reject(Status::ResourceLimit,"Solid support index allocation exceeds reservation");
    return result;
}
SolidSupportSelection QuerySolidSupport(const coated::Inputs& input,const SolidSupportQueryIndex& index,
    const s::Main& face) {
    Require(index.source_solids==input.solids.data() && index.solid_count==input.solids.size() &&
        index.node_count==input.nodes.size() && index.offsets.size()==input.nodes.size()+1 &&
        index.emitted.size()==input.solids.size(),"Solid support query has foreign or incomplete source backing");
    const unsigned arity=face.nodes[2]==face.nodes[3]?3:4;
    for (unsigned k=0;k<4;++k) Require(face.nodes[k]<input.nodes.size(),"Solid support query node is outside domain");
    for (unsigned k=0;k<arity;++k)
        for (unsigned j=0;j<k;++j) Require(face.nodes[k]!=face.nodes[j],"Solid support query repeats an active face node");
    const auto first=index.offsets[face.nodes[0]],last=index.offsets[face.nodes[0]+1];
    Require(first<=last && last<=index.rows.size(),"Solid support incidence range is invalid");
    SolidSupportSelection result;
    for (auto at=first;at<last;++at) {
        const auto row=index.rows[at];
        Require(row<input.solids.size(),"Solid support incidence has an invalid physical row");
        const auto& solid=input.solids[row];
        bool all=true;
        for (const auto node:face.nodes)
            all=all && std::find(solid.nodes.begin(),solid.nodes.end(),node)!=solid.nodes.end();
        if (all) result.matches.push_back(row);
    }
    if (result.matches.capacity()>2*input.solids.size())
        Reject(Status::ResourceLimit,"Solid matching roster exceeds admitted capacity");
    if (result.matches.empty()) return result;
    if (result.matches.size()>2) {
        // Original source uses the first two native incidences and warns.
        // No representative order is promoted to that owner pair.
        result.state=SolidSupportState::NeedsNativeReaderOrder;
        return result;
    }
    result.first=result.matches.front();
    if (result.matches.size()==2) {
        result.second=result.matches.back();
        if (input.solids[result.first].source_id<input.solids[result.second].source_id)
            std::swap(result.first,result.second);
        // Exact NTY25 flag branches:11 retains an internal pair;01 selects
        // the flagged second;10 and00 retain the maximum-EID first only.
        const auto a=index.emitted[result.first],b=index.emitted[result.second];
        if (!a && b) result.first=result.second;
        if (!a || !b) result.second=SIZE_MAX;
    }
    result.state=SolidSupportState::Resolved;
    result.exterior_orientation=result.second==SIZE_MAX;
    return result;
}
}
