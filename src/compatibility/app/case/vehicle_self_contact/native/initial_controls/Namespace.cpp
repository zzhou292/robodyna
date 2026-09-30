#include "Internal.h"
#include "PopulationProfile.h"
#include "modelio/tied_shell/Internal.h"
#include "modelio/source_assembly/JsonReader.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <climits>
#include <functional>
#include <map>
#include <set>
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
namespace {
namespace reader=modelio::assembly::reader;
namespace tied=modelio::tied_shell;
struct File {
    std::vector<tied::SourceEvidence> records;
    std::vector<std::pair<std::string,bool>> children;
    bool contains_contact=false;
    std::size_t parts=0,plain_bodies=0,rigid_walls=0,transform_cards=0,node_lines=0,transform_uses=0;
};
bool Contact(const std::string& keyword) {return keyword.rfind("*CONTACT",0)==0;}
bool SameBlock(const modelio::assembly::SourceBlock& a,const modelio::assembly::SourceBlock& b) {
    return a.filename==b.filename&&a.keyword==b.keyword&&a.sha256==b.sha256&&a.first_line==b.first_line&&a.last_line==b.last_line;
}
File Read(const ids::Member& member,const output::Value& metadata,Limits limits,std::size_t& checked) {
    // ImportContext already verified every block and the complete include graph.
    // Reuse its ordinary card reader for only the consumed interface/include rows.
    Require(member.bytes.find("$HC_ID")==std::string_view::npos&&member.bytes.find("$HMNAME")==std::string_view::npos,
        "CONTACT comment-derived native IDs require a separate source policy");
    tied::detail::Requests requests;File file;
    for(const auto& block:reader::Array(metadata,"blocks",limits.blocks,1).GetArray()) {
        const auto key=reader::Text(block,"keyword");
        AdmitPopulationKeyword(key,member.filename,reader::Unsigned(block,"first_line"));
        Require(checked<limits.blocks,"Complete source block census exceeds initializer cap");++checked;
        if(key=="*NODE") {
            const auto first=reader::Unsigned(block,"first_line"),last=reader::Unsigned(block,"last_line");
            Require(last>=first&&last-first<=std::size_t(INT_MAX)-file.node_lines,"Explicit source node line bound overflows");
            file.node_lines+=last-first;
        }
        file.transform_uses+=key=="*INCLUDE_TRANSFORM";
        file.parts+=key=="*PART";
        file.plain_bodies+=key=="*CONSTRAINED_NODAL_RIGID_BODY";
        file.rigid_walls+=key=="*RIGIDWALL_PLANAR"||key=="*RIGIDWALL_PLANAR_FINITE_ID"||key=="*RIGIDWALL_PLANAR_FINITE_FORCES_ID";
        if(!Contact(key)&&key!="*INCLUDE"&&key!="*INCLUDE_TRANSFORM"&&key!="*DEFINE_TRANSFORMATION")continue;
        if(Contact(key)&&key!="*CONTACT_AUTOMATIC_SINGLE_SURFACE"&&key!="*CONTACT_TIED_SHELL_EDGE_TO_SURFACE"&&key!="*CONTACT_INTERIOR")
            Reject(Status::UnsupportedSource,"Explicit-CID or unknown CONTACT source changes the native namespace",member.filename);
        tied::detail::Request request;
        request.evidence.block={member.filename,key,{},reader::Text(block,"source_block_sha256"),
            reader::Unsigned(block,"first_line"),reader::Unsigned(block,"last_line")};
        Require(requests.emplace(request.evidence.block.first_line,std::move(request)).second,"Repeated interface source block");
    }
    tied::Limits source_limits;source_limits.blocks=limits.blocks;source_limits.metadata_bytes=limits.metadata_bytes;
    file.records=tied::detail::ReadRequestedSources(requests,std::string(member.bytes),source_limits);
    for(const auto& record:file.records) {
        file.contains_contact|=Contact(record.block.keyword);
        if(record.block.keyword=="*DEFINE_TRANSFORMATION")file.transform_cards+=record.cards.size();
        if(record.block.keyword=="*INCLUDE"||record.block.keyword=="*INCLUDE_TRANSFORM") {
            Require(!record.cards.empty(),"Missing authenticated include target");
            file.children.emplace_back(reader::auxiliary::Trim(record.cards[0].second),record.block.keyword=="*INCLUDE_TRANSFORM");
        }
    }
    return file;
}
}
Namespace ResolveNamespace(const MainSource& main,const ids::ImportContext& imported,const ids::ImportMembers& members,
    const Wall* wall,Limits limits) {
    Require(imported.data().profile==ids::Profile::DirectKeywordR14FreshRadiossPoSortById&&
        imported.data().diagnostic.status==ids::Readiness::Ready&&imported.data().source_digest==main.provenance().source_digest,
        "CONTACT namespace needs the authenticated fresh direct PO import");
    output::Document doc;
    const auto& canonical=imported.canonical().data();
    doc.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag|rapidjson::kParseValidateEncodingFlag>(
        canonical.canonical_bytes.data(),canonical.canonical_bytes.size());
    Require(!doc.HasParseError()&&doc.IsObject(),"Missing complete CONTACT inventory");reader::UniqueKeys(doc);
    const auto& inventory=reader::Member(doc,"source_files");
    Namespace result;std::map<std::string,File> files;
    for(const auto& member:members.members)
        Require(files.emplace(member.filename,Read(member,reader::Member(inventory,member.filename.c_str()),limits,result.checked_blocks)).second,
            "Repeated CONTACT source member");
    std::set<std::string> visited;std::vector<std::string> file_order;
    // First encounter allocates source file indices. ReadKeyword later visits
    // component/file/position, not a flattened recursive CONTACT occurrence list.
    // No transformed CONTACT is admitted, so all consumed rows share component0.
    std::function<void(const std::string&,bool)> visit=[&](const std::string& name,bool transformed) {
        const auto found=files.find(name);
        Require(found!=files.end()&&visited.insert(name).second,"CONTACT include graph is incomplete or cyclic");
        if(transformed&&found->second.contains_contact)
            Reject(Status::UnsupportedSource,"Transformed CONTACT ID offsets/order are not admitted",name);
        file_order.push_back(name);
        for(const auto& child:found->second.children)visit(child.first,transformed||child.second);
    };
    visit(members.entry_member,false);Require(visited.size()==files.size(),"CONTACT source graph has unreachable members");
    auto& range=result.population;
    range.lower=main.startup_input().node_count+(wall?4:0);
    range.complete_original_nodes=canonical.canonical_nodes;
    // Original complete node domain is a conservative superset of the declared
    // retained physical domain. Bound possible native generated nodes by the
    // complete generator census, never a selected/contact-only count.
    std::size_t transform_uses=1; // Conservative independent pass plus each admitted include reference.
    for(const auto& [name,file]:files)transform_uses+=file.transform_uses;
    for(const auto& [name,file]:files) {
        range.explicit_node_bound+=file.node_lines;
        range.rigid_definition_bound+=2*(file.parts+file.plain_bodies);
        range.transform_bound+=file.transform_cards*transform_uses;
        range.rigid_wall_bound+=3*file.rigid_walls;
    }
    for(const auto& row:imported.data().precursors)
        range.discrete_bound+=row.kind==ids::SourceKind::DiscreteNamespaceOnly;
    range.upper=range.explicit_node_bound+(wall?4:0)+range.rigid_definition_bound+
        range.discrete_bound+range.transform_bound+range.rigid_wall_bound;
    Require(range.lower&&range.lower<=range.upper&&range.upper<=1500000&&range.upper<=INT_MAX,
        "Complete source population interval does not prove one exact base-multiplier branch");
    const auto& selected=main.mixed().initial().selection().data().sources.at(0).block;
    const auto& tied=main.gap_operands().corrected().pre_correction().physical().source_domain().source().tied_source().data();
    Require(tied.contact_source<tied.sources.size(),"Actual tied source contact is unavailable");
    std::size_t type2=0,type25=0,interior=0;
    for(const auto& name:file_order)for(const auto& source:files.at(name).records) {
        const auto& key=source.block.keyword;
        if(key=="*CONTACT_INTERIOR"){++interior;continue;}
        if(!Contact(key))continue;
        Require(result.interfaces.size()<limits.interfaces&&result.interfaces.size()<std::size_t(INT_MAX),
            "Native interface namespace exceeds capacity or native integer range");
        // Plain cards enter PO with ID0; stable source SortById preserves the
        // reader table order. Fresh Radioss IdManager starts1 and AddId advances
        // after each real creation. The qualified complete original context
        // excludes prior INTER creators and applicable INTERIOR TYPE7 output.
        const auto id=std::uint64_t(result.interfaces.size()+1);
        const bool is_tied=key=="*CONTACT_TIED_SHELL_EDGE_TO_SURFACE";
        if(is_tied) {
            Require(SameBlock(source.block,tied.sources[tied.contact_source].block),"CONTACT tied identity differs from actual source declaration");
            ++type2;
        } else {
            Require(SameBlock(source.block,selected),"CONTACT self identity differs from actual original selection");
            result.self=id;++type25;
        }
        result.interfaces.push_back({is_tied?InterfaceKind::Type2:InterfaceKind::Type25,
            InterfaceOrigin::OriginalDefinition,id,std::uint32_t(id),source.block});
    }
    const auto& census=main.gap_operands().corrected().provenance().interfaces;
    Require(type2==census.type2_sources&&type25==census.type25_sources&&interior==census.interior_sources&&result.self,
        "Complete original CONTACT table differs from the qualified interface census");
    if(wall) {
        Require(result.interfaces.size()<limits.interfaces&&wall->ids().interface>result.interfaces.size()&&wall->ids().interface<=INT_MAX,
            "Declared additional wall interface collides with original native namespace");
        result.wall=wall->ids().interface;
        // Explicit positive added ID is greater than the complete original pool.
        // It is converted/declared after the original zero-ID CONTACT records;
        // no native preload is silently allowed to shift their generated IDs.
        modelio::assembly::SourceBlock source;
        source.keyword="DECLARED_ENVELOPE_TYPE25";source.sha256=wall->digest();
        result.interfaces.push_back({InterfaceKind::Type25,InterfaceOrigin::DeclaredAdditionalInterface,result.wall,
            std::uint32_t(result.interfaces.size()+1),std::move(source)});
    }
    return result;
}
}
