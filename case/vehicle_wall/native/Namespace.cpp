#include "Internal.h"
#include "case/vehicle_self_contact/native/TopologyDigestFields.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <map>
#include <set>
#include <tuple>

namespace crash::cases::vehicle_wall::native::detail {
namespace read=modelio::assembly::reader;
namespace aux=read::auxiliary;
namespace fields=vehicle_self_contact::native::detail::digest;
namespace {
enum class Kind : std::uint32_t {
    Node, Shell, Solid, Beam, Mass, Discrete, Seatbelt, Part, Material, Section,
    NodeSet, PartSet, SegmentSet, Curve, Frame, Transform, RigidWall, Airbag, RigidBody, Weld, Joint, Other
};
const char* KindName(Kind kind) {
    static constexpr const char* names[]{"NODE","SHELL","SOLID","BEAM","MASS","DISCRETE","SEATBELT_ACCELEROMETER",
        "PART","MATERIAL","SECTION","NODE_SET","PART_SET","SEGMENT_SET","CURVE","FRAME","TRANSFORM",
        "RIGID_WALL","AIRBAG","RIGID_BODY","WELD","JOINT","OTHER"};
    return names[static_cast<unsigned>(kind)];
}
struct Entry {std::uint64_t id=0;Kind kind=Kind::Other;std::uint32_t member=0;std::size_t row=0;};
struct Spec {Kind kind;unsigned card=0,width=10;bool rows=false,canonical=false;};
std::optional<Spec> Definition(const std::string& key) {
    if(key=="*NODE")return Spec{Kind::Node,0,8,true,true};
    if(key=="*ELEMENT_SHELL")return Spec{Kind::Shell,0,8,true,true};
    if(key=="*ELEMENT_SOLID")return Spec{Kind::Solid,0,8,true,true};
    if(key=="*ELEMENT_BEAM")return Spec{Kind::Beam,0,8,true,true};
    if(key=="*ELEMENT_MASS")return Spec{Kind::Mass,0,8,true,false};
    if(key=="*ELEMENT_DISCRETE")return Spec{Kind::Discrete,0,8,true,false};
    // Keyword971 ACCELEROMETER format uses10-column SBACID, unlike raw8 elements.
    if(key=="*ELEMENT_SEATBELT_ACCELEROMETER")return Spec{Kind::Seatbelt,0,10,true,false};
    if(key=="*PART")return Spec{Kind::Part,1};
    if(key=="*SECTION_SHELL"||key=="*SECTION_SOLID"||key=="*SECTION_BEAM"||key=="*SECTION_DISCRETE")
        return Spec{Kind::Section};
    static const std::set<std::string> materials{
        "*MAT_BLATZ-KO_RUBBER","*MAT_DAMPER_VISCOUS","*MAT_ELASTIC","*MAT_LOW_DENSITY_FOAM",
        "*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY","*MAT_PIECEWISE_LINEAR_PLASTICITY",
        "*MAT_RIGID","*MAT_SPOTWELD","*MAT_SPRING_ELASTIC","*MAT_SPRING_NONLINEAR_ELASTIC"};
    if(materials.count(key))return Spec{Kind::Material};
    if(key=="*SET_NODE_GENERAL"||key=="*SET_NODE_ADD"||key=="*SET_NODE_LIST")return Spec{Kind::NodeSet};
    if(key=="*SET_NODE_LIST_TITLE")return Spec{Kind::NodeSet,1};
    if(key=="*SET_PART_ADD"||key=="*SET_PART_LIST")return Spec{Kind::PartSet};
    if(key=="*SET_PART_LIST_TITLE")return Spec{Kind::PartSet,1};
    if(key=="*SET_SEGMENT")return Spec{Kind::SegmentSet};
    if(key=="*DEFINE_CURVE")return Spec{Kind::Curve};
    if(key=="*DEFINE_COORDINATE_NODES")return Spec{Kind::Frame};
    if(key=="*DEFINE_TRANSFORMATION")return Spec{Kind::Transform};
    if(key=="*RIGIDWALL_PLANAR_FINITE_FORCES_ID"||key=="*RIGIDWALL_PLANAR_FINITE_ID")return Spec{Kind::RigidWall};
    if(key=="*AIRBAG_SIMPLE_AIRBAG_MODEL_ID")return Spec{Kind::Airbag};
    if(key=="*HOURGLASS")return Spec{Kind::Other};
    if(key=="*CONSTRAINED_NODAL_RIGID_BODY")return Spec{Kind::RigidBody};
    static const std::set<std::string> nondefinitions{
        "*KEYWORD","*END","*TITLE","*INCLUDE","*INCLUDE_TRANSFORM","*PARAMETER","*PARAMETER_EXPRESSION",
        "*CONTACT_AUTOMATIC_SINGLE_SURFACE","*CONTACT_INTERIOR","*CONTACT_TIED_SHELL_EDGE_TO_SURFACE",
        "*CONSTRAINED_EXTRA_NODES_SET","*CONSTRAINED_RIGID_BODIES",
        "*CONSTRAINED_SPOTWELD_ID","*CONSTRAINED_JOINT_CYLINDRICAL_ID","*CONSTRAINED_JOINT_REVOLUTE_ID",
        "*CONSTRAINED_JOINT_SPHERICAL_ID","*ELEMENT_MASS_PART","*INITIAL_VELOCITY_GENERATION",
        "*LOAD_BODY_Z","*RIGIDWALL_PLANAR",
        "*CONTROL_ACCURACY","*CONTROL_CONTACT","*CONTROL_CPU","*CONTROL_ENERGY","*CONTROL_HOURGLASS",
        "*CONTROL_OUTPUT","*CONTROL_SHELL","*CONTROL_SOLID","*CONTROL_TERMINATION","*CONTROL_TIMESTEP",
        "*DATABASE_ABSTAT","*DATABASE_BINARY_D3PLOT","*DATABASE_BINARY_D3THDT","*DATABASE_BINARY_INTFOR",
        "*DATABASE_DEFORC","*DATABASE_ELOUT","*DATABASE_EXTENT_BINARY","*DATABASE_GLSTAT","*DATABASE_JNTFORC",
        "*DATABASE_MATSUM","*DATABASE_NODOUT","*DATABASE_RCFORC","*DATABASE_RWFORC","*DATABASE_SECFORC",
        "*DATABASE_SLEOUT","*DATABASE_HISTORY_NODE_ID","*DATABASE_HISTORY_NODE_SET_LOCAL"};
    if(nondefinitions.count(key))return {};
    Reject(Status::UnsupportedSource,"Unresolved namespace semantics for source keyword: "+key);
}
std::uint64_t Id(const std::string& row,unsigned width,const std::string& file,std::size_t line) {
    const auto value=read::SourceScalar(row,0,width);
    if(!value||*value<=0||*value>INT_MAX||std::floor(*value)!=*value)
        Reject(Status::InvalidInput,"Source definition ID is not a positive native integer",file,line);
    return static_cast<std::uint64_t>(*value);
}
std::map<std::string,std::uint64_t> Offsets(const springs::ContextData& context) {
    std::map<std::string,std::uint64_t> result{{context.entry_member,0}};
    for(std::size_t pass=0;pass<context.members.size();++pass) {
        for(const auto& source:context.evidence) {
            if(source.block.keyword!="*INCLUDE"&&source.block.keyword!="*INCLUDE_TRANSFORM")continue;
            const auto parent=result.find(source.block.filename);if(parent==result.end())continue;
            Require(!source.cards.empty(),"Include namespace descriptor has no child");
            const auto child=aux::Trim(source.cards[0].second);
            std::uint64_t offset=0;
            if(source.block.keyword=="*INCLUDE_TRANSFORM") {
                Require(source.cards.size()==5,"Incomplete namespace include transform");
                const auto first=read::SourceScalar(source.cards[1].second,0);
                Require(first&&*first>=0&&*first<=INT_MAX&&std::floor(*first)==*first,
                    "Namespace offset must be a literal nonnegative native integer");
                offset=static_cast<std::uint64_t>(*first);
                for(unsigned column=1;column<7;++column)
                    Require(read::SourceScalar(source.cards[1].second,column).value_or(0)==double(offset),
                        "First wall namespace profile requires equal source ID offsets");
                Require(read::SourceScalar(source.cards[2].second,0).value_or(0)==double(offset),
                    "Rigid-wall namespace offset differs from the common ID offset");
                for(unsigned column=0;column<4;++column) {
                    const auto scale=read::SourceScalar(source.cards[3].second,column);
                    Require(!scale||*scale==1,"Scaled include source is outside the declared wall profile");
                }
            }
            Require(offset<=std::uint64_t(INT_MAX)-parent->second,"Nested namespace offset overflows native integers");
            offset+=parent->second;
            const auto inserted=result.emplace(child,offset);
            Require(inserted.second||inserted.first->second==offset,"Conflicting namespace include offset");
        }
    }
    Require(result.size()==context.members.size(),"Namespace closure did not reach every authenticated member");
    return result;
}
}
std::size_t NamespaceWorkspace(const source::CanonicalData& canonical,const springs::ImportMembers& input,Limits limits) {
    const auto count=canonical.canonical_nodes+canonical.canonical_shells+
        source::FindArray(canonical,"solids_records").descriptor.layout.rows+
        source::FindArray(canonical,"beams_records").descriptor.layout.rows+32768;
    if(count>limits.namespace_entries)Reject(Status::ResourceLimit,"Namespace definition count reservation exceeds cap");
    Require(canonical.canonical_bytes.size()<=limits.source_metadata_bytes,"Namespace source metadata exceeds cap");
    std::size_t largest=0,member=0;
    for(const auto* name:{"node_ids","shells_records","solids_records","beams_records"})
        largest=std::max(largest,source::FindArray(canonical,name).descriptor.bytes);
    for(const auto& value:input.members)member=std::max(member,value.bytes.size());
    std::size_t bytes=0;
    const auto add=[&](std::size_t n,std::size_t width) {
        Require(width&&n<=(SIZE_MAX-bytes)/width,"Namespace workspace arithmetic overflows");bytes+=n*width;
    };
    add(count,2*sizeof(Entry)); // Sorted records, including conservative capacity slack.
    add(canonical.canonical_bytes.capacity()+1,8); // Existing canonical DOM reservation convention.
    add(largest,2);add(member,1); // Sequential decode and current source extraction copy.
    add(limits.source_metadata_bytes,3); // Bounded retained cards/raw text/vector staging.
    add(fields::ChunkWords,2*sizeof(std::uint64_t));
    add(limits.metadata_bytes,2); // Chunk hashes/metadata, separately from fixed chunk scratch.
    if(bytes>limits.namespace_bytes)Reject(Status::ResourceLimit,"Complete namespace workspace exceeds cap");
    return bytes;
}
std::pair<NamespaceReport,AllocatedIds> Namespace(const springs::ImportContext& context,
        const springs::ImportMembers& input,const std::string& wall_sha,Limits limits) {
    return NamespaceValues(context.canonical().data(),context.data(),input,wall_sha,limits);
}
std::pair<NamespaceReport,AllocatedIds> NamespaceValues(const source::CanonicalData& canonical,
        const springs::ContextData& c,const springs::ImportMembers& input,
        const std::string& wall_sha,Limits limits) {
    if(c.diagnostic.status!=springs::Readiness::Ready)
        Reject(Status::UnsupportedSource,c.diagnostic.reason,c.diagnostic.file,c.diagnostic.line,c.diagnostic.source_id);
    Require(c.profile==springs::Profile::DirectKeywordR14FreshRadiossPoSortById,
        "Namespace requires the authenticated fresh PO direct-reader profile");
    (void)NamespaceWorkspace(canonical,input,limits);
    output::Document document;document.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(
        canonical.canonical_bytes.data(),canonical.canonical_bytes.size());
    Require(!document.HasParseError()&&document.IsObject(),"Invalid namespace source inventory");
    read::UniqueKeys(document);
    const auto offsets=Offsets(c);
    std::map<std::string,const springs::Member*> members;
    for(const auto& member:input.members)members.emplace(member.filename,&member);
    std::vector<Entry> entries;
    const auto bound=canonical.canonical_nodes+canonical.canonical_shells+
        source::FindArray(canonical,"solids_records").descriptor.layout.rows+
        source::FindArray(canonical,"beams_records").descriptor.layout.rows+32768;
    Require(bound<=limits.namespace_entries&&bound<=limits.namespace_bytes/(2*sizeof(Entry)),
        "Namespace entry reservation exceeds cap");
    entries.reserve(bound);
    Require(entries.capacity()<=2*bound,"Namespace vector capacity exceeds admitted reservation");
    const auto add=[&](std::uint64_t id,Kind kind,std::uint32_t member,std::size_t row,std::uint64_t offset) {
        if(!id||id>INT_MAX||offset>std::uint64_t(INT_MAX)-id)
            Reject(Status::Unrepresentable,"Declared namespace ID/offset exceeds native integer",{},row,id);
        if(entries.size()==bound)
            Reject(Status::ResourceLimit,"Complete namespace entry count exceeds cap");
        entries.push_back({id+offset,kind,member,row});
    };
    NamespaceReport result;result.members=c.members.size();result.source_digest=c.source_digest;
    const auto& inventory=read::Member(document,"source_files");
    std::uint32_t file_index=0;
    std::map<std::string,std::uint32_t> inventory_ordinals;
    for(const auto& file:inventory.GetObject()) {
        const std::string name(file.name.GetString(),file.name.GetStringLength());
        Require(inventory_ordinals.emplace(name,file_index).second,"Duplicate source inventory member");
        const auto found=members.find(name);Require(found!=members.end(),"Missing namespace source member");
        Require(output::Sha256(std::string(found->second->bytes))==read::Text(file.value,"sha256"),
            "Namespace member differs from authenticated closed context");
        const auto offset=offsets.at(name);
        const bool main=read::Text(file.value,"sha256")==canonical.inputs.source_member.sha256;
        const bool display=read::Text(file.value,"sha256")==wall_sha;
        modelio::tied_shell::detail::Requests requests;
        std::set<Kind> canonical_blocks;
        for(const auto& block:read::Array(file.value,"blocks",8192).GetArray()) {
            const auto key=read::Text(block,"keyword");
            const auto spec=Definition(key);if(!spec)continue;
            if(main&&spec->canonical) {
                Require(canonical_blocks.insert(spec->kind).second,"Repeated canonical geometry definition block");
                const char* array=spec->kind==Kind::Node?"node_ids":spec->kind==Kind::Shell?"shells_records":
                    spec->kind==Kind::Solid?"solids_records":"beams_records";
                const auto& data=source::FindArray(canonical,array);
                Require(data.descriptor.layout.rows==read::Unsigned(block,"data_records"),
                    "Canonical namespace array is not the complete original block");
                const auto values=output::arrays::Decode<std::uint64_t>(data.descriptor,data.bytes,
                    {canonical.limits.file_bytes,std::max(canonical.limits.nodes,canonical.limits.parents),64});
                for(std::size_t i=0;i<data.descriptor.layout.rows;++i)
                    add(values.at(i*data.descriptor.layout.columns),spec->kind,file_index,i,offset);
            } else {
                modelio::tied_shell::detail::Request request;
                const auto first=read::Unsigned(block,"first_line");
                request.evidence.block={name,key,{},read::Text(block,"source_block_sha256"),first,read::Unsigned(block,"last_line")};
                requests.emplace(first,std::move(request));
            }
        }
        modelio::tied_shell::Limits read_limits;read_limits.blocks=8192;read_limits.metadata_bytes=limits.source_metadata_bytes;
        const auto evidence=modelio::tied_shell::detail::ReadRequestedSources(requests,std::string(found->second->bytes),read_limits);
        for(const auto& source:evidence) {
            const auto spec=*Definition(source.block.keyword);
            Require(source.cards.size()>spec.card,"Missing namespace definition card");
            const auto last=spec.rows?source.cards.size():spec.card+1;
            for(std::size_t i=spec.card;i<last;++i) {
                if(aux::Trim(source.cards[i].second).empty())continue;
                add(Id(source.cards[i].second,spec.width,name,source.cards[i].first),spec.kind,file_index,
                    source.cards[i].first,offset);
            }
            if(display&&source.block.keyword=="*MAT_RIGID") {
                Require(result.display_material_keyword.empty(),"Ambiguous original display wall material provenance");
                result.display_material_keyword=source.block.keyword;
                result.display_material_block_sha256=source.block.sha256;
                result.display_density_native=read::RequiredScalar(source.cards[0].second,1);
                result.display_young_native=read::RequiredScalar(source.cards[0].second,2);
                result.display_poisson=read::RequiredScalar(source.cards[0].second,3);
            }
        }
        ++file_index;
    }
    Require(!result.display_material_keyword.empty(),"Original wall display material provenance is absent");
    // Reuse the already parsed complete generated-connection source rosters;
    // their alternating cards and converter ordering are not parsed again.
    for(const auto& row:c.welds)add(row.original_id,Kind::Weld,inventory_ordinals.at(row.location.file),row.location.line,0);
    for(const auto& row:c.joints)add(row.original_id,Kind::Joint,inventory_ordinals.at(row.location.file),row.location.line,0);
    std::sort(entries.begin(),entries.end(),[](const auto& a,const auto& b){return std::tie(a.kind,a.id)<std::tie(b.kind,b.id);});
    for(std::size_t i=0;i<entries.size();++i) {
        if(i&&entries[i].kind==entries[i-1].kind&&entries[i].id==entries[i-1].id) {
            const auto filename=[&](std::uint32_t ordinal) {
                std::uint32_t at=0;
                for(const auto& member:inventory.GetObject())if(at++==ordinal)
                    return std::string(member.name.GetString(),member.name.GetStringLength());
                return std::string("unknown_inventory_member");
            };
            const auto& first=entries[i-1];
            Reject(Status::IdentityMismatch,std::string("Duplicate ")+KindName(entries[i].kind)+
                " definition; first="+filename(first.member)+":"+std::to_string(first.row),
                filename(entries[i].member),entries[i].row,entries[i].id);
        }
        result.maximum_declared=std::max(result.maximum_declared,entries[i].id);
    }
    // Complete authenticated SPRING precursor/weld/joint census. This is only
    // its range envelope; physical source indices/mapping remain the existing
    // resolver's authority. The new elastic Q4 contributes no SPRING generator.
    for(const auto& row:c.precursors)result.maximum_spring=std::max(result.maximum_spring,row.original_id);
    const auto generated=c.welds.size()+c.joints.size();
    Require(generated<=std::size_t(INT_MAX-result.maximum_spring),"Generated SPRING namespace overflows");
    result.maximum_spring+=generated;
    result.allocation_ceiling=std::max(result.maximum_declared,result.maximum_spring);
    Require(result.allocation_ceiling<=std::uint64_t(INT_MAX)-11,"Wall declaration IDs exhaust native integer namespace");
    auto next=result.allocation_ceiling;
    AllocatedIds ids;for(auto& node:ids.nodes)node=++next;
    ids.shell=++next;ids.part=++next;ids.material=++next;ids.section=++next;
    ids.node_set=++next;ids.surface=++next;ids.interface=++next;
    result.definitions=entries.size();result.generated_node_after_complete_input=true;
    fields::Fields digest("declared-envelope-namespace-v1:"+c.source_digest,limits.metadata_bytes);
    digest.Add<std::uint64_t>("definitions",entries.size(),4,[&](auto i){const auto& e=entries[i/4];
        const std::uint64_t words[]{std::uint64_t(e.kind),e.id,e.member,e.row};return words[i%4];});
    const std::uint64_t range[]{result.maximum_declared,result.maximum_spring,result.allocation_ceiling,next};
    digest.Add<std::uint64_t>("namespace_bounds",1,4,[&](auto i){return range[i];});
    result.digest=digest.Finish().sha256;
    return {std::move(result),ids};
}
} // namespace crash::cases::vehicle_wall::native::detail
