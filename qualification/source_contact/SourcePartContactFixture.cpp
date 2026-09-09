#include "SourcePartContactFixture.h"

#include "output/ArtifactIO.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace crash::qualification::source_contact {
namespace {
namespace io=crash::output;
const io::Value& Member(const io::Value& object,const char* name) {
    io::Require(object.IsObject() && object.HasMember(name),"Missing fixture member");
    return object[name];
}
const io::Value& Array(const io::Value& object,const char* name,std::size_t count) {
    const auto& value=Member(object,name);
    io::Require(value.IsArray() && value.Size()==count,"Unexpected fixture array size"); return value;
}
std::uint64_t Unsigned(const io::Value& value,std::uint64_t maximum=UINT64_MAX) {
    io::Require(value.IsUint64() && value.GetUint64()<=maximum,"Invalid fixture integer"); return value.GetUint64();
}
std::uint64_t Unsigned(const io::Value& object,const char* name,std::uint64_t maximum=UINT64_MAX) {
    return Unsigned(Member(object,name),maximum);
}
bool Flag(const io::Value& object,const char* name) {
    const auto& value=Member(object,name); io::Require(value.IsBool(),"Invalid fixture boolean"); return value.GetBool();
}
std::string_view Text(const io::Value& object,const char* name) {
    const auto& value=Member(object,name); io::Require(value.IsString(),"Invalid fixture text");
    return {value.GetString(),value.GetStringLength()};
}
} // namespace

FixtureReport LoadPinnedSourcePartContact(const std::filesystem::path& path,SourcePartContactFixture* output) {
    if (!output) return {FixtureStatus::InvalidArgument,"Null fixture output"};
    std::string bytes;
    try { bytes=io::ReadBounded(path,ReadinessBytes); }
    catch (const std::exception& error) { return {FixtureStatus::ReadFailure,error.what()}; }
    try {
        if (bytes.size()!=ReadinessBytes || io::Sha256(bytes)!=ReadinessSha256)
            return {FixtureStatus::HashMismatch,"Expected the unchanged pinned E2a readiness artifact"};
        // Authenticate once-read bytes BEFORE parsing. Only this fixed report
        // is accepted, so unrelated/new JSON schemas and duplicate-member
        // variants cannot enter this qualification consumer.
        io::Document document;
        document.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
        io::Require(!document.HasParseError() && document.IsObject(),"Invalid pinned fixture JSON");
        io::Require(Text(document,"schema")=="robo-dyna.source-part-readiness.v1" &&
                    Flag(document,"selected_geometry_source_verified") &&
                    !Flag(document,"geometry_modified") && !Flag(document,"mechanics_capacity_changed") &&
                    !Flag(document,"simulation_ready"),"Unexpected fixture admission/scope");
        const auto& geometry=Member(document,"geometry");
        io::Require(Unsigned(geometry,"part_id")==PartId && Unsigned(geometry,"section_id")==PartId &&
                    Unsigned(geometry,"material_id")==PartId &&
                    Text(geometry,"source_frame")=="Untransformed original yaris-coarse-v1l.key coordinates" &&
                    Text(geometry,"source_member")=="2010-toyota-yaris-coarse-v1l/yaris-coarse-v1l.key",
                    "Unexpected fixture identity or coordinate frame");
        const auto& nodes=Array(geometry,"nodes",NodeCount);
        const auto& shells=Array(geometry,"shells",ParentCount);
        SourcePartContactFixture candidate;
        for (std::size_t i=0;i<NodeCount;++i) {
            const auto& source=nodes[static_cast<rapidjson::SizeType>(i)]; auto& node=candidate.nodes_[i];
            node.source_id=Unsigned(source,"source_id");
            node.canonical_index=static_cast<std::uint32_t>(Unsigned(source,"canonical_index",UINT32_MAX));
            node.source_line=static_cast<std::uint32_t>(Unsigned(source,"source_line",UINT32_MAX));
            node.blank_mask=static_cast<std::uint16_t>(Unsigned(source,"blank_mask",UINT16_MAX));
            io::Require(node.source_id && node.source_line,"Invalid source node identity");
            for (std::size_t j=0;j<i;++j)
                io::Require(node.source_id!=candidate.nodes_[j].source_id &&
                            node.canonical_index!=candidate.nodes_[j].canonical_index,"Duplicate source node identity");
            const auto& codes=Array(source,"codes",2);
            for (unsigned k=0;k<2;++k) {
                io::Require(codes[k].IsInt(),"Invalid source node code"); node.codes[k]=codes[k].GetInt();
            }
            const auto& point=Array(source,"position_m",3);
            for (unsigned k=0;k<3;++k) {
                io::Require(point[k].IsNumber() && std::isfinite(point[k].GetDouble()),"Invalid source position");
                candidate.coordinates_[3*i+k]=point[k].GetDouble();
            }
        }
        std::size_t q4_count=0,t3_count=0; std::array<bool,NodeCount> incident{};
        for (std::size_t i=0;i<ParentCount;++i) {
            const auto& source=shells[static_cast<rapidjson::SizeType>(i)]; auto& parent=candidate.parents_[i];
            parent.source_id=Unsigned(source,"source_id");
            parent.canonical_index=static_cast<std::uint32_t>(Unsigned(source,"canonical_index",UINT32_MAX));
            parent.source_line=static_cast<std::uint32_t>(Unsigned(source,"source_line",UINT32_MAX));
            parent.blank_mask=static_cast<std::uint16_t>(Unsigned(source,"blank_mask",UINT16_MAX));
            parent.arity=static_cast<std::uint8_t>(Unsigned(source,"arity",4));
            io::Require(parent.source_id && parent.source_line && (parent.arity==3 || parent.arity==4),
                        "Invalid source parent identity or arity");
            for (std::size_t j=0;j<i;++j)
                io::Require(parent.source_id!=candidate.parents_[j].source_id &&
                            parent.canonical_index!=candidate.parents_[j].canonical_index,"Duplicate source parent identity");
            const auto& raw=Array(source,"raw_record",6);
            const auto& canonical=Array(source,"canonical_node_indices",4);
            const auto& local=Array(source,"local_node_indices",4);
            for (unsigned n=0;n<6;++n) parent.raw_record[n]=Unsigned(raw[n]);
            io::Require(parent.raw_record[0]==parent.source_id && parent.raw_record[1]==PartId,
                        "Raw source parent/part disagreement");
            for (unsigned n=0;n<4;++n) {
                parent.local_node_indices[n]=static_cast<std::uint32_t>(Unsigned(local[n],NodeCount-1));
                parent.canonical_node_indices[n]=static_cast<std::uint32_t>(Unsigned(canonical[n],UINT32_MAX));
                const auto& node=candidate.nodes_[parent.local_node_indices[n]];
                io::Require(parent.canonical_node_indices[n]==node.canonical_index &&
                            parent.raw_record[2+n]==node.source_id,"Source connectivity/index disagreement");
                incident[parent.local_node_indices[n]]=true;
            }
            for (unsigned a=0;a<parent.arity;++a) for (unsigned b=0;b<a;++b)
                io::Require(parent.local_node_indices[a]!=parent.local_node_indices[b],"Repeated physical parent node");
            if (parent.arity==3) {
                io::Require(parent.local_node_indices[3]==parent.local_node_indices[2],"Malformed native T3 fourth slot");
                ++t3_count;
            } else ++q4_count;
        }
        io::Require(q4_count==Q4Count && t3_count==T3Count,"Incomplete source parent family coverage");
        for (bool used:incident) io::Require(used,"Unused source selection node");
        candidate.prepared_=true; *output=candidate;
        return {FixtureStatus::Ok,"Pinned source geometry copied without modification; no mechanics admission"};
    } catch (const std::exception& error) { return {FixtureStatus::InvalidFixture,error.what()}; }
}

bool SourcePartContactFixture::q4_parent(std::size_t index,tlfea::contact::SurfaceQ4& output) const noexcept {
    if (!prepared_ || index>=ParentCount || parents_[index].arity!=4) return false;
    const auto& source=parents_[index]; tlfea::contact::SurfaceQ4 parent;
    for (unsigned n=0;n<4;++n) parent.nodes[n]=source.local_node_indices[n];
    parent.feature_id=source.source_id; parent.parent_element_id=source.source_id;
    output=parent; return true;
}
bool SourcePartContactFixture::t3_parent(std::size_t index,tlfea::contact::SurfaceTriangle& output) const noexcept {
    if (!prepared_ || index>=ParentCount || parents_[index].arity!=3) return false;
    const auto& source=parents_[index]; tlfea::contact::SurfaceTriangle parent;
    for (unsigned n=0;n<3;++n) parent.nodes[n]=source.local_node_indices[n];
    parent.feature_id=source.source_id; parent.parent_element_id=source.source_id;
    parent.interpolation=tlfea::contact::SurfaceInterpolation::kLinearTriangle;
    output=parent; return true;
}
} // namespace crash::qualification::source_contact
