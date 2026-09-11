#include "Internal.h"
namespace crash::modelio::vehicle::rigid_part::detail {
void ReadSources(SourceData& out,const source::CanonicalData& d,const std::string& member,Limits limits) {
    Require(member.size()==d.inputs.source_member.bytes && output::Sha256(member)==d.inputs.source_member.sha256,
            "Rigid original member authentication failed");
    output::Document manifest;
    manifest.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag|
                   rapidjson::kParseValidateEncodingFlag>(d.canonical_bytes.data(),d.canonical_bytes.size());
    Require(!manifest.HasParseError() && manifest.IsObject(),"Invalid canonical rigid source metadata");
    tied_shell::detail::Requests requests;
    const auto& file=Member(Member(manifest,"source_files"),"yaris-coarse-v1l.key");
    const auto& blocks=Array(file,"blocks",limits.blocks,1);
    std::size_t previous=0;
    for (const auto& block:blocks.GetArray()) {
        const auto keyword=Text(block,"keyword");
        const auto first=Unsigned(block,"first_line"),last=Unsigned(block,"last_line");
        Require(Text(block,"file")=="yaris-coarse-v1l.key" && first>previous && last>=first,
                "Rigid source block order/extent changed");
        previous=last;
        const bool selected=keyword.rfind("*SET_NODE_",0)==0 ||
            keyword.rfind("*CONSTRAINED_NODAL_RIGID_BODY",0)==0 ||
            keyword.rfind("*CONSTRAINED_EXTRA_NODES",0)==0 || keyword=="*CONSTRAINED_RIGID_BODIES" ||
            keyword.rfind("*CONSTRAINED_JOINT_",0)==0 || keyword=="*ELEMENT_MASS" ||
            keyword=="*ELEMENT_DISCRETE";
        if (!selected) continue;
        tied_shell::detail::Request request;
        request.evidence.block={"yaris-coarse-v1l.key",keyword,{},Text(block,"source_block_sha256"),first,last};
        output::arrays::CheckHash(request.evidence.block.sha256);
        Require(requests.emplace(first,std::move(request)).second,"Repeated rigid source request");
    }
    tied_shell::Limits source_limits;
    source_limits.metadata_bytes=limits.metadata_bytes;
    source_limits.blocks=limits.blocks;
    out.sources=tied_shell::detail::ReadRequestedSources(requests,member,source_limits);
}
}
