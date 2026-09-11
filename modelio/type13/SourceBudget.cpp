#include "ReadInternal.h"
#include <algorithm>

namespace crash::modelio::type13::reader {
namespace {
void Add(std::size_t& bytes,std::size_t count,std::size_t width,std::size_t cap) {
    Require(width&&bytes<=cap&&count<=(cap-bytes)/width,"TYPE13 host startup byte cap exceeded");
    bytes+=count*width;
}
}
std::size_t Preflight(const ArtifactIdentity& identity,ReadLimits limits) {
    const ReadLimits maximum;
    Require(limits.bytes&&limits.bytes<=maximum.bytes&&limits.nodes>=7494&&limits.nodes<=maximum.nodes&&
        limits.beams>=4442&&limits.beams<=maximum.beams&&limits.host_bytes&&limits.host_bytes<=maximum.host_bytes,
        "TYPE13 loading limits exceed the startup domain");
    Require(identity.bytes&&identity.bytes<=limits.bytes&&identity.sha256.size()==64&&
        std::all_of(identity.sha256.begin(),identity.sha256.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}),
        "TYPE13 loading requires an explicit bounded content identity");
    std::size_t bytes=0;
    Add(bytes,1,sizeof(Data)+64,limits.host_bytes);
    // Same conservative value/DOM/copy multiplier as VehicleSourcePlan. This
    // is a startup payload budget, not an allocator/RSS or runtime forecast.
    Add(bytes,identity.bytes,16,limits.host_bytes);
    Add(bytes,limits.nodes,sizeof(Node)+256,limits.host_bytes);
    Add(bytes,limits.beams,sizeof(Beam)+384,limits.host_bytes);
    return bytes;
}
std::size_t OwnedPayload(const Data& data,std::size_t cap) {
    std::size_t bytes=0;Add(bytes,1,sizeof(Data)+64,cap);
    const auto string=[&](const std::string& s){Add(bytes,s.capacity(),1,cap);Add(bytes,1,1,cap);};
    string(data.authenticated_bytes);string(data.identity.sha256);string(data.canonical_manifest_sha256);
    for(const auto* block:{&data.part_source,&data.section_source,&data.material_source}) {
        string(block->filename);string(block->keyword);string(block->raw_text);string(block->sha256);
    }
    Add(bytes,data.nodes.capacity(),sizeof(Node),cap);Add(bytes,data.beams.capacity(),sizeof(Beam),cap);
    for(const auto& node:data.nodes)string(node.raw_text);
    for(const auto& beam:data.beams)string(beam.raw_text);
    return bytes; // Inline-string capacity is conservatively counted again.
}
}
