#include "Internal.h"
#include <algorithm>

namespace crash::modelio::vehicle::detail {
std::size_t Preflight(const source::CanonicalData& d,const assembly::ArtifactIdentity& id,Limits limits) {
    const Limits maximum;
    Require(limits.declaration_bytes&&limits.declaration_bytes<=maximum.declaration_bytes&&
        limits.host_bytes&&limits.host_bytes<=maximum.host_bytes&&limits.parents&&limits.parents<=maximum.parents&&
        limits.nodes&&limits.nodes<=maximum.nodes&&limits.parts&&limits.parts<=maximum.parts&&
        limits.tables&&limits.tables<=maximum.tables&&limits.curve_points>=2&&limits.curve_points<=maximum.curve_points,
        "Invalid vehicle source plan limits");
    Require(d.retained_shells<=limits.parents&&d.retained_nodes<=limits.nodes&&d.selected_parts.size()<=limits.parts&&
        id.bytes&&id.bytes<=limits.declaration_bytes&&id.sha256.size()==64&&
        std::all_of(id.sha256.begin(),id.sha256.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}),
        "Vehicle source count/content identity exceeds plan limits");
    std::size_t budget=0;
    const auto add=[&](std::size_t n,std::size_t width) {
        Require(n<=(limits.host_bytes-budget)/width,"Vehicle source startup byte cap exceeded");budget+=n*width;
    };
    // Conservative simultaneous value/DOM/copy budget, not allocator/RSS or a
    // native-runtime forecast. Include canonical shared payload once; source
    // loading is a separate already-completed CanonicalSource operation.
    for(const auto& a:d.arrays)add(a.bytes.size(),1);
    add(d.canonical_bytes.size(),1);add(d.scope_bytes.size(),5);
    add(id.bytes,16); // Input, parser DOM, typed cards/raw text and staged copies.
    add(d.canonical_nodes,32);add(d.canonical_shells,120);
    add(d.selected_parts.size()+d.parts.size(),2048);
    return budget;
}
void CheckAuthority(const source::CanonicalData& d,const Value& doc) {
    TextIs(doc,"schema",DeclarationSchema);Flag(doc,"simulation_ready",false);Flag(doc,"native_startup_qualified",false);
    const auto& s=Member(doc,"source");
    TextIs(s,"canonical_manifest_sha256",d.inputs.canonical_manifest.sha256);
    TextIs(s,"scope_sha256",d.inputs.scope_report.sha256);TextIs(s,"archive_sha256",d.archive_sha256);
    TextIs(s,"member_sha256",d.inputs.source_member.sha256);TextIs(s,"tire_policy",d.inputs.tire_policy);
    Require(Unsigned(s,"scope_bytes")==d.inputs.scope_report.bytes&&Unsigned(s,"member_bytes")==d.inputs.source_member.bytes,
            "Vehicle declaration source byte identity differs");
}
} // namespace crash::modelio::vehicle::detail
