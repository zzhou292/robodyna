#include "Internal.h"
#include "lib_src/math/ScalarBits.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
void Check(const MainSource& main,const Wall* wall) {
    const auto& corrected=main.gap_operands().corrected();
    Require(main.provenance().stage==post_gapm::Stage::PreparedSupportAndPreBucGaps&&
        main.gap_report().completed&&main.primary_owners().size()==main.counts().primaries&&
        main.provenance().source_digest==corrected.provenance().source_digest,
        "Initializer controls need genuine completed post-GAPM source values");
    const auto& interfaces=corrected.provenance().interfaces;
    Require(interfaces.disposition==nodal_correction::InterfaceDisposition::CompleteNoApplicableType24&&
        interfaces.type25_sources==1&&interfaces.type2_sources==1,
        "Initializer original interface census is outside the closed direct-source profile");
    if(!wall)return;
    const auto& canonical=main.mixed().initial().selection().canonical();
    Require(&wall->vehicle_origin().source().tied_source().canonical().data()==&canonical.data()&&
        wall->namespace_report().source_digest==main.provenance().source_digest&&
        wall->declaration().profile==vehicle_wall::native::Profile::EnvelopeFixedElasticV1,
        "Initializer wall and vehicle do not share authentic original source authority");
    const auto mesh=main.startup_input();const auto prefix=wall->vehicle_origin().domain().nodes();
    Require(mesh.node_count==prefix.size()&&wall->domain().node_count()==prefix.size()+4,
        "Initializer declared wall has no exact original-domain prefix");
    for(std::size_t i=0;i<prefix.size();++i) {
        const auto native=mesh.positions.at(std::uint32_t(i));const auto& x=prefix[i];
        Require(mesh.node_source_ids[i]==x.source_id&&
            tl::math::SameScalarBits(native.x*main.provenance().units.length_m,x.position.x)&&
            tl::math::SameScalarBits(native.y*main.provenance().units.length_m,x.position.y)&&
            tl::math::SameScalarBits(native.z*main.provenance().units.length_m,x.position.z),
            "Initializer wall prefix differs from original NID/native coordinate authority");
    }
}
Forecast Budget(const MainSource& main,const ids::ImportMembers& members,const Wall* wall,Limits limits) {
    const Limits hard;
    Require(limits.host_bytes&&limits.host_bytes<=hard.host_bytes&&limits.interfaces&&limits.interfaces<=hard.interfaces&&
        limits.blocks&&limits.blocks<=hard.blocks&&limits.metadata_bytes&&limits.metadata_bytes<=hard.metadata_bytes,
        "Initializer control capacity is invalid");
    Check(main,wall);
    Forecast f;f.upstream_retained=main.forecast().retained_bytes;f.prior_peak=main.forecast().peak_bytes;
    if(wall){f.wall_retained_bound=wall->forecast().peak_bytes;f.prior_peak=std::max(f.prior_peak,wall->forecast().peak_bytes);}
    const auto imported=ids::ImportContext::Preflight(main.mixed().initial().selection().canonical(),members,limits.import);
    f.import_workspace=imported.total_bytes; // Conservative extra charge includes shared canonical backing.
    tl::util::BoundedArenaLayout workspace(limits.host_bytes);tl::util::ArenaRegion region;
    const bool workspace_ok=workspace.Append<std::byte>(16*limits.metadata_bytes,region)&&
        workspace.Append<std::byte>(2*limits.blocks*1024,region)&&
        workspace.Append<std::byte>(2*main.gap_operands().shells().size()*(sizeof(std::uint64_t)+3*sizeof(std::size_t)),region);
    if(!workspace_ok)Reject(Status::ResourceLimit,"Initializer control namespace/gap workspace exceeds cap");
    f.namespace_workspace=workspace.bytes();
    f.output_values=2*limits.interfaces*(sizeof(Interface)+512)+sizeof(GapScalars)+sizeof(RawControls)+
        sizeof(Controls)+4*limits.metadata_bytes+8192;
    tl::util::BoundedArenaLayout current(limits.host_bytes);
    if(!(current.Append<std::byte>(f.upstream_retained,region)&&current.Append<std::byte>(f.wall_retained_bound,region)&&
        current.Append<std::byte>(f.output_values,region)))Reject(Status::ResourceLimit,"Initializer control retained graph exceeds cap");
    f.retained_bytes=current.bytes();
    if(!(current.Append<std::byte>(f.import_workspace,region)&&current.Append<std::byte>(f.namespace_workspace,region)))
        Reject(Status::ResourceLimit,"Initializer control complete current construction exceeds cap");
    f.current_phase=current.bytes();f.peak_bytes=std::max(f.prior_peak,f.current_phase);
    if(f.peak_bytes>limits.host_bytes)Reject(Status::ResourceLimit,"Initializer control complete phase chain exceeds cap");return f;
}
std::size_t RetainedNamespace(const Namespace& values,std::size_t cap) {
    tl::util::BoundedArenaLayout bytes(cap);tl::util::ArenaRegion region;
    if(!bytes.Append<Interface>(values.interfaces.capacity(),region))Reject(Status::ResourceLimit,"Retained interface capacity exceeds forecast");
    for(const auto& row:values.interfaces)for(const auto* value:{&row.source.filename,&row.source.keyword,&row.source.raw_text,&row.source.sha256})
        if(!bytes.Append<char>(value->capacity()+1,region))Reject(Status::ResourceLimit,"Retained interface source strings exceed forecast");
    return bytes.bytes();
}

}
