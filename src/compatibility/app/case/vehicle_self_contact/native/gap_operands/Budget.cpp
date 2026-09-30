#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::gap_operands::detail {
Forecast Budget(const source::CorrectedNodalSource& source,Limits limits) {
    const Limits hard;const auto& counts=source.pre_correction().counts();
    Require(limits.host_bytes&&limits.host_bytes<=hard.host_bytes&&limits.shells&&limits.shells<=hard.shells&&
        limits.beams&&limits.beams<=hard.beams&&limits.springs&&limits.springs<=hard.springs&&
        limits.metadata_bytes&&limits.metadata_bytes<=hard.metadata_bytes,"Invalid gap operand source limits");
    const auto springs=counts.type13+counts.type25+counts.type45;
    if(counts.shells>limits.shells||counts.beams>limits.beams||springs>limits.springs)
        Reject(Status::ResourceLimit,"Gap source population exceeds its envelope");
    Forecast f;
    auto add=[&](std::size_t& field,std::size_t count,std::size_t width) {
        if(!width||f.peak_bytes>limits.host_bytes||count>(limits.host_bytes-f.peak_bytes)/width)
            Reject(Status::ResourceLimit,"Complete gap source reservation exceeds cap");
        const auto bytes=count*width;field+=bytes;f.peak_bytes+=bytes;
    };
    add(f.shared_source,source.forecast().peak_bytes,1);
    add(f.shell_rows,2*counts.shells,sizeof(values::PhysicalShell));
    add(f.line_rows,2*counts.beams,sizeof(values::Line));
    add(f.spring_rows,2*springs,sizeof(values::Spring));
    add(f.bindings,2*(counts.shells+counts.beams+springs),sizeof(Binding));
    const auto& canonical=source.pre_correction().physical().shell_source().references().source().canonical().data();
    add(f.source_workspace,16,canonical.canonical_bytes.size());
    add(f.metadata,16,limits.metadata_bytes);
    add(f.metadata,65536,1);
    for(const auto* text:{&source.provenance().source_digest,&source.provenance().property_digest,
        &source.pre_correction().provenance().contributor_digest})add(f.metadata,2,text->capacity()+1);
    return f;
}
}
