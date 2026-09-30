#include "Coverage.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact::activity::coverage {
Population Shells(const detail::SourceInputs& in,const Canonical& canonical) {
    const auto& array=output::full_shell::source::FindArray(canonical,"shells_records");
    output::Require(array.descriptor.layout.columns==6,"Canonical shell record width differs");
    const auto records=output::arrays::Decode<std::uint64_t>(array.descriptor,array.bytes);
    std::vector<std::uint8_t> selected(records.size()/6);
    const auto& physical=in.owner.physical();const auto& binding=*physical.shells();
    const auto& mechanical=in.owner.execution_source().mechanical();
    const auto& refs=mechanical.vehicle_references();
    output::Require(refs.counts().rejected==0&&refs.counts().unresolved==0&&
        refs.rows().size()+1==binding.qeph_count()+binding.t3_count()+binding.qbat_count(),
        "Activity shell coverage requires every selected source plus the declared environment parent");
    for(const auto& row:refs.rows()) {
        std::uint64_t id=0,nodes[4]{};std::size_t local[4]{};
        using Family=vehicle_startup::ReferenceFamily;
        if(row.family==Family::Qeph&&row.reference_index<binding.qeph_count()) {
            id=binding.qeph_source_id(row.reference_index);const auto x=binding.qeph_nodes(row.reference_index);std::copy(x.begin(),x.end(),local);
        } else if(row.family==Family::T3&&row.reference_index<binding.t3_count()) {
            id=binding.t3_source_id(row.reference_index);const auto x=binding.t3_nodes(row.reference_index);
            for(unsigned k=0;k<4;++k)local[k]=x[k<3?k:2];
        } else if(row.family==Family::Qbat&&row.reference_index<binding.qbat_count()) {
            id=binding.qbat_source_id(row.reference_index);const auto x=binding.qbat_nodes(row.reference_index);std::copy(x.begin(),x.end(),local);
        }
        output::Require(id==row.element_id&&row.status==vehicle_startup::ReferenceStatus::Success,"Activity shell family/source identity differs");
        for(unsigned k=0;k<4;++k)nodes[k]=physical.domain()->nodes()[physical.mapping()->owner_index(local[k])].source_id;
        CheckRecord(records,6,row.canonical_parent,id,nodes,4,selected);
    }
    for(std::size_t row=0;row<selected.size();++row) {
        const auto part=records[6*row+1];const auto& parts=selected[row]?canonical.selected_parts:canonical.excluded_parts;
        output::Require(std::binary_search(parts.begin(),parts.end(),part),"Shell activity exclusion differs from the authenticated no-tire selection");
    }
    const auto& environment=mechanical.environment_parent();
    output::Require(environment.qeph_index<binding.qeph_count()&&binding.qeph_source_id(environment.qeph_index)==environment.element_id&&
        environment.element_id==in.wall.wall().ids().shell,"Environment shell is not the declared mesh wall");
    return Finish(records,6,4,selected,*physical.domain());
}
}
