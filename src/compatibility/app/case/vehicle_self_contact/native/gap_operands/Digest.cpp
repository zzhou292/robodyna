#include "Internal.h"
#include "../TopologyDigestFields.h"
namespace crash::cases::vehicle_self_contact::native::gap_operands::detail {
std::string Digest(const Packed& values,const Provenance& provenance,std::size_t cap) {
    for(const auto* hash:{&provenance.source_digest,&provenance.contributor_digest,&provenance.property_digest})
        output::arrays::CheckHash(*hash);
    native::detail::digest::Fields fields("v5-native-gap-operands-v1:"+provenance.source_digest+
        provenance.contributor_digest+provenance.property_digest,cap);
    fields.Add<double>("native_units",1,3,[&](auto i) {
        return i==0?provenance.units.length_m:i==1?provenance.units.mass_kg:provenance.units.time_s;
    });
    const auto& proof=values.proof;
    const std::uint64_t controls[]{std::uint64_t(proof.overrides),std::uint64_t(proof.order),proof.checked_keywords,
        proof.maximum_terms,proof.skipped_springs,std::uint64_t(proof.no_retained_trusses)};
    fields.Add<std::uint64_t>("source_proof",1,6,[&](auto i) { return controls[i]; });
    const auto& c=values.counts;
    const std::uint64_t counts[]{c.nodes,c.shells,c.quads,c.triangles,c.beams,c.type13,c.type25,c.type45,
        c.solids_without_direct_gap_term,c.namespace_only_springs};
    fields.Add<std::uint64_t>("complete_population",1,10,[&](auto i) { return counts[i]; });
    fields.Add<std::uint64_t>("source_bindings",values.bindings.size(),6,[&](auto i) {
        const auto& b=values.bindings[i/6];
        const std::uint64_t row[]{std::uint64_t(b.family),b.original_id,b.native_id,b.source_part_id,b.source_index,b.operand_row};
        return row[i%6];
    });
    fields.Add<std::uint64_t>("shell_identity",values.shells.size(),2,[&](auto i) {
        const auto& shell=values.shells[i/2];return i%2?std::uint64_t(shell.layout):shell.source_element_id;
    });
    fields.Add<std::uint32_t>("physical_shell_nodes",values.shells.size(),4,[&](auto i) { return values.shells[i/4].nodes[i%4]; });
    fields.Add<double>("native_shell_gap_operands",values.shells.size(),3,[&](auto i) {
        const auto& shell=values.shells[i/3];
        return i%3==0?shell.part_contact_thickness:i%3==1?shell.element_thickness:shell.property_thickness;
    });
    fields.Add<std::uint64_t>("beam_eid",values.beams.size(),1,[&](auto i) { return values.beams[i].source_element_id; });
    fields.Add<std::uint32_t>("beam_endpoints",values.beams.size(),2,[&](auto i) { return values.beams[i/2].nodes[i%2]; });
    fields.Add<double>("native_pre_pmass_beam_operands",values.beams.size(),2,[&](auto i) {
        return i%2?values.beams[i/2].native_area:values.beams[i/2].part_contact_thickness;
    });
    fields.Add<std::uint64_t>("spring_native_id_type",values.springs.size(),2,[&](auto i) {
        return i%2?std::uint64_t(values.springs[i/2].property_type):values.springs[i/2].native_element_id;
    });
    fields.Add<std::uint32_t>("physical_spring_endpoints",values.springs.size(),2,[&](auto i) { return values.springs[i/2].nodes[i%2]; });
    fields.Add<double>("spring_part_override",values.springs.size(),1,[&](auto i) { return values.springs[i].part_contact_thickness; });
    return fields.Finish().sha256;
}
}
