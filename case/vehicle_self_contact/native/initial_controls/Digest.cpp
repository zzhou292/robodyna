#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
std::string Digest(const MainSource& main,const Wall* wall,const RawControls& raw,const GapScalars& gaps,
    const Namespace& table,Limits limits) {
    ::crash::cases::vehicle_self_contact::native::detail::digest::Fields fields(
        "native-initializer-controls-v1:"+main.provenance().source_digest+":"+main.provenance().output_digest+":"+
        (wall?wall->digest():"no-declared-addition"),limits.metadata_bytes);
    const std::uint64_t declarations[]{1,1,5,1,1,0,1,1,0,0,2,1,0,4,0,8000000,0,128,
        std::uint64_t(raw.source_soft),std::uint64_t(raw.source_ignore),std::uint64_t(raw.reader_gap_mode),
        std::uint64_t(raw.reader_idel),std::uint64_t(raw.reader_tied_removal),std::uint64_t(raw.reader_sharp),
        std::uint64_t(raw.property_type),wall?2u:0u,wall?1u:0u,
        std::uint64_t(raw.source_death_blank),std::uint64_t(raw.sensor_disabled),std::uint64_t(raw.stop_nonnegative),0};
    fields.Add<std::uint64_t>("source_and_resolved_control_words",1,std::size(declarations),[&](auto i){return declarations[i];});
    const auto law=ResolveSelfLaw(main.mixed().initial().selection().data(),main.provenance().units);
    const double law_values[]{law.lifecycle.minimum_coefficient,law.lifecycle.maximum_coefficient,law.normal.damping_factor,
        law.friction.alpha,law.friction_coefficients.base,law.friction_coefficients.c[0],law.friction_coefficients.c[1],
        law.friction_coefficients.c[2],law.friction_coefficients.c[3],law.friction_coefficients.c[4],law.friction_coefficients.c[5],
        raw.stfac,raw.slsfac,raw.dtstif,raw.global_stiffness,raw.friction_viscosity,raw.stop_time_lower_bound_native};
    fields.Add<double>("resolved_source_law_values_native",1,std::size(law_values),[&](auto i){return law_values[i];});
    const double scalars[]{static_cast<double>(.20f),0.,0.,raw.reader_global_gap,gaps.secondary_maximum,
        gaps.pre_ini_main_maximum,gaps.global_search_gap};
    fields.Add<double>("pre_ini_source_scalars_native",1,std::size(scalars),[&](auto i){return scalars[i];});
    fields.Add<std::uint64_t>("native_interface_identity",table.interfaces.size(),4,[&](auto i){
        const auto& row=table.interfaces[i/4];switch(i%4){case 0:return row.native_id;
        case 1:return std::uint64_t(row.native_storage_ordinal);case 2:return std::uint64_t(row.kind);
        default:return std::uint64_t(row.origin);}});
    const auto& p=table.population;
    const std::uint64_t population[]{p.lower,p.upper,p.complete_original_nodes,p.explicit_node_bound,p.rigid_definition_bound,
        p.discrete_bound,p.transform_bound,p.rigid_wall_bound,0};
    fields.Add<std::uint64_t>("complete_native_population_interval_not_exact",1,std::size(population),[&](auto i){return population[i];});
    // Variable strings bind through fixed SHA256 byte encoding; no addresses,
    // C++ padding or selected representative origin erases source provenance.
    std::string source;
    for(const auto& row:table.interfaces) {
        source+=std::to_string(row.source.filename.size())+":"+row.source.filename;
        source+=":"+row.source.keyword+":"+row.source.sha256+":"+std::to_string(row.source.first_line)+":"+std::to_string(row.source.last_line);
    }
    const auto source_hash=output::Sha256(source);
    fields.Add<std::uint32_t>("interface_source_blocks_sha256",1,source_hash.size(),[&](auto i){return std::uint32_t(static_cast<unsigned char>(source_hash[i]));});
    return fields.Finish().sha256;
}
}
