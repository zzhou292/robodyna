#include "Coverage.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact::activity::coverage {
Coverage Audit(const detail::SourceInputs& in,Limits limits) {
    const auto& mechanical=in.owner.execution_source().mechanical();
    const auto& canonical=mechanical.vehicle_references().source().canonical().data();
    output::Require(canonical.inputs.tire_policy=="omit_original_tire_shells"&&limits.workspace_bytes&&
        limits.workspace_bytes<=Limits{}.workspace_bytes,"Unsupported activity coverage or workspace policy");
    std::size_t largest=0,rows=0;
    for(const auto* name:{"shells_records","beams_records","solids_records"}) {
        const auto& array=output::full_shell::source::FindArray(canonical,name);
        largest=std::max(largest,array.bytes.size());rows=std::max(rows,std::size_t(array.descriptor.layout.rows));
    }
    // Decode temporarily retains native bytes and the typed vector. Finish
    // keeps two bounded ID buffers and one serialized digest buffer. The
    // physical source identity index is conservatively charged simultaneously.
    tl::util::BoundedArenaLayout budget(limits.workspace_bytes);tl::util::ArenaRegion ignored;
    output::Require(budget.Append<std::byte>(2*largest,ignored)&&budget.Append<std::uint64_t>(3*rows,ignored)&&
        budget.Append<std::uint8_t>(rows,ignored)&&budget.Append<std::byte>(8u<<20,ignored),
        "Complete source coverage construction exceeds workspace cap");
    Coverage result;result.physical_nodes=in.owner.physical().domain()->node_count();result.wall_shells=1;
    result.canonical_sha256=canonical.inputs.canonical_manifest.sha256;
    result.scope_sha256=canonical.inputs.scope_report.sha256;result.source_member_sha256=canonical.inputs.source_member.sha256;
    result.shells=Shells(in,canonical);result.beams=Beams(in,canonical);result.solids=Solids(in,canonical);
    result.type13=mechanical.beams().connection_count();result.beam18=mechanical.structural_beams().parents().size();
    Connections(in,result);
    const auto& shell=*in.owner.physical().shells();const auto& scope=in.owner.physical().coefficients()->scope();
    result.families={shell.qeph_count(),shell.t3_count(),shell.qbat_count(),scope.solid18_parents,scope.solid24_parents,
        scope.solid6z_parents,scope.solid18_law44_parents,scope.solid18_law90_parents,result.beam18,result.welds,result.type13,result.joints};
    output::Require(result.shells.executed==canonical.retained_shells&&result.shells.excluded==canonical.excluded_shells&&
        result.beams.executed==result.type13+result.beam18&&result.solids.executed==scope.solid18_parents+
        scope.solid24_parents+scope.solid6z_parents+scope.solid18_law44_parents+scope.solid18_law90_parents,
        "Executed native support family census is incomplete");
    return result;
}
}
