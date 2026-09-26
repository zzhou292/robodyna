#include "Internal.h"
#include <algorithm>
namespace crash::modelio::native_spring_ids {
namespace {
void Add(std::size_t& bytes, std::size_t value) {
    if (value > SIZE_MAX-bytes) detail::Reject(Readiness::ResourceLimit, "SPRING resolution forecast overflow");
    bytes += value;
}
void SharedSource(const type13::SourceType13& beams, const type25::VehicleType25Source& welds,
        const type45::VehicleType45Source& joints, const ImportContext& context) {
    const auto& canonical = context.canonical().data();
    if (&welds.source().tied_source().canonical().data() != &canonical ||
        &joints.source_domain().source().tied_source().canonical().data() != &canonical ||
        beams.data().canonical_manifest_sha256 != canonical.inputs.canonical_manifest.sha256 ||
        !welds.domain().SharesStorage(joints.source_domain().domain()) ||
        joints.policy() != type45::Policy::OriginalDirectSdiType45VehicleSupportsV5 ||
        welds.declaration().policy != type25::Policy::OriginalDefaultSpotweldsV1)
        detail::Reject(Readiness::IdentityMismatch, "Generated ID handles do not share the retained V5 source/domain");
}
}
Forecast Preflight(const type13::SourceType13& beams, const type25::VehicleType25Source& welds,
        const type45::VehicleType45Source& joints, const ImportContext& context, Limits limits) {
    detail::CheckLimits(limits); SharedSource(beams, welds, joints, context);
    const auto count = context.data().precursors.size()+context.data().welds.size()+context.data().joints.size();
    if (count > limits.rows) detail::Reject(Readiness::ResourceLimit, "Complete generated ID count exceeds cap");
    Forecast result;
    // Conservatively overcharges shared immutable source handles rather than
    // claiming their private storage layouts or allocating another model.
    result.retained_source = context.data().forecast.total_bytes;
    Add(result.retained_source, beams.data().startup_budget_bytes);
    Add(result.retained_source, welds.forecast().total_bytes);
    Add(result.retained_source, joints.forecast().total_bytes);
    result.result_reservation = 4*count*(sizeof(Row)+256)+65536;
    if (result.result_reservation > limits.resolve_bytes)
        detail::Reject(Readiness::ResourceLimit, "Integer mapping workspace exceeds cap");
    result.total_bytes = result.retained_source; Add(result.total_bytes, result.result_reservation);
    if (result.total_bytes > limits.host_bytes) detail::Reject(Readiness::ResourceLimit, "Complete generated ID source reservation exceeds cap");
    return result;
}
Resolution Resolve(const type13::SourceType13& beams, const type25::VehicleType25Source& welds,
        const type45::VehicleType45Source& joints, const ImportContext& context, Limits limits) {
    Resolution result;
    try {
        if (context.data().diagnostic.status != Readiness::Ready) { result.diagnostic = context.data().diagnostic; return result; }
        (void)Preflight(beams, welds, joints, context, limits);
        std::vector<SourceRow> retained;
        retained.reserve(beams.data().beams.size()+welds.model().connection_count()+joints.data().rows.size());
        for (std::size_t i = 0; i < beams.data().beams.size(); ++i) {
            const auto& beam = beams.data().beams[i]; SourceRow row;
            row.kind = SourceKind::Type13; row.original_id = beam.id; row.source_index = i;
            row.canonical_index = beam.canonical_index; row.physical_participant = true;
            for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
                if (beam.node_indices[endpoint] >= beams.data().nodes.size()) detail::Reject(Readiness::InvalidSource, "TYPE13 source endpoint index invalid");
                row.endpoints[endpoint] = beams.data().nodes[beam.node_indices[endpoint]].id;
            }
            retained.push_back(std::move(row));
        }
        for (std::size_t i = 0; i < welds.model().connection_count(); ++i) {
            const auto& input = welds.model().connections()[i]; SourceRow row;
            row.kind = SourceKind::DefaultSpotweld; row.original_id = input.source_element_id;
            row.endpoints = {input.source_node_id[0], input.source_node_id[1]}; row.source_index = i;
            row.physical_participant = true; retained.push_back(std::move(row));
        }
        for (std::size_t i = 0; i < joints.data().rows.size(); ++i) {
            const auto& input = joints.data().rows[i]; SourceRow row;
            row.kind = SourceKind::RegularJoint; row.original_id = input.source_id;
            row.endpoints = {input.nodes[0].source_id, input.nodes[1].source_id}; row.source_index = i;
            row.physical_participant = input.disposition == type45::Disposition::Required;
            retained.push_back(std::move(row));
        }
        return detail::ResolveRows(context.data(), std::move(retained), limits);
    } catch (const detail::Failure& error) {
        result = {}; result.diagnostic = error.diagnostic;
    } catch (const std::exception& error) {
        result = {}; result.diagnostic = {Readiness::InvalidSource, std::string(error.what()).substr(0,1024), {}, 0, 0};
    }
    return result;
}
} // namespace crash::modelio::native_spring_ids
