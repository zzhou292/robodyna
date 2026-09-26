#include "solid_surfaces/Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::initial_surfaces {
namespace d = detail;
struct InitialSurfaceSource::Data {
    Data(const Context& value, const Selection& selected) : context(value), selection(selected) {}
    Context context;
    Selection selection;
    coated::Inputs geometry;
    std::vector<Face> faces;
    std::vector<std::uint8_t> solid_flags;
    Certificate certificate;
    Census census;
    Provenance provenance;
    Forecast forecast;
};
Forecast InitialSurfaceSource::Preflight(const Context& context, const Selection& selection, Limits limits) {
    return d::Budget(context, selection, limits);
}
Preparation InitialSurfaceSource::Prepare(const Context& context, const Selection& selection,
        const std::string& member, Limits limits) {
    Preparation result;
    try {
        const auto forecast = Preflight(context, selection, limits);
        const auto controls = d::ResolveControls(context, selection, limits);
        auto next = std::make_shared<Data>(context, selection);
        next->forecast = forecast;
        const auto& model = context.pre_correction().physical();
        coated::Limits geometry_limits;
        geometry_limits.host_bytes = limits.host_bytes;
        geometry_limits.nodes = limits.nodes;
        geometry_limits.shells = limits.shells;
        geometry_limits.solids = limits.solids;
        geometry_limits.metadata_bytes = limits.metadata_bytes;
        next->geometry = coated::detail::PrepareInputs(model, selection, member, {}, geometry_limits);
        const auto packed = d::Pack(next->geometry, selection.data().selected_part_ids);
        d::CheckCapacity(next->geometry, packed, 0);
        next->provenance.source_digest = context.provenance().source_digest;
        next->provenance.selection_digest = output::Sha256(selection.data().combine_sha256 +
            ":" + selection.data().auxiliary_sha256);
        next->provenance.control_rule = controls.rule;
        next->provenance.input_digest = d::InputDigest(next->geometry, packed, next->provenance, limits.metadata_bytes);
        values::Forecast actual, probe_capacity;
        const auto source = packed.Input(false);
        const auto probe = packed.Input(true);
        if (values::Preflight(source, {}, actual).status != values::Status::Ok ||
            values::Preflight(probe, {}, probe_capacity).status != values::Status::Ok)
            d::Reject(Status::InvalidInput, "Genuine initial surface descriptors were rejected");
        const auto output_bytes = std::max(actual.output_bytes, probe_capacity.output_bytes);
        const auto scratch_bytes = std::max(actual.scratch_bytes, probe_capacity.scratch_bytes);
        if (output_bytes > forecast.extraction_output || scratch_bytes > forecast.extraction_scratch ||
            actual.maximum_faces > forecast.maximum_faces)
            d::Reject(Status::ResourceLimit, "Actual extraction exceeds admitted public reservations");
        tl::util::HostArena output, scratch;
        if (!output.Initialize(output_bytes) || !scratch.Initialize(scratch_bytes))
            d::Reject(Status::ResourceLimit, "Initial surface arena allocation failed");
        values::Snapshot observed;
        // Same selected solid set and qualified exterior cancellation. An
        // explicit SOLID clause has no selected shell PART tag, so this probe
        // reveals exactly the faces that reach native shell suppression.
        auto extracted = values::Build(probe, {}, output, scratch, &observed);
        if (extracted.status != values::Status::Ok)
            d::Reject(Status::InvalidInput, "Complete physical surface probe rejected");
        const auto membership = d::CertifyMembership(packed, observed, next->certificate);
        if (membership.status != Status::Ready) { result.report = membership; return result; }
        // Boolean internal cancellation is invariant under row permutation:
        // every selected solid remains in the complete existence search. The
        // certificate above proves the only first-shell branch invariant.
        extracted = values::Build(source, {}, output, scratch, &observed);
        if (extracted.status != values::Status::Ok || !extracted.count_complete)
            d::Reject(Status::InvalidInput, "Complete authenticated PART extraction rejected");
        next->faces.reserve(observed.face_count);
        for (std::size_t i = 0; i < observed.face_count; ++i)
            next->faces.push_back(d::ExternalFace(observed.faces[i], packed, next->geometry));
        if (observed.solid_count)
            next->solid_flags.assign(observed.surface_solid_flags, observed.surface_solid_flags + observed.solid_count);
        if (next->faces.capacity() > 2*observed.face_count || next->solid_flags.capacity() > 2*observed.solid_count)
            d::Reject(Status::ResourceLimit, "Retained initial surface capacities exceed reservation");
        const auto ordering = d::CertifyOrder(next->faces, next->certificate);
        if (ordering.status != Status::Ready) { result.report = ordering; return result; }
        auto& census = next->census;
        census.nodes = next->geometry.nodes.size();
        census.physical_shells = next->geometry.shells.size();
        census.physical_solids = next->geometry.solids.size();
        census.original_selected_solids = selection.data().counts.solids;
        census.extraction = observed.counts;
        if (census.original_selected_solids < census.extraction.selected_solids ||
            census.extraction.selected_quads + census.extraction.selected_triangles != selection.data().counts.retained_shells)
            d::Reject(Status::InvalidInput, "Initial surface selection census differs from complete source");
        census.omitted_selected_solids = census.original_selected_solids - census.extraction.selected_solids;
        census.faces = next->faces.size();
        for (const auto& face : next->faces) {
            if (face.nodes[2] == face.nodes[3]) ++census.triangle_faces;
            else ++census.quad_faces;
        }
        next->provenance.output_digest = d::OutputDigest(next->faces, next->solid_flags,
            next->certificate, next->provenance, limits.metadata_bytes);
        result.report = {Status::Ready, "Complete source initial surface buffer has certified order independence"};
        result.source.emplace(InitialSurfaceSource(std::move(next)));
    } catch (const d::Failure& failure) {
        result.source.reset();
        result.report = failure.report;
    } catch (const std::bad_alloc&) {
        result.source.reset();
        result.report = {Status::ResourceLimit, "Initial source allocation failed"};
    } catch (const std::exception& failure) {
        result.source.reset();
        result.report = {Status::InvalidInput, std::string(failure.what()).substr(0, 1024)};
    }
    return result;
}
const Context& InitialSurfaceSource::context() const noexcept { return data_->context; }
const Selection& InitialSurfaceSource::selection() const noexcept { return data_->selection; }
const coated::Inputs& InitialSurfaceSource::geometry() const noexcept { return data_->geometry; }
const std::vector<Face>& InitialSurfaceSource::faces() const noexcept { return data_->faces; }
const std::vector<std::uint8_t>& InitialSurfaceSource::emitted_solid_flags() const noexcept { return data_->solid_flags; }
const Certificate& InitialSurfaceSource::certificate() const noexcept { return data_->certificate; }
const Census& InitialSurfaceSource::census() const noexcept { return data_->census; }
const Provenance& InitialSurfaceSource::provenance() const noexcept { return data_->provenance; }
const Forecast& InitialSurfaceSource::forecast() const noexcept { return data_->forecast; }
}
