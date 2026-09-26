#include "lib_utils/BoundedArena.h"
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
    std::vector<OriginGroup> origin_groups;
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
        const auto part_preflight = values::Preflight(source, {}, actual);
        if (part_preflight.status != values::Status::Ok) {
            result.report = d::NumericalFailure(NumericalStage::PartPreflight, part_preflight,
                "Genuine PART surface descriptors were rejected");
            return result;
        }
        const auto probe_preflight = values::Preflight(probe, {}, probe_capacity);
        if (probe_preflight.status != values::Status::Ok) {
            result.report = d::NumericalFailure(NumericalStage::ProbePreflight, probe_preflight,
                "Genuine SOLID probe descriptors were rejected");
            return result;
        }
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
        if (extracted.status != values::Status::Ok) {
            result.report = d::NumericalFailure(NumericalStage::ProbeBuild, extracted,
                "Complete physical surface probe rejected");
            return result;
        }
        const auto membership = d::CertifyMembership(packed, observed, next->certificate);
        if (membership.status != Status::Ready) { result.report = membership; return result; }
        // Boolean internal cancellation is invariant under row permutation:
        // every selected solid remains in the complete existence search. The
        // certificate above proves the only first-shell branch invariant.
        extracted = values::Build(source, {}, output, scratch, &observed);
        if (extracted.status != values::Status::Ok || !extracted.count_complete) {
            result.report = d::NumericalFailure(NumericalStage::PartBuild, extracted,
                "Complete authenticated PART extraction rejected");
            return result;
        }
        next->faces.reserve(observed.face_count);
        for (std::size_t i = 0; i < observed.face_count; ++i)
            next->faces.push_back(d::ExternalFace(observed.faces[i], packed, next->geometry));
        if (observed.solid_count)
            next->solid_flags.assign(observed.surface_solid_flags, observed.surface_solid_flags + observed.solid_count);
        if (next->faces.capacity() > 2*observed.face_count || next->solid_flags.capacity() > 2*observed.solid_count)
            d::Reject(Status::ResourceLimit, "Retained initial surface capacities exceed reservation");
        const auto ordering = d::CertifyConsumerOrder(next->faces, next->certificate, &next->origin_groups);
        if (ordering.status != Status::Ready) { result.report = ordering; return result; }
        if (next->origin_groups.capacity() > forecast.origin_group_bytes / sizeof(OriginGroup))
            d::Reject(Status::ResourceLimit, "Retained origin group capacity exceeds reservation");
        auto& census = next->census;
        census.nodes = next->geometry.nodes.size();
        census.physical_shells = next->geometry.shells.size();
        census.physical_solids = next->geometry.solids.size();
        for (const auto& solid : packed.solids) {
            if (solid.topology == values::SolidTopology::DeclaredPenta6) ++census.declared_penta;
            else {
                ++census.reader_bricks;
                if (solid.topology == values::SolidTopology::NativeRaw8) ++census.native_raw8_bricks;
            }
        }

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
        result.report = {Status::Ready, "Initial consumed fields are invariant; every source origin and solid tag remains retained"};
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
const std::vector<OriginGroup>& InitialSurfaceSource::origin_groups() const noexcept { return data_->origin_groups; }
const std::vector<std::uint8_t>& InitialSurfaceSource::emitted_solid_flags() const noexcept { return data_->solid_flags; }
const Certificate& InitialSurfaceSource::certificate() const noexcept { return data_->certificate; }
const Census& InitialSurfaceSource::census() const noexcept { return data_->census; }
const Provenance& InitialSurfaceSource::provenance() const noexcept { return data_->provenance; }
const Forecast& InitialSurfaceSource::forecast() const noexcept { return data_->forecast; }
}

namespace crash::cases::vehicle_self_contact::native::initial_surfaces {

std::size_t InitialSurfaceSource::retained_host_upper_bound(std::size_t cap) const {
    tl::util::BoundedArenaLayout bytes(cap); tl::util::ArenaRegion unused;
    const auto add=[&](std::size_t count) { output::Require(bytes.Append<std::byte>(count,unused),"Retained initial surface source exceeds cap"); };
    // Context's public bound remains conservative; no private native layout or
    // guessed overlap is subtracted. Selection reports its actual own payload.
    add(context().forecast().peak_bytes); add(selection().data().owned_payload_bytes);
    add(sizeof(InitialSurfaceSource)+sizeof(Data)+4096);
    const auto& g=data_->geometry;
    add(g.nodes.capacity()*sizeof(coated::Node));add(g.shells.capacity()*sizeof(coated::Shell));add(g.solids.capacity()*sizeof(coated::Solid));
    add(data_->faces.capacity()*sizeof(Face));add(data_->origin_groups.capacity()*sizeof(OriginGroup));add(data_->solid_flags.capacity());
    for(const auto* text:{&data_->provenance.source_digest,&data_->provenance.selection_digest,&data_->provenance.input_digest,
            &data_->provenance.output_digest,&data_->provenance.control_rule})add(text->capacity()+1);
    return bytes.bytes();
}
}
