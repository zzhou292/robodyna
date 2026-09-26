#include "main_coefficients/Compose.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::main_coefficients {
namespace d = detail;
struct SelectedShellMainSource::Data {
    Data(const c::CorrectedNodalSource& source, const modelio::self_contact::OriginalSelection& selected, Forecast capacity)
        : corrected(source), selection(selected), forecast(capacity) {}
    c::CorrectedNodalSource corrected;
    modelio::self_contact::OriginalSelection selection;
    Forecast forecast;
    Provenance provenance;
    Certificate certificate;
    tl::util::HostArena arena;
    coated::s::Snapshot topology;
    coated::s::Report topology_report;
    std::vector<PrimaryBinding> bindings;
    std::vector<CandidateOwner> possible_owners;
    std::vector<double> coefficients;
};
namespace {
void RecordDuplicates(const d::Packed& input, Certificate& result) {
    for (std::size_t first = 0; first < input.keys.size();) {
        auto last = first + 1;
        while (last < input.keys.size() && d::SameKey(input.keys[first], input.keys[last])) ++last;
        if (last - first > 1) {
            ++result.physical_duplicate_groups;
            result.physical_duplicate_rows += last - first;
        }
        first = last;
    }
}
n::NativeSolidMainCoefficientInput SolidValues(const c::CorrectedNodalSource& source,
    const coated::Inputs& input, const coated::Solid& solid, const coated::s::Main& main,
    n::ShellLayout layout, std::size_t primary, Certificate& certificate) {
    n::NativeExteriorMainGeometryInput packet;
    packet.layout = layout;
    for (unsigned k = 0; k < 4; ++k) packet.face[k] = input.nodes.at(main.nodes[k]).native_position;
    for (unsigned k = 0; k < 8; ++k) packet.solid_raw[k] = input.nodes.at(solid.nodes[k]).native_position;
    n::NativeExteriorMainGeometryResult geometry;
    const auto status = n::EvaluateNativeExteriorMainGeometry(packet, &geometry);
    if (status != n::CoefficientStatus::Ok)
        d::Reject(status == n::CoefficientStatus::NonfiniteResult ? Status::NonfiniteResult : Status::InvalidInput,
            "Native coated main geometry rejected source reader-phase operands", primary, main.source_id, solid.source_id);
    d::CheckIdentity(main, geometry, primary, main.source_id);
    if (geometry.signed_volume < 0.) ++certificate.negative_support_volumes;
    const auto query = source.material_slots(solid.part_id);
    if (query.status != c::MaterialSlotStatus::Ready || !query.values)
        d::Reject(Status::MissingMaterialContext, "Post-UPDMAT solid material slots unavailable", primary,
            main.source_id, solid.source_id);
    const auto& material = *query.values;
    d::Require(tl::math::SameScalarBits(material.units.length_m, input.units.length_m) &&
        tl::math::SameScalarBits(material.units.mass_kg, input.units.mass_kg) &&
        tl::math::SameScalarBits(material.units.time_s, input.units.time_s), "Main geometry/material units differ");
    const auto& parts = source.part_controls();
    const auto at = std::lower_bound(parts.begin(), parts.end(), solid.part_id,
        [](const auto& part, auto id) { return part.part_id < id; });
    d::Require(at != parts.end() && at->part_id == solid.part_id &&
        at->section_id == material.section_id && at->material_id == material.material_id,
        "Main support effective property and material identities differ");
    n::NativeSolidMainCoefficientInput value;
    value.face = n::MainFaceKind::OrdinaryExterior;
    value.layout = n::SolidLayout::EightSlot;
    value.incompressibility_control = at->effective_control ? 1 : 0;
    value.scale = 1.;
    value.fill = 1.; // Closed direct-key source: LECFILL default, no INIBRI/FILL writer.
    value.area = geometry.area;
    value.volume = geometry.signed_volume; // Preserve pre-INITIA sign; never ABS.
    value.bulk = material.pm32;
    value.controlled_bulk = material.pm107;
    return value;
}
}
Forecast SelectedShellMainSource::Preflight(const c::CorrectedNodalSource& source,
    const modelio::self_contact::OriginalSelection& selection, Limits limits) {
    return d::Budget(source, selection, limits);
}
Preparation SelectedShellMainSource::Prepare(const c::CorrectedNodalSource& source,
    const modelio::self_contact::OriginalSelection& selection, const std::string& member, const std::string& combine_member, Limits limits) {
    Preparation result;
    try {
        const auto forecast = Preflight(source, selection, limits);
        const bool grouping = d::GroupingContext(selection, combine_member);
        auto next = std::make_shared<Data>(source, selection, forecast);
        const auto& model = source.pre_correction().physical();
        const auto input = coated::detail::PrepareInputs(model, selection, member, {}, limits.coating);
        const auto classified = coated::Classify(input);
        if (!classified.contact_complete)
            d::Reject(Status::UnsupportedSource, "Selected coating support is ambiguous or unsupported");
        const auto order = coated::SurfaceOrder(input, classified);
        d::Require(order.primary_to_physical.size() == selection.data().counts.retained_shells,
            "Selected main roster changed during native surface ordering");
        const auto& canonical = selection.canonical().data();
        const auto& declared = selection.data();
        const native::Provenance bound{canonical.inputs.canonical_manifest, canonical.inputs.scope_report,
            canonical.inputs.source_member, declared.auxiliary_sha256, declared.combine_sha256, canonical.inputs.units};
        next->provenance.units = input.units;
        next->provenance.input_digest = coated::detail::InputDigest(input, classified, &order, {},
            coated::detail::SourceBinding(bound), limits.metadata_bytes).sha256;
        next->provenance.property_digest = source.provenance().property_digest;
        next->provenance.material_digest = source.provenance().material_digest;
        next->provenance.import_digest = source.provenance().source_digest;
        coated::detail::TopologyInput packed_topology(input, classified, order);
        if (!next->arena.Initialize(forecast.coating.topology.output_bytes))
            d::Reject(Status::ResourceLimit, "Main topology output allocation failed");
        {
            tl::util::HostArena scratch;
            if (!scratch.Initialize(forecast.coating.topology.scratch_bytes))
                d::Reject(Status::ResourceLimit, "Main topology scratch allocation failed");
            next->topology_report = coated::s::BuildStarter(packed_topology.View(), limits.coating.topology,
                next->arena, scratch, &next->topology);
        }
        if (next->topology_report.status != coated::s::Status::Ok) {
            const auto p = next->topology_report.primary;
            const auto eid = p < order.primary_to_physical.size()
                ? input.shells[order.primary_to_physical[p]].primary.source_id : 0;
            d::Reject(next->topology_report.status == coated::s::Status::ResourceLimit ? Status::ResourceLimit :
                Status::UnsupportedSource, "Native shell topology rejected complete selected source", p, eid);
        }
        next->provenance.topology_digest = coated::detail::OutputDigest(next->topology,
            next->provenance.input_digest, limits.metadata_bytes).sha256;
        const auto operands = d::PackShells(source, input, limits);
        RecordDuplicates(operands, next->certificate);
        const auto count = order.primary_to_physical.size();
        next->bindings.reserve(count);
        next->possible_owners.reserve(input.shells.size());
        next->coefficients.resize(next->topology.main_count);
        for (std::size_t primary = 0; primary < count; ++primary) {
            const auto physical = order.primary_to_physical[primary];
            const auto& shell = input.shells[physical];
            const auto& main = next->topology.mains[primary];
            const auto& role = classified.roles[physical];
            const auto support = d::SelectSupport(input, operands, main, physical, grouping);
            PrimaryBinding binding;
            binding.contact_element = shell.primary.source_id;
            binding.contact_part = shell.part_id;
            binding.contact_physical = physical;
            binding.role = role.state;
            binding.partner = next->topology.primary_to_partner[primary];
            binding.owner = support.proof;
            binding.winner_begin = next->possible_owners.size();
            binding.winner_count = support.winners.size();
            d::Require(binding.partner > count && binding.partner <= next->topology.main_count &&
                next->topology.expanded_to_primary[binding.partner - 1] == primary,
                "Native shell partner mapping differs from its physical parent");
            std::optional<n::NativeSolidMainCoefficientInput> solid_values;
            if (role.state != coated::RoleState::Ordinary) {
                d::Require(role.matches == 1 && role.first_solid < input.solids.size(),
                    "Selected coating has no unique genuine solid support");
                const auto& solid = input.solids[role.first_solid];
                binding.solid_element = solid.source_id;
                binding.solid_part = solid.part_id;
                solid_values = SolidValues(source, input, solid, main, shell.primary.layout, primary, next->certificate);
                ++next->certificate.coated;
            }
            std::optional<d::PrimaryValues> value;
            for (const auto candidate : support.winners) {
                const auto& parent = input.shells[candidate];
                const auto& part = operands.parts.at(operands.shells.at(candidate).part);
                const auto evaluated = d::EvaluateValues(part.coefficient, parent.primary.layout,
                    solid_values ? &*solid_values : nullptr);
                if (!value) value = evaluated;
                // If ownership is known, use exactly its source operand result.
                // Otherwise every admissible winner must publish equal bits.
                if (support.owner == SIZE_MAX && !d::SameValues(*value, evaluated))
                    d::Reject(Status::NeedsNativeShellOrder, "Native tied shell owners yield different coefficient bits",
                        primary, binding.contact_element, input.shells[support.winners.front()].primary.source_id,
                        parent.primary.source_id);
                if (candidate == support.owner) {
                    value = evaluated;
                    binding.support_element = parent.primary.source_id;
                    binding.support_part = parent.part_id;
                    binding.support_physical = candidate;
                }
                next->possible_owners.push_back({parent.primary.source_id, parent.part_id, candidate});
            }
            d::Require(value.has_value(), "Missing native coefficient support result");
            next->coefficients[primary] = value->primary;
            next->coefficients[binding.partner - 1] = value->partner;
            switch (binding.owner) {
            case OwnerProof::UniqueBest: ++next->certificate.unique_owners; break;
            case OwnerProof::NativeCornerOrder: ++next->certificate.corner_owners; break;
            case OwnerProof::NativeMaterialGroupOrder: ++next->certificate.material_group_owners; break;
            case OwnerProof::UnresolvedEqualValues: ++next->certificate.unresolved_owners; break;
            }
            next->bindings.push_back(binding);
        }
        d::Require(next->possible_owners.size() <= input.shells.size() && next->possible_owners.capacity() <= 2*input.shells.size() &&
            next->bindings.capacity() <= 2*count && next->coefficients.capacity() <= 2*next->topology.main_count,
            "Published main-source vectors exceed their admitted capacities");
        next->certificate.primaries = count;
        next->certificate.grouping_controls_certified = grouping;
        next->certificate.orientation_identity = true;
        next->certificate.owners_complete = next->certificate.unresolved_owners == 0;
        next->provenance.coefficient_digest = d::Digest(next->provenance, next->certificate,
            next->bindings, next->possible_owners, next->coefficients, limits.metadata_bytes);
        result.report.status = Status::CoefficientValuesReady;
        result.report.reason = next->certificate.owners_complete
            ? "Complete selected shell K and mechanical support owners are source certified"
            : "Complete selected shell K values; tied mechanical ownership requires native shell order";
        result.source.emplace(SelectedShellMainSource(std::move(next)));
    } catch (const d::Failure& failure) {
        result.source.reset();
        result.report = failure.report;
    } catch (const std::bad_alloc&) {
        result.source.reset();
        result.report = {Status::ResourceLimit, "Selected main source allocation failed"};
    } catch (const std::exception& error) {
        result.source.reset();
        result.report = {Status::InvalidInput, std::string(error.what()).substr(0, 1024)};
    }
    return result;
}
const c::CorrectedNodalSource& SelectedShellMainSource::corrected() const noexcept { return data_->corrected; }
const modelio::self_contact::OriginalSelection& SelectedShellMainSource::selection() const noexcept { return data_->selection; }
const coated::s::Snapshot& SelectedShellMainSource::topology() const noexcept { return data_->topology; }
const coated::s::Report& SelectedShellMainSource::topology_report() const noexcept { return data_->topology_report; }
tl::util::ConstView<double> SelectedShellMainSource::coefficients() const noexcept {
    return {data_->coefficients.data(), data_->coefficients.size()};
}
tl::util::ConstView<PrimaryBinding> SelectedShellMainSource::bindings() const noexcept {
    return {data_->bindings.data(), data_->bindings.size()};
}
tl::util::ConstView<CandidateOwner> SelectedShellMainSource::possible_owners() const noexcept {
    return {data_->possible_owners.data(), data_->possible_owners.size()};
}
const Certificate& SelectedShellMainSource::certificate() const noexcept { return data_->certificate; }
const Provenance& SelectedShellMainSource::provenance() const noexcept { return data_->provenance; }
const Forecast& SelectedShellMainSource::forecast() const noexcept { return data_->forecast; }
}
