#include "lib_utils/BoundedArena.h"
#include "mixed_interface/Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::mixed_interface {
namespace d = detail;
struct MixedInterfaceSource::Data {
    explicit Data(const Initial& input) : initial(input) {}
    Initial initial;
    tl::util::HostArena interface_arena, sides_arena;
    f::Snapshot classified;
    s::MixedSidesSnapshot sides;
    std::vector<RoleObservation> roles;
    Certificate certificate;
    Provenance provenance;
    Forecast forecast;
    AdmissionCensus census;
};
Forecast MixedInterfaceSource::Preflight(const Initial& input, Limits limits) {
    return d::Budget(input, limits);
}
namespace {
Report InterfaceFailure(NumericalStage stage, f::Report report) {
    Report result;
    result.status = report.status == f::Status::ResourceLimit ? Status::ResourceLimit : Status::InvalidInput;
    result.reason = "Mixed interface numerical source stage rejected authentic packet";
    result.numerical_stage = stage;
    result.interface_report = report;
    result.raw_face = report.raw_face;
    return result;
}
Report SidesFailure(NumericalStage stage, s::Report report) {
    Report result;
    result.status = report.status == s::Status::ResourceLimit ? Status::ResourceLimit : Status::InvalidInput;
    result.reason = "Mixed shell sides numerical stage rejected classified packet";
    result.numerical_stage = stage;
    result.sides_report = report;
    return result;
}
}
Preparation MixedInterfaceSource::Prepare(const Initial& input, Limits limits) {
    Preparation result;
    try {
        const auto forecast = Preflight(input, limits);
        const auto& geometry = input.geometry();
        auto packed = d::Pack(geometry, input.selection().data().selected_part_ids, input.faces());
        // Complete physical solid membership, using the already qualified source
        // classifier. Nonselected ambiguity is retained but not consumed here.
        const auto source_roles = coated::Classify(geometry);
        const auto selected = d::SelectedRoleReport(geometry, source_roles);
        if (selected.status != Status::Ready) { result.report = selected; return result; }
        const auto packet = packed.Input();
        f::Forecast exact;
        const auto admitted = f::Preflight(packet, {}, exact);
        if (admitted.status != f::Status::Ok) {
            result.report = InterfaceFailure(NumericalStage::InterfacePreflight, admitted);
            return result;
        }
        if (exact.output_bytes > forecast.interface_output || exact.scratch_bytes > forecast.shared_scratch)
            d::Reject(Status::ResourceLimit, "Mixed interface exact forecast exceeds public reservation");
        auto next = std::make_shared<Data>(input);
        next->forecast = forecast;
        // One scratch arena is reused after each phase. The reservation covers
        // both exact interface work and the count-only sides upper bound.
        tl::util::HostArena scratch;
        s::Input upper;
        upper.profile = s::Profile::MixedSurface;
        upper.topology = s::TopologyPolicy::NativeMixedSurface;
        upper.node_count = geometry.nodes.size();
        upper.primary_count = input.faces().size();
        upper.shell_primary_count = input.census().extraction.shell_faces;
        upper.raw_origin_count = input.faces().size();
        const auto side_upper = s::PreflightMixedSides(upper);
        if (side_upper.status != s::Status::Ok || side_upper.output_bytes > forecast.sides_output ||
            side_upper.scratch_bytes > forecast.shared_scratch)
            d::Reject(Status::ResourceLimit, "Mixed sides upper forecast differs from reservation");
        const auto scratch_bytes = std::max(exact.scratch_bytes, side_upper.scratch_bytes);
        if (!next->interface_arena.Initialize(exact.output_bytes) || !scratch.Initialize(scratch_bytes))
            d::Reject(Status::ResourceLimit, "Mixed interface source arena allocation failed");
        const auto classified = f::Build(packet, {}, next->interface_arena, scratch, &next->classified);
        if (classified.status != f::Status::Ok) {
            result.report = InterfaceFailure(NumericalStage::InterfaceBuild, classified);
            return result;
        }
        const auto certified = d::Certify(geometry, packed, source_roles, input.emitted_solid_flags(),
            next->classified, next->certificate, next->roles);
        if (certified.status != Status::Ready) { result.report = certified; return result; }
        const auto sides_input = d::SideInput(packed, next->classified);
        const auto side_exact = s::PreflightMixedSides(sides_input);
        if (side_exact.status != s::Status::Ok) {
            result.report = SidesFailure(NumericalStage::SidesPreflight, {side_exact.status});
            return result;
        }
        if (side_exact.output_bytes > forecast.sides_output || side_exact.scratch_bytes > scratch.bytes())
            d::Reject(Status::ResourceLimit, "Mixed actual sides exceed admitted source capacity");
        if (!next->sides_arena.Initialize(side_exact.output_bytes))
            d::Reject(Status::ResourceLimit, "Mixed sides output allocation failed");
        const auto sides = s::BuildMixedSides(sides_input, {}, next->sides_arena, scratch, &next->sides);
        if (sides.status != s::Status::Ok) {
            result.report = SidesFailure(NumericalStage::SidesBuild, sides);
            return result;
        }
        next->provenance.source_digest = input.provenance().source_digest;
        next->provenance.initial_digest = input.provenance().output_digest;
        next->provenance.output_digest = d::Digest(next->provenance, next->classified, next->sides,
            next->roles, next->certificate, limits.metadata_bytes);
        next->census = d::Census(input);
        result.source.emplace(MixedInterfaceSource(std::move(next)));
        result.report = {Status::Ready, "Classified mixed primaries and shell sides; post-GAPM support still required"};
    } catch (const d::Failure& failure) {
        result.source.reset();
        result.report = failure.report;
    } catch (const std::bad_alloc&) {
        result.source.reset();
        result.report = {Status::ResourceLimit, "Mixed source allocation failed"};
    } catch (const std::exception& failure) {
        result.source.reset();
        result.report = {Status::InvalidInput, std::string(failure.what()).substr(0, 1024)};
    }
    return result;
}
const Initial& MixedInterfaceSource::initial() const noexcept { return data_->initial; }
const s::MixedSidesSnapshot& MixedInterfaceSource::sides() const noexcept { return data_->sides; }
const s::PrimaryFace* MixedInterfaceSource::primary() const noexcept { return data_->classified.primary; }
const std::vector<RoleObservation>& MixedInterfaceSource::raw_roles() const noexcept { return data_->roles; }
const Certificate& MixedInterfaceSource::certificate() const noexcept { return data_->certificate; }
const Provenance& MixedInterfaceSource::provenance() const noexcept { return data_->provenance; }
const Forecast& MixedInterfaceSource::forecast() const noexcept { return data_->forecast; }
const AdmissionCensus& MixedInterfaceSource::admission_census() const noexcept { return data_->census; }
}

namespace crash::cases::vehicle_self_contact::native::mixed_interface {

std::size_t MixedInterfaceSource::retained_host_upper_bound(std::size_t cap) const {
    tl::util::BoundedArenaLayout bytes(cap); tl::util::ArenaRegion unused;
    const auto add=[&](std::size_t count) { output::Require(bytes.Append<std::byte>(count,unused),"Retained mixed interface source exceeds cap"); };
    add(initial().retained_host_upper_bound(cap));add(sizeof(MixedInterfaceSource)+sizeof(Data)+4096);
    add(data_->interface_arena.bytes());add(data_->sides_arena.bytes());add(data_->roles.capacity()*sizeof(RoleObservation));
    for(const auto* text:{&data_->provenance.source_digest,&data_->provenance.initial_digest,&data_->provenance.output_digest})add(text->capacity()+1);
    return bytes.bytes();
}
}
