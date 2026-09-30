#include "VehicleSelfContactSetup.h"

#include "modelio/physical_domain/Policy.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact {
namespace {

void CheckSource(const vehicle_runtime::Execution& execution,
                 const vehicle_runtime::Attachments& attachments,
                 const modelio::self_contact::OriginalSelection& original) {
    using output::Require;
    const auto& model = execution.model();
    const auto& physical = execution.physical();
    const auto& attachment_model = attachments.attachments().model();
    Require(model.source_domain().policy() ==
            modelio::physical_domain::Policy::
                RetainedShellAssembliesVehicleSupportsV5,
        "Vehicle self-contact setup requires the actual V5 physical policy");
    Require(model.SharesStorage(attachments.physical()),
        "Self-contact execution and attachments must retain one physical model");
    Require(physical.prepared() && physical.execution() &&
            physical.catalog() &&
            physical.catalog()->execution_sections() &&
            physical.domain()->SharesStorage(
                model.source_domain().domain()) &&
            physical.coefficients()->nodes().data() ==
                model.coefficients().nodes().data() &&
            physical.execution()->rigid()->groups().data() ==
                model.rigid_assembly().groups().data() &&
            physical.execution()->rigid()->members().data() ==
                model.rigid_assembly().members().data() &&
            attachment_model.domain()->SharesStorage(*physical.domain()),
        "Self-contact physical/catalog/rigid/CIN identities differ");
    Require(&execution.resolution() ==
                model.shell_source().references().resolution() &&
            &attachments.witnesses().binding().references().source()
                    .canonical().data() ==
                &model.shell_source().references().source()
                    .canonical().data() &&
            &attachments.witnesses().binding().shells() ==
                &model.shell_source().shells() &&
            attachments.witnesses().runtime_mappable(),
        "Self-contact execution and complete CIN witness source differ");
    Require(&original.canonical().data() ==
                &execution.resolution().source().canonical().data() &&
            &original.canonical().data() ==
                &model.source_domain().source().tied_source()
                    .canonical().data(),
        "Original self-contact selection does not share canonical authority");
}

contact::SelfContactActiveUseSource Support(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments) noexcept {
    const auto& roster = attachments.witnesses().data();
    return {&execution.model().rigid_assembly(),
        {&attachments.attachments().model(), roster.ranges.data(),
         roster.witnesses.data(), roster.ranges.size(),
         roster.witnesses.size()}};
}

std::size_t Add(std::initializer_list<std::size_t> values,
                std::size_t cap, const char* message) {
    tl::util::BoundedArenaLayout sum(cap);
    tl::util::ArenaRegion unused;
    for (const auto value : values)
        output::Require(sum.Append<std::byte>(value, unused), message);
    return sum.bytes();
}

std::size_t Difference(std::size_t total,
                       std::initializer_list<std::size_t> values,
                       const char* message) {
    for (const auto value : values) {
        output::Require(value <= total, message);
        total -= value;
    }
    return total;
}

std::size_t SharedSourceReservation(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,
    std::size_t cap) {
    const auto& model = execution.model().forecast();
    const auto& cin = attachments.attachments().forecast();
    const auto& roster = attachments.witnesses().data();
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion unused;
    for (const auto bytes : {
             model.shell_source, model.physical_source,
             model.producer_source,
             execution.physical().owned_payload_bytes(),
             execution.model().solids().owned_payload_bytes(),
             cin.total_host_bytes,
             attachments.witnesses().forecast().fixed_bytes,
             attachments.forecast().fixed_bytes,
             execution.forecast().fixed_bytes, std::size_t{4096}})
        output::Require(budget.Append<std::byte>(bytes, unused),
            "Retained self-contact app source graph exceeds host cap");
    output::Require(
        budget.Append<vehicle_startup::cin_stage::WitnessRange>(
            roster.ranges.capacity(), unused) &&
        budget.Append<vehicle_startup::cin_stage::ActiveWitness>(
            roster.witnesses.capacity(), unused) &&
        budget.Append<vehicle_startup::TiedCinWitnessOrigin>(
            roster.origins.capacity(), unused),
        "Retained self-contact CIN roster exceeds host cap");
    return budget.bytes();
}

std::size_t SelectedSharedReservation(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,
    std::size_t cap) {
    const auto& physical = execution.physical();
    const auto& cin_model = attachments.attachments().model();
    const auto cin = cin_model.forecast();
    output::Require(cin.model_payload_bytes >=
            sizeof(tl::constraints::tied_shell::TiedCinAttachmentModel),
        "Retained selected CIN model reservation is invalid");
    auto retained_cin = cin.model_payload_bytes -
        sizeof(tl::constraints::tied_shell::TiedCinAttachmentModel);
    if (!cin_model.domain()->SharesStorage(*physical.domain()))
        retained_cin = Add({retained_cin, cin.domain_payload_bytes}, cap,
            "Retained selected CIN domain reservation exceeds host cap");
    retained_cin = Add(
        {retained_cin, cin.post_kinchk_payload_bytes}, cap,
        "Retained selected CIN reservation exceeds host cap");
    // The active-use binding authenticates the execution's already-retained
    // rigid assembly, so its incremental rigid reservation is zero.
    return Add({physical.owned_payload_bytes(), retained_cin}, cap,
        "Selected shared-source reservation exceeds host cap");
}

void AdmitOuterBeforeSelected(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,
    const modelio::self_contact::OriginalSelection& original,
    std::size_t fixed_bytes, SetupLimits& limits) {
    output::Require(limits.host_bytes &&
            limits.host_bytes <= (std::size_t{40} << 30),
        "Invalid vehicle self-contact setup host cap");
    const auto shared =
        SharedSourceReservation(execution, attachments, limits.host_bytes);
    const auto base = Add({shared, original.data().owned_payload_bytes,
            fixed_bytes, std::size_t{256}},
        limits.host_bytes,
        "Retained vehicle source leaves no selected-source capacity");
    const auto selected_shared = SelectedSharedReservation(
        execution, attachments, limits.host_bytes);
    const auto selected_outer_cap = Add(
        {limits.host_bytes - base, selected_shared}, limits.host_bytes,
        "Mapped selected-source outer cap overflows");
    limits.selected.host_bytes =
        std::min(limits.selected.host_bytes, selected_outer_cap);
}

SetupForecast ComposeForecast(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,
    const modelio::self_contact::OriginalSelection& original,
    SourceForecast selected, SetupLimits limits,
    std::size_t fixed_bytes) {
    output::Require(limits.host_bytes &&
            limits.host_bytes <= (std::size_t{40} << 30),
        "Invalid vehicle self-contact setup host cap");
    SetupForecast result;
    result.selected = selected;
    result.shared_vehicle_source_reservation_bytes =
        SharedSourceReservation(execution, attachments, limits.host_bytes);
    result.retained_original_selection_reservation_bytes =
        original.data().owned_payload_bytes;
    result.selected_incremental_reservation_bytes = Difference(
        selected.retained_reservation_bytes,
        {selected.shared_physical_reservation_bytes,
         selected.shared_rigid_reservation_bytes,
         selected.shared_cin_reservation_bytes},
        "Selected self-contact shared-source forecast is invalid");
    result.retained_setup_reservation_bytes = Add(
        {result.shared_vehicle_source_reservation_bytes,
         result.retained_original_selection_reservation_bytes,
         result.selected_incremental_reservation_bytes, fixed_bytes,
         std::size_t{256}},
        limits.host_bytes,
        "Vehicle self-contact retained setup exceeds host cap");
    result.peak_temporary_reservation_bytes =
        selected.peak_temporary_reservation_bytes;
    result.peak_host_reservation_bytes = Add(
        {result.retained_setup_reservation_bytes,
         result.peak_temporary_reservation_bytes},
        limits.host_bytes,
        "Vehicle self-contact setup peak exceeds host cap");
    return result;
}

}  // namespace

struct VehicleSelfContactSetup::Data {
    Data(const vehicle_runtime::Execution& selected_execution,
         const vehicle_runtime::Attachments& selected_attachments,
         const modelio::self_contact::OriginalSelection& selected_original,
         SelectedSelfContactSource selected_source,
         SetupForecast selected_forecast)
        : execution(selected_execution), attachments(selected_attachments),
          original(selected_original), source(selected_source),
          forecast(selected_forecast) {}
    vehicle_runtime::Execution execution;
    vehicle_runtime::Attachments attachments;
    modelio::self_contact::OriginalSelection original;
    SelectedSelfContactSource source;
    SetupForecast forecast;
};

SetupForecast VehicleSelfContactSetup::Preflight(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,
    const modelio::self_contact::OriginalSelection& original,
    Config config, SetupLimits limits) {
    CheckSource(execution, attachments, original);
    AdmitOuterBeforeSelected(execution, attachments, original,
        sizeof(Data) + sizeof(VehicleSelfContactSetup), limits);
    const auto selected = SelectedSelfContactSource::Preflight(
        execution.physical(), original.data(),
        Support(execution, attachments), config, limits.selected);
    return ComposeForecast(execution, attachments, original, selected,
        limits, sizeof(Data) + sizeof(VehicleSelfContactSetup));
}

VehicleSelfContactSetup VehicleSelfContactSetup::Prepare(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,
    const modelio::self_contact::OriginalSelection& original,
    Config config, SetupLimits limits) {
    CheckSource(execution, attachments, original);
    AdmitOuterBeforeSelected(execution, attachments, original,
        sizeof(Data) + sizeof(VehicleSelfContactSetup), limits);
    const auto support = Support(execution, attachments);
    auto selected = SelectedSelfContactSource::Prepare(
        execution.physical(), original.data(), support,
        config, limits.selected);
    const auto forecast = ComposeForecast(
        execution, attachments, original, selected.forecast(),
        limits, sizeof(Data) + sizeof(VehicleSelfContactSetup));
    const auto& active = selected.active_uses();
    const auto* rigid = active.rigid();
    const auto cin = active.cin();
    output::Require(rigid &&
            rigid->groups().data() ==
                execution.model().rigid_assembly().groups().data() &&
            rigid->members().data() ==
                execution.model().rigid_assembly().members().data() &&
            cin.model && cin.model->rows().data ==
                attachments.attachments().model().rows().data &&
            cin.range_count ==
                attachments.witnesses().data().ranges.size() &&
            cin.witness_count ==
                attachments.witnesses().data().witnesses.size() &&
            selected.MatchesPhysical(execution.physical()),
        "Prepared self-contact setup lost actual rigid/CIN/physical identity");
    auto next = std::make_shared<Data>(
        execution, attachments, original, selected, forecast);
    return VehicleSelfContactSetup(std::move(next));
}

bool VehicleSelfContactSetup::SharesStorage(
    const VehicleSelfContactSetup& other) const noexcept {
    return data_ && data_ == other.data_;
}

bool VehicleSelfContactSetup::MatchesSource(
    const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,
    const modelio::self_contact::OriginalSelection& original) const noexcept {
    return data_ &&
        &data_->execution.physical() == &execution.physical() &&
        &data_->attachments.witnesses() == &attachments.witnesses() &&
        &data_->original.data() == &original.data();
}

const vehicle_runtime::Execution&
VehicleSelfContactSetup::execution() const noexcept {
    return data_->execution;
}

const vehicle_runtime::Attachments&
VehicleSelfContactSetup::attachments() const noexcept {
    return data_->attachments;
}

const modelio::self_contact::OriginalSelection&
VehicleSelfContactSetup::original() const noexcept {
    return data_->original;
}

const tl::fea::ShellPhysicalBinding&
VehicleSelfContactSetup::physical() const noexcept {
    return data_->execution.physical();
}

const SelectedSelfContactSource&
VehicleSelfContactSetup::selected() const noexcept {
    return data_->source;
}

const Config& VehicleSelfContactSetup::config() const noexcept {
    return data_->source.config();
}

const SetupForecast& VehicleSelfContactSetup::forecast() const noexcept {
    return data_->forecast;
}

const SourceInventory&
VehicleSelfContactSetup::inventory() const noexcept {
    return data_->source.inventory();
}

const SourceCounts& VehicleSelfContactSetup::counts() const noexcept {
    return data_->source.census().source;
}

const Census& VehicleSelfContactSetup::census() const noexcept {
    return data_->source.census();
}

const contact::SelfContactSurfaceBinding&
VehicleSelfContactSetup::surface() const noexcept {
    return data_->source.surface();
}

const contact::FixedContactFacetBinding&
VehicleSelfContactSetup::facets() const noexcept {
    return data_->source.facets();
}

const contact::SelfContactActiveUseBinding&
VehicleSelfContactSetup::active_uses() const noexcept {
    return data_->source.active_uses();
}

}  // namespace crash::cases::vehicle_self_contact
