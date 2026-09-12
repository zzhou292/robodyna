#include "VehiclePhysicalAttachments.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_startup::physical_attachments {
struct VehiclePhysicalAttachments::Storage {
    physical_model::VehiclePhysicalModel physical;
    TiedCinWitnessRoster witnesses;
    Forecast forecast;
    Storage(const physical_model::VehiclePhysicalModel& p, const TiedCinWitnessRoster& w, Forecast f)
        : physical(p), witnesses(w), forecast(f) {}
};
Forecast VehiclePhysicalAttachments::Preflight(const physical_model::VehiclePhysicalModel& physical,
                                               const TiedSearchPostKinChk& post, Limits limits) {
    using output::Require;
    const Limits hard;
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
        limits.attachments.host_bytes && limits.attachments.host_bytes <= hard.attachments.host_bytes &&
        limits.witnesses.host_bytes && limits.witnesses.host_bytes <= hard.witnesses.host_bytes,
        "Invalid complete vehicle attachment limits");
    const auto& declaration = post.classification().context().auxiliary().declaration();
    Require(&declaration.canonical().data() == &physical.shell_source().references().source().canonical().data(),
        "Physical CIN source must retain the same original canonical authority");
    const auto& domain = physical.source_domain().domain();
    const auto mapped = TiedCinAttachments::Forecast(post, domain, limits.attachments);
    const auto policy = physical.source_domain().policy();
    Require(((policy == modelio::physical_domain::Policy::RetainedShellAssembliesV1 && domain.node_count() == 372435) ||
        policy == modelio::physical_domain::Policy::RetainedShellAssembliesExtendedSolidsV4) &&
        post.result().slaves().count == 11165,
        "Physical CIN requires the complete retained vehicle scope");
    Forecast f;
    f.physical_source = physical.forecast().total_bytes;
    f.attachment_reservation = mapped.total_host_bytes;
    // Charge the full witness cap before constructing either intermediate.
    // It includes its retained attachments and shell backing. Double-counting
    // shared upstream values is conservative and needs no private size access.
    f.witness_reservation = limits.witnesses.host_bytes;
    f.fixed_bytes = sizeof(Storage) + sizeof(VehiclePhysicalAttachments) + 64;
    tl::util::BoundedArenaLayout all(limits.host_bytes);
    tl::util::ArenaRegion ignored;
    for (auto bytes : {f.physical_source, f.attachment_reservation, f.witness_reservation, f.fixed_bytes})
        Require(all.Append<unsigned char>(bytes, ignored), "Complete physical CIN composition exceeds host cap");
    f.total_bytes = all.bytes();
    return f;
}
VehiclePhysicalAttachments VehiclePhysicalAttachments::Prepare(const physical_model::VehiclePhysicalModel& physical,
                                                                const TiedSearchPostKinChk& post, Limits limits) {
    using output::Require;
    const auto forecast = Preflight(physical, post, limits);
    const auto cin = TiedCinAttachments::Prepare(post, physical.source_domain().domain(), limits.attachments);
    // This explicit original profile has no CIN/rigid overlap. A change in
    // that source relationship needs its actual ordered runtime treatment.
    const auto rows = cin.model().rows();
    for (std::size_t i = 0; i < rows.count; ++i) {
        const auto& row = rows.data[i];
        Require(!physical.rigid_assembly().FindMember(row.secondary_domain_node),
            "Retained CIN secondary also belongs to a rigid assembly");
        for (auto node : row.master_domain_nodes)
            Require(!physical.rigid_assembly().FindMember(node),
                "Retained CIN master also belongs to a rigid assembly");
    }
    const auto witnesses = TiedCinWitnessRoster::Prepare(cin, physical.shell_source(), limits.witnesses);
    Require(witnesses.runtime_mappable(), "Complete physical CIN shell support remains unresolved");
    return VehiclePhysicalAttachments(std::make_shared<Storage>(physical, witnesses, forecast));
}
const physical_model::VehiclePhysicalModel& VehiclePhysicalAttachments::physical() const noexcept { return storage_->physical; }
const TiedCinWitnessRoster& VehiclePhysicalAttachments::witnesses() const noexcept { return storage_->witnesses; }
const TiedCinAttachments& VehiclePhysicalAttachments::attachments() const noexcept { return storage_->witnesses.attachments(); }
const Forecast& VehiclePhysicalAttachments::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::cases::vehicle_startup::physical_attachments
