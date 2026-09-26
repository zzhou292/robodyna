#include "Storage.h"
#include "../Reports.h"
namespace crash::cases::vehicle_runtime {
SourceKind Source::kind() const noexcept {
    return data_->original()?SourceKind::OriginalVehicle:SourceKind::VehicleWithEnvironment;
}
const vehicle_wall::native::EnvelopeOwnerSource* Source::environment() const noexcept { return data_->environment(); }
const tl::fea::ShellPhysicalBinding& Source::physical() const noexcept {
    if(const auto* old=data_->original()) return old->execution.physical();
    return data_->environment()->physical();
}
const vehicle_startup::VehicleShellReferences& Source::vehicle_references() const noexcept {
    if(const auto* old=data_->original()) return old->execution.model().shell_source().references();
    return data_->environment()->execution_source().mechanical().vehicle_references();
}
const tl::fea::NodalCoefficientLedger& Source::coefficients() const noexcept { return *physical().coefficients(); }
const tl::fea::NodalRigidAssemblyBinding& Source::rigid() const noexcept { return *physical().execution()->rigid(); }
const tl::fea::solids::Model& Source::solids() const noexcept {
    if(const auto* old=data_->original()) return old->execution.model().solids();
    return data_->environment()->execution_source().mechanical().solids();
}
const tl::fea::type13::Model& Source::beams() const noexcept {
    if(const auto* old=data_->original()) return old->execution.model().beams();
    return data_->environment()->execution_source().mechanical().beams();
}
const tl::fea::beam18::Model* Source::structural_beams() const noexcept {
    if(const auto* old=data_->original()) return old->execution.model().structural_beams();
    return &data_->environment()->execution_source().mechanical().structural_beams();
}
const tl::fea::type45::Model* Source::joints() const noexcept {
    if(const auto* old=data_->original()) return old->joints?&old->joints->model():nullptr;
    return &data_->environment()->joints();
}
const vehicle_startup::TiedCinAttachments& Source::cin() const noexcept {
    if(const auto* old=data_->original()) return old->attachments.attachments();
    return data_->environment()->attachments();
}
const vehicle_startup::TiedCinWitnessRoster& Source::witnesses() const noexcept {
    if(const auto* old=data_->original()) return old->attachments.witnesses();
    return data_->environment()->witnesses();
}
tl::fea::NodalCinWitnessSource Source::witness_source() const noexcept {
    const auto& rows=witnesses().data();
    return {&cin().model(),rows.ranges.data(),rows.witnesses.data(),rows.ranges.size(),rows.witnesses.size()};
}
tl::fea::ShellBatchStartup Source::startup() const noexcept {
    if(const auto* next=data_->environment()) return next->startup();
    return detail::InitialTranslation();
}
SourceRoles Source::roles() const {
    if(const auto* old=data_->original()) return ResolveSourceRoles(old->attachments);
    return data_->environment()->roles();
}
detail::OwnerPacking Source::PackOwner(const SourceRoles& roles, std::size_t cap) const {
    if(const auto* next=data_->environment()) {
        output::Require(roles.node==next->roles().node,"Combined owner packing received foreign source roles");
        return next->PackOwner(cap);
    }
    return detail::PackOwner(coefficients(),rigid(),roles,startup().uniform_velocity,cap);
}
std::uint8_t Source::translation_fixed(std::size_t node) const noexcept {
    if(const auto* next=data_->environment()) return next->execution_source().mechanical().wall().translation_fixed_bits()[node];
    return 0;
}
std::uint8_t Source::rotation_fixed(std::size_t node) const noexcept {
    if(const auto* next=data_->environment()) return next->execution_source().mechanical().wall().rotation_fixed()[node];
    return 0;
}
std::size_t Source::vehicle_nodes() const noexcept {
    if(const auto* next=data_->environment()) return next->execution_source().mechanical().embedding().original().node_count();
    return physical().domain()->node_count();
}
const Execution& Source::original_execution() const {
    output::Require(data_->original()!=nullptr,"Combined runtime source has no original-only execution wrapper");
    return data_->original()->execution;
}
const Attachments& Source::original_attachments() const {
    output::Require(data_->original()!=nullptr,"Combined runtime source has no original-only attachment wrapper");
    return data_->original()->attachments;
}
}
