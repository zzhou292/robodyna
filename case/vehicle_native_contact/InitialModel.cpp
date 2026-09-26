#include "InitialModel.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <cmath>
namespace crash::cases::vehicle_native_contact::detail {
namespace c = vehicle_self_contact::native::initial_controls;
namespace {
using Role = vehicle_dynamics::native_contact::Role;
initial::Controls Controls(const c::Controls& source) {
    initial::Controls out;
    out.level = source.level;
    out.gap_mode = source.gap_mode;
    out.initial_penetration = source.initial_penetration;
    out.damping_flag = source.damping_flag;
    out.sharp = source.sharp;
    out.arithmetic_precision = source.arithmetic_precision;
    out.partitions = source.partitions;
    out.starter_workers = source.starter_workers;
    out.edge_mode = source.edge_mode;
    out.thermal = source.thermal;
    out.neighbor_removal = source.neighbor_removal;
    out.tied_removal = source.tied_removal;
    out.thickness_update = source.thickness_update;
    out.stiffness_formulation = source.stiffness_formulation;
    out.stiffness_mass_update = source.stiffness_mass_update;
    out.gap_load_cards = source.gap_load_cards;
    out.base_multiplier = source.base_multiplier;
    out.drad = source.drad;
    out.gap_load = source.gap_load;
    out.native_voxel_capacity = source.native_voxel_capacity;
    out.curvature = source.curvature;
    out.native_packet_size = static_cast<int>(source.native_packet_size);
    return out;
}
std::size_t SolidCount(const tl::fea::solids::Model& model) {
    return model.solid18().size() + model.solid24().size() + model.solid6z().size() +
        model.solid18_law44().size() + model.solid18_law90().size();
}
}
InitialModelForecast InitialModel::Preflight(const SourceInputs& in, std::size_t cap) {
    const auto& geometry = in.self.main_source().mixed().initial().geometry();
    const auto& model = in.owner.execution_source().mechanical();
    output::Require(cap && cap <= (128u << 20) && geometry.solids.size() <= 16384 &&
                        in.controls.interfaces().size() <= 1024 &&
                        geometry.solids.size() == SolidCount(model.solids()) &&
                        geometry.shells.size() + 1 == model.shells().qeph_count() +
                            model.shells().t3_count() + model.shells().qbat_count(),
                    "Initializer complete physical family census differs from the actual owner source");
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion unused;
    output::Require(budget.Append<initial::EightSlotSolid>(2 * geometry.solids.size(), unused) &&
                        budget.Append<initial::InterfaceIdentity>(2 * in.controls.interfaces().size(), unused) &&
                        budget.Append<tied_removal::Interface>(2, unused) &&
                        budget.Append<std::byte>(sizeof(InitialModel) + 256, unused),
                    "Initializer common source descriptors exceed their capacity");
    return {budget.bytes()};
}
InitialModel InitialModel::Prepare(const SourceInputs& in, const TiedRemovalSource& tied, std::size_t cap) {
    InitialModel result;
    result.forecast_ = Preflight(in, cap);
    output::Require(tied.attachments().model().SharesStorage(in.owner.attachments().model()),
                    "Initializer TYPE2 roster belongs to a different actual owner");
    const auto& geometry = in.self.main_source().mixed().initial().geometry();
    result.solids_.reserve(geometry.solids.size());
    for (const auto& source : geometry.solids) {
        output::Require(source.phase == vehicle_self_contact::native::coated::PacketPhase::ReaderBeforeInitia,
                        "Initializer solid is not the authentic pre-INITIA raw8 packet");
        initial::EightSlotSolid solid;
        solid.native_source_id = source.source_id;
        solid.part_source_id = source.part_id;
        std::copy(source.nodes.begin(), source.nodes.end(), solid.nodes);
        for (auto node : solid.nodes)
            output::Require(node < geometry.nodes.size(), "Initializer raw solid node leaves its authentic original prefix");
        result.solids_.push_back(solid);
    }
    result.interfaces_.reserve(in.controls.interfaces().size());
    for (const auto& source : in.controls.interfaces()) {
        initial::InterfaceIdentity row;
        row.source_id = source.native_id;
        row.native_storage_ordinal = source.native_storage_ordinal;
        row.kind = source.kind == c::InterfaceKind::Type2 ? initial::InterfaceKind::Type2 : initial::InterfaceKind::Type25;
        row.origin = source.origin == c::InterfaceOrigin::OriginalDefinition ?
            initial::InterfaceOrigin::OriginalDefinition : initial::InterfaceOrigin::DeclaredAdditionalInterface;
        result.interfaces_.push_back(row);
    }
    const auto ties = tied.interfaces();
    output::Require(ties.size() == 1, "Complete initializer TYPE2 scope differs from the actual owner");
    result.tied_.assign(ties.begin(), ties.end());
    output::Require(result.solids_.capacity() <= 2 * geometry.solids.size() &&
                        result.interfaces_.capacity() <= 2 * in.controls.interfaces().size() &&
                        result.tied_.capacity() <= 2,
                    "Actual initializer common source capacity exceeds forecast");
    return result;
}
initial::Input InitialModel::input(const SourceInputs& source, Role role, const InterfaceFields& fields) const {
    output::Require(role == Role::Self || role == Role::MeshWall, "Unknown initializer case role");
    const bool self = role == Role::Self;
    initial::Input out;
    out.phase = initial::Phase::StarterNormalsAndPreBucGaps;
    out.units = source.self.main_source().provenance().units;
    const auto* controls = self ? &source.controls.controls() : source.controls.wall_controls();
    output::Require(controls != nullptr, "Declared wall initializer controls are unavailable");
    out.controls = Controls(*controls);
    out.native_interface_id = self ? source.controls.self_interface_id() : source.controls.wall_interface_id();
    out.mesh = self ? source.self.startup_input() : source.wall.startup_input();
    out.starter = self ? source.self.snapshot() : source.wall.starter();
    out.contact = fields.starter_view();
    out.stamp.source = out.native_interface_id;
    out.stamp.topology = out.mesh.source_generation;
    out.stamp.physical_domain = source.owner.physical().domain()->source_instance_id();
    out.interface_phase = initial::InterfaceCensusPhase::CompleteOriginalAndDeclaredAdditions;
    out.interfaces = interfaces_.data();
    out.interface_count = interfaces_.size();
    const auto main_nodes = self ? source.self.main_source().main_nodes() : source.wall.main_nodes();
    out.main_nodes = main_nodes.data();
    out.main_node_count = main_nodes.size();
    const auto gaps = fields.main_search_gaps();
    out.main_search_gap = gaps.data();
    out.main_search_gap_count = gaps.size();
    // The wall reduction comes from its own complete shell-mask/beam/spring
    // producer; the original self scalar is never reused for this new interface.
    out.global_search_gap = self ? source.controls.gaps().global_search_gap :
        source.wall.gap_report().maximum_secondary + source.wall.wall().geometry().native_half_gap;
    output::Require(std::isfinite(out.global_search_gap) && out.global_search_gap >= 0,
                    "Interface-specific source global GAP is unrepresentable");
    out.solid_scope = solids_.empty() ? initial::SolidScope::ExplicitNoSolids : initial::SolidScope::CompleteEightSlotModel;
    out.solids = solids_.empty() ? nullptr : solids_.data();
    out.solid_count = solids_.size();
    const auto& model = source.owner.execution_source().mechanical();
    auto& census = out.contributors;
    census.census = native::search_startup::Census::CompleteDeclaredModel;
    census.physical_nodes = model.domain().node_count();
    census.physical_shells = model.shells().qeph_count() + model.shells().t3_count() + model.shells().qbat_count();
    census.rigid_bodies = model.rigid_assembly().groups().size();
    census.cin_links = source.owner.attachments().model().rows().count;
    census.tied_interfaces = tied_.size();
    census.other_interfaces = interfaces_.size() - tied_.size() - 1;
    // The typed complete model admits only the supported shell/beam/connector
    // and EightSlot families checked above. It does not drop a physical row.
    census.unsupported_elements = 0;
    const auto& population = source.controls.native_population();
    output::Require(!population.exact_count_available && population.lower >= census.physical_nodes,
                    "Native population bound does not cover the complete physical domain");
    out.native_population = {native::search_startup::NativePopulationPolicy::CompleteModelMultiplierTier,
        population.lower, population.upper};
    census.native_auxiliary_nodes = SIZE_MAX; // Explicit unavailable exact count.
    out.tied_phase = tied_removal::Finalization::CompactedAfterKinChk;
    out.tied_interfaces = tied_.data();
    out.tied_interface_count = tied_.size();
    return out;
}
} // namespace crash::cases::vehicle_native_contact::detail
