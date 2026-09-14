#include "Source.h"
#include "case/vehicle_startup/physical_attachments/tests/PreparePost.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include <iostream>
#include <set>
#include <stdexcept>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
NativeParent Native(const fe::ShellPhysicalBinding& physical, const fe::ShellPlasticityParentInput& p) {
    const auto& shells = *physical.shells();
    const auto i = p.family_index;
    if (p.family == fe::ShellBindingFamily::Qeph) {
        const auto& r = shells.qeph_reference(i).input;
        return {r.thickness,r.placement == fe::ShellReferencePlacement::Centered,4,shells.qeph_nodes(i).data()};
    }
    if (p.family == fe::ShellBindingFamily::T3) {
        const auto& r = shells.t3_reference(i).input;
        return {r.thickness,r.placement == fe::ShellReferencePlacement::Centered,3,shells.t3_nodes(i).data()};
    }
    if (p.family == fe::ShellBindingFamily::Qbat) {
        const auto& q = shells.qbat_reference(i);
        const auto& r = q.quadrilateral().input;
        return {r.thickness,r.placement == fe::ShellReferencePlacement::Centered &&
            q.input().options.offset_ratio == 0,4,shells.qbat_nodes(i).data()};
    }
    throw std::runtime_error("Unexpected family in authenticated shell execution");
}
std::uint64_t NativeNodeId(const fe::ShellPhysicalBinding& physical,
    const fe::ShellPlasticityParentInput& parent, unsigned local) {
    const auto& shells = *physical.shells();
    const auto index = parent.family_index;
    if (parent.family == fe::ShellBindingFamily::Qeph && local < 4)
        return shells.qeph_reference(index).input.node_ids[local];
    if (parent.family == fe::ShellBindingFamily::T3 && local < 3)
        return shells.t3_reference(index).input.node_ids[local];
    if (parent.family == fe::ShellBindingFamily::Qbat && local < 4)
        return shells.qbat_reference(index).quadrilateral().input.node_ids[local];
    throw std::runtime_error("Unexpected native shell node in source qualifier");
}
const VehicleShellExecution& Execution() {
    static const auto value = [] {
        const auto& model = physical_model::supports_test::Model();
        if (model.source_domain().policy() != modelio::physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5)
            throw std::runtime_error("Self-contact source qualifier requires the actual V5 physical policy");
        std::cout << "V5 shell execution complete preflight=" << VehicleShellExecution::Preflight(model).total_bytes << std::endl;
        return VehicleShellExecution::Prepare(model);
    }();
    return value;
}
const vehicle_runtime::Attachments& PhysicalAttachments() {
    static const auto post =
        physical_attachments::test::PreparePost(
            physical_model::supports_test::Scope(),
            physical_model::supports_test::Inputs().member);
    static const auto value = vehicle_runtime::Attachments::Prepare(
        physical_model::supports_test::Model(), post);
    return value;
}
const Selection& Inventory() {
    static const auto value = [] {
        Selection next;
        const auto& physical = Execution().physical();
        const auto count = physical.catalog()->parent_count();
        if (count > contact::SelfContactSurfaceLimits::Vehicle().max_parents)
            throw std::runtime_error("Original inventory exceeds the qualified shell cap");
        next.centered.reserve(count);
        next.excluded.reserve(count);
        for (std::size_t row = 0; row < count; ++row) {
            const auto& p = *physical.catalog()->parent(row);
            const contact::SelfContactParentSelection selected{row,p.family,p.family_index,p.source_parent_id,p.source_part_id};
            (Native(physical,p).centered ? next.centered : next.excluded).push_back(selected);
        }
        return next;
    }();
    return value;
}
const modelio::self_contact::OriginalSelection& OriginalContactSelection() {
    static const auto value = [] {
        const auto read = [](const char* variable, std::size_t bytes) {
            const auto* path = std::getenv(variable);
            if (!path || !*path)
                throw std::runtime_error("Missing original self-contact member");
            auto value = output::ReadBounded(path, bytes);
            if (value.size() != bytes)
                throw std::runtime_error("Original self-contact member extent differs");
            return value;
        };
        const auto auxiliary = read("ROBO_SELF_CONTACT_AUX_MEMBER", 44991);
        const auto combine = read("ROBO_SELF_CONTACT_COMBINE_MEMBER", 10577);
        return modelio::self_contact::OriginalSelection::Prepare(
            modelio::vehicle::test::Canonical(), auxiliary, combine);
    }();
    return value;
}
const vehicle_self_contact::VehicleSelfContactSetup& LevelZeroSetup() {
    static const auto value =
        vehicle_self_contact::VehicleSelfContactSetup::Prepare(
            Execution(), PhysicalAttachments(),
            OriginalContactSelection(), {0});
    return value;
}
const Selection& ContactInventory() {
    static const auto value = [] {
        const auto& physical = Execution().physical();
        const auto& source = OriginalContactSelection().data();
        const std::set<std::uint64_t> selected(
            source.selected_part_ids.begin(), source.selected_part_ids.end());
        Selection next;
        next.centered.reserve(source.counts.retained_shells);
        next.excluded.reserve(source.counts.retained_shells);
        for (std::size_t row = 0;
             row < physical.catalog()->parent_count(); ++row) {
            const auto& parent = *physical.catalog()->parent(row);
            if (!selected.count(parent.source_part_id)) continue;
            const contact::SelfContactParentSelection value{
                row, parent.family, parent.family_index,
                parent.source_parent_id, parent.source_part_id};
            (Native(physical, parent).centered
                ? next.centered : next.excluded).push_back(value);
        }
        if (next.centered.size() + next.excluded.size() !=
            source.counts.retained_shells)
            throw std::runtime_error(
                "Original contact set and physical shell catalog differ");
        return next;
    }();
    return value;
}
const contact::SelfContactSurfaceBinding& Surface() {
    static const auto value = [] {
        const auto& physical = Execution().physical();
        const auto input = Input(Inventory().centered);
        const auto limits = contact::SelfContactSurfaceLimits::Vehicle();
        const auto preflight = contact::SelfContactSurfaceBinding::Preflight(physical,input,limits);
        if (preflight.report.status != contact::SelfContactSurfaceStatus::Ok)
            throw std::runtime_error(preflight.report.message);
        std::cout << "Self-contact centered subset parents=" << input.parent_count
            << " excluded_offsets=" << Inventory().excluded.size()
            << " arena=" << preflight.forecast.arena_bytes
            << " source=" << preflight.forecast.retained_source_bytes
            << " startup=" << preflight.forecast.startup_payload_bytes << std::endl;
        contact::SelfContactSurfaceBinding next;
        const auto report = next.Initialize(physical,input,limits);
        if (report.status != contact::SelfContactSurfaceStatus::Ok) throw std::runtime_error(report.message);
        return next;
    }();
    return value;
}
const contact::SelfContactSurfaceBinding& ContactSurface() {
    static const auto value = [] {
        const auto& physical = Execution().physical();
        const auto input = Input(ContactInventory().centered);
        const auto limits = contact::SelfContactSurfaceLimits::Vehicle();
        const auto preflight =
            contact::SelfContactSurfaceBinding::Preflight(
                physical, input, limits);
        if (preflight.report.status !=
            contact::SelfContactSurfaceStatus::Ok)
            throw std::runtime_error(preflight.report.message);
        contact::SelfContactSurfaceBinding next;
        const auto report = next.Initialize(physical, input, limits);
        if (report.status != contact::SelfContactSurfaceStatus::Ok)
            throw std::runtime_error(report.message);
        std::cout << "Original contact-set centered parents="
                  << input.parent_count
                  << " excluded_offsets=" << ContactInventory().excluded.size()
                  << " startup=" << preflight.forecast.startup_payload_bytes
                  << std::endl;
        return next;
    }();
    return value;
}
} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
