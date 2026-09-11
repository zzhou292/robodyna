#pragma once
#include "modelio/vehicle_sections/VehicleSectionResolution.h"

namespace crash::cases::vehicle_startup::detail {
// Borrowed only during the staged factory call. The published assessment retains
// the corresponding immutable handle and its shared canonical source backing.
struct DeclarationView {
    explicit DeclarationView(const modelio::vehicle::VehicleSourcePlan& value) : source(value) {}
    explicit DeclarationView(const modelio::vehicle::VehicleSectionResolution& value)
        : source(value.source()), resolution(&value) {}
    const modelio::vehicle::VehicleSourcePlan& source;
    const modelio::vehicle::VehicleSectionResolution* resolution=nullptr;
    const modelio::assembly::Material* Material(std::size_t part) const noexcept {
        return resolution ? resolution->material(part) : source.material(part);
    }
    const modelio::assembly::Section* Section(std::size_t part) const noexcept {
        return resolution ? resolution->section(part) : source.section(part);
    }
    std::size_t Available() const noexcept {
        return resolution ? resolution->counts().existing_shells+resolution->counts().failure_shells+
                            resolution->counts().glass_shells+resolution->counts().midlayer_shells+resolution->counts().rigid_shells :
                            source.counts().supported_parents;
    }
    tl::fea::ShellReferencePlacement Placement(std::size_t part) const noexcept {
        return resolution ? resolution->parts()[part].placement : tl::fea::ShellReferencePlacement::Centered;
    }
    const modelio::vehicle::NativeParentMapping* Mapping(std::size_t parent) const noexcept {
        return resolution ? resolution->native_mapping(parent) : nullptr;
    }
    std::size_t SourceBound() const noexcept {
        // The resolution's bound already includes the historical plan once.
        return resolution ? resolution->startup_budget_bytes() : source.startup_budget_bytes();
    }
};
} // namespace crash::cases::vehicle_startup::detail
