#pragma once
#include "modelio/vehicle_sections/VehicleSectionResolution.h"

namespace crash::cases::vehicle_startup::shell_execution::detail {
namespace fe = tl::fea;
namespace source = modelio::vehicle;
struct Packing {
    std::vector<fe::ShellPlasticityCurveInput> curves;
    std::vector<fe::ShellPlasticityMaterialInput> materials;
    std::vector<fe::ShellPlasticitySectionInput> sections;
    std::vector<fe::ShellPlasticityParentInput> parents;
    std::vector<fe::ShellFailureParentInput> failure;
    void Reserve(std::size_t parts, std::size_t parent_count, std::size_t curve_count);
    std::size_t capacity_bytes() const noexcept;
    fe::ShellBatchPlasticityBindingInput input() const noexcept;
};
// Value packing, not source admission. Complete authority is checked by the
// VehicleShellExecution factory before invoking these independently tested seams.
bool Same(const fe::ShellPlasticityMaterialInput&, const fe::ShellPlasticityMaterialInput&) noexcept;
bool Same(const fe::ShellPlasticitySectionInput&, const fe::ShellPlasticitySectionInput&) noexcept;
bool Same(const fe::ShellPlasticityParentInput&, const fe::ShellPlasticityParentInput&) noexcept;
void AddPart(Packing&, const source::PartDisposition&, const modelio::assembly::Material&,
    const modelio::assembly::Section&, const fe::ShellPlasticityMaterialInput&,
    source::SourceShellRole, fe::ShellSectionFormulation);
void PackCurves(Packing&, const std::vector<modelio::assembly::Curve>&,
                const std::vector<modelio::assembly::Curve>&);
} // namespace crash::cases::vehicle_startup::shell_execution::detail
