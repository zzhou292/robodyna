#include "Packing.h"
#include "modelio/source_assembly/NativeDeclarationViews.h"
#include <algorithm>

namespace crash::cases::vehicle_startup::shell_execution::detail {
namespace {
using output::Require;
bool Bits(double a, double b) noexcept { return output::Bits(a) == output::Bits(b); }
template<class T, class Id>
const T* Find(const std::vector<T>& values, std::uint64_t id, Id get) {
    const auto at = std::find_if(values.begin(), values.end(), [&](const auto& value) { return get(value) == id; });
    return at == values.end() ? nullptr : &*at;
}
}
bool Same(const fe::ShellPlasticityMaterialInput& a, const fe::ShellPlasticityMaterialInput& b) noexcept {
    return a.material_id == b.material_id && a.curve_id == b.curve_id && a.law == b.law &&
        a.hardening == b.hardening && a.continuation == b.continuation &&
        Bits(a.young_pa, b.young_pa) && Bits(a.poisson_ratio, b.poisson_ratio) &&
        Bits(a.density_kg_m3, b.density_kg_m3) && a.rate.enabled == b.rate.enabled &&
        a.rate.policy == b.rate.policy && Bits(a.rate.cutoff_hz, b.rate.cutoff_hz) &&
        Bits(a.rate.cowper_symonds_c_per_s, b.rate.cowper_symonds_c_per_s) &&
        Bits(a.rate.cowper_symonds_p, b.rate.cowper_symonds_p) &&
        Bits(a.linear.initial_yield_pa, b.linear.initial_yield_pa) &&
        Bits(a.linear.tangent_modulus_pa, b.linear.tangent_modulus_pa);
}
bool Same(const fe::ShellPlasticitySectionInput& a, const fe::ShellPlasticitySectionInput& b) noexcept {
    return a.section_id == b.section_id && Bits(a.thickness_m, b.thickness_m) &&
        a.through_thickness_points == b.through_thickness_points && a.formulation == b.formulation;
}
bool Same(const fe::ShellPlasticityParentInput& a, const fe::ShellPlasticityParentInput& b) noexcept {
    return a.family == b.family && a.family_index == b.family_index &&
        a.source_parent_id == b.source_parent_id && a.source_part_id == b.source_part_id &&
        a.material_id == b.material_id && a.section_id == b.section_id &&
        fe::SameShellParentExecution(a.execution,b.execution);
}
void Packing::Reserve(std::size_t parts, std::size_t count, std::size_t curve_count) {
    curves.reserve(curve_count);
    materials.reserve(parts);
    sections.reserve(parts);
    parents.reserve(count);
    failure.reserve(count);
}
std::size_t Packing::capacity_bytes() const noexcept {
    return curves.capacity() * sizeof(fe::ShellPlasticityCurveInput) +
        materials.capacity() * sizeof(fe::ShellPlasticityMaterialInput) +
        sections.capacity() * sizeof(fe::ShellPlasticitySectionInput) +
        parents.capacity() * sizeof(fe::ShellPlasticityParentInput) +
        failure.capacity() * sizeof(fe::ShellFailureParentInput);
}
fe::ShellBatchPlasticityBindingInput Packing::input() const noexcept {
    return {curves.empty() ? nullptr : curves.data(), materials.data(), sections.data(), parents.data(),
            curves.size(), materials.size(), sections.size(), parents.size()};
}
void AddPart(Packing& out, const source::PartDisposition& part,
    const modelio::assembly::Material& source_material, const modelio::assembly::Section& source_section,
    const fe::ShellPlasticityMaterialInput& supplied, source::SourceShellRole role,
    fe::ShellSectionFormulation formulation) {
    Require(part.part_id && part.material_id == source_material.id && part.material_id == supplied.material_id &&
        part.section_id == source_section.id && Bits(supplied.young_pa, source_material.young_pa) &&
        Bits(supplied.poisson_ratio, source_material.poisson_ratio) &&
        Bits(supplied.density_kg_m3, source_material.density_kg_m3), "Shell material differs from retained source identity");
    auto material = supplied;
    auto section = modelio::assembly::detail::NativeSection(source_section);
    section.formulation = formulation;
    for (double thickness : source_section.thickness_m)
        Require(Bits(thickness, section.thickness_m), "Execution section thickness differs within the original cell");
    if (role == source::SourceShellRole::OriginalRigidPart) {
        Require(source_material.source.keyword == "*MAT_RIGID" && supplied.law == fe::ShellSectionLaw::LayeredLaw1Nip3 &&
            formulation == fe::ShellSectionFormulation::LayeredNip3 && section.through_thickness_points == 3,
            "Rigid skin requires the original reference-only MAT20 declaration");
        material.law = fe::ShellSectionLaw::RigidSkin;
        section.formulation = fe::ShellSectionFormulation::Nonconstitutive;
        section.through_thickness_points = 0;
    } else {
        Require(role == source::SourceShellRole::ConstitutiveShell && source_material.source.keyword != "*MAT_RIGID" &&
            supplied.law != fe::ShellSectionLaw::RigidSkin, "Constitutive shell cannot replace an original rigid role");
    }
    const auto* old_material = Find(out.materials, material.material_id, [](const auto& m) { return m.material_id; });
    const auto* old_section = Find(out.sections, section.section_id, [](const auto& s) { return s.section_id; });
    Require(!old_material || Same(*old_material, material), "Shared source MID has conflicting execution declarations");
    Require(!old_section || Same(*old_section, section), "Shared source SID has conflicting execution declarations");
    if (!old_material) out.materials.push_back(material);
    if (!old_section) out.sections.push_back(section);
}
} // namespace crash::cases::vehicle_startup::shell_execution::detail
