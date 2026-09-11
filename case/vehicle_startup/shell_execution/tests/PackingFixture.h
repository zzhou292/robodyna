#pragma once
#include "../Packing.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>

namespace crash::cases::vehicle_startup::shell_execution::test {
namespace fe = tl::fea;
namespace source = modelio::vehicle;
struct Part {
    source::PartDisposition id;
    modelio::assembly::Material material;
    modelio::assembly::Section section;
    fe::ShellPlasticityMaterialInput native;
    source::SourceShellRole role = source::SourceShellRole::ConstitutiveShell;
    fe::ShellSectionFormulation formulation = fe::ShellSectionFormulation::LayeredNip3;
    explicit Part(std::uint64_t value = 100) {
        id.part_id = value;
        id.material_id = material.id = native.material_id = value;
        id.section_id = section.id = value;
        material.young_pa = native.young_pa = 200e9;
        material.poisson_ratio = native.poisson_ratio = .3;
        material.density_kg_m3 = native.density_kg_m3 = 7800;
        material.source.keyword = "*MAT_PIECEWISE_LINEAR_PLASTICITY";
        native.hardening = tl::material::ShellPlasticityHardeningKind::LinearLaw44;
        native.linear = {2e8, 1e9};
        native.rate = {true, 8000, 8, 10000};
        section.through_thickness_points = 3;
        section.thickness_m = {.001,.001,.001,.001};
    }
    void Add(detail::Packing& out) const {
        detail::AddPart(out,id,material,section,native,role,formulation);
    }
    void Rigid() {
        material.source.keyword = "*MAT_RIGID";
        native.law = fe::ShellSectionLaw::LayeredLaw1Nip3;
        native.hardening = tl::material::ShellPlasticityHardeningKind::Tabulated;
        native.linear = {};
        native.rate = {};
        role = source::SourceShellRole::OriginalRigidPart;
    }
};
inline modelio::assembly::Curve Curve(std::uint64_t id = 10) {
    modelio::assembly::Curve value;
    value.id = id;
    value.plastic_strain = {0,.1,.3};
    value.stress_pa = {2e8,3e8,3.5e8};
    return value;
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
