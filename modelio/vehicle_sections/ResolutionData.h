#pragma once
#include "Internal.h"
#include "MidlayerDeclarations.h"
#include "modelio/rigid_part/RigidPartSource.h"
#include "modelio/source_assembly/NativeMaterialInput.h"

namespace crash::modelio::vehicle {
struct VehicleSectionResolution::Data {
    explicit Data(const VehicleSourcePlan& value) : source(value) {}
    VehicleSourcePlan source;
    ResolutionKey key;
    std::shared_ptr<const Data> base;
    std::optional<resolution::MidlayerDeclaration> midlayer;
    std::optional<rigid_part::RigidPartSource> rigid;
    std::size_t midlayer_part = SIZE_MAX;
    std::vector<NativeParentMapping> mapping;
    NativeFormulationCounts native_counts;
    resolution::Declarations declarations;
    std::vector<SectionParentResolution> parents;
    std::vector<tl::fea::ShellPlasticityMaterialInput> native_materials;
    std::vector<tl::fea::ShellFailureParentInput> native_parents;
    std::size_t budget = 0;
    const std::vector<SectionParentResolution>& Parents() const noexcept {
        return base ? base->Parents() : parents;
    }
    const assembly::Material* Material(std::size_t p) const noexcept {
        if (p >= declarations.parts.size()) return nullptr;
        if (rigid && rigid->body(p)) return &rigid->body(p)->declaration.material;
        if (base) return p == midlayer_part ? &midlayer->material : base->Material(p);
        const auto& part = declarations.parts[p];
        if (part.status == SectionDisposition::Existing) return source.material(p);
        if (part.status == SectionDisposition::GlassTab1) return &declarations.glass[part.material_index].material;
        return part.status == SectionDisposition::ConstantFailure ?
            &declarations.failure.materials[part.material_index] : nullptr;
    }
    const assembly::Section* Section(std::size_t p) const noexcept {
        if (p >= declarations.parts.size()) return nullptr;
        if (rigid && rigid->body(p)) return &rigid->body(p)->declaration.section;
        if (base) return p == midlayer_part ? &midlayer->section : base->Section(p);
        const auto& part = declarations.parts[p];
        if (part.status == SectionDisposition::Existing) return source.section(p);
        if (part.status == SectionDisposition::GlassTab1) return &declarations.glass[part.section_index].section;
        return part.status == SectionDisposition::ConstantFailure ?
            &declarations.failure.sections[part.section_index] : nullptr;
    }
    const rigid_part::RigidPartSource* Rigid() const noexcept {
        return rigid ? &*rigid : (base ? base->Rigid() : nullptr);
    }
    const tl::fea::ShellPlasticityMaterialInput* NativeMaterial(std::size_t p) const noexcept {
        if (!Material(p)) return nullptr;
        if (!base) return &native_materials[p];
        if (p == midlayer_part) return &native_materials[0];
        if (rigid && rigid->body(p)) return &native_materials[rigid->data().part_to_body[p]];
        return base->NativeMaterial(p);
    }
    const std::vector<assembly::Curve>& FailureCurves() const noexcept {
        return base ? base->FailureCurves() : declarations.failure.curves;
    }
    tl::fea::ShellSectionFormulation Formulation(std::size_t p) const noexcept {
        if (p == midlayer_part && midlayer) return tl::fea::ShellSectionFormulation::OneThicknessPoint;
        return base ? base->Formulation(p) : tl::fea::ShellSectionFormulation::LayeredNip3;
    }
    void PrepareNativeValues() {
        native_materials.resize(declarations.parts.size());
        for (std::size_t p = 0; p < declarations.parts.size(); ++p) {
            const auto* material = Material(p);
            if (!material) continue;
            if (declarations.parts[p].status == SectionDisposition::GlassTab1) {
                native_materials[p] = resolution::NativeGlassMaterial(*material);
                continue;
            }
            auto native = assembly::detail::NativeMaterial(*material);
            if (declarations.parts[p].status == SectionDisposition::ConstantFailure && material->curve_id) {
                native.continuation = tl::material::ShellPlasticityCurveContinuation::NativeLastSegment;
            }
            native_materials[p] = native;
        }
        native_parents.resize(parents.size());
        for (std::size_t e = 0; e < parents.size(); ++e) {
            const auto& parent = parents[e];
            const auto& part = declarations.parts[parent.part_index];
            if (part.status == SectionDisposition::Unresolved) continue;
            const auto& original = source.parts()[parent.part_index];
            auto& native = native_parents[e];
            native.source = {parent.topology == SourceShellTopology::Q4 ?
                                 tl::fea::ShellBindingFamily::Qeph : tl::fea::ShellBindingFamily::T3,
                             parent.topology_index, parent.source_parent_id, original.part_id,
                             original.material_id, original.section_id};
            if (part.status == SectionDisposition::ConstantFailure) {
                native.policy = tl::fea::ShellFailurePolicy::ConstantAllPoints;
                native.constant.failure_strain = part.failure_strain;
            } else if (part.status == SectionDisposition::GlassTab1) {
                native.policy = tl::fea::ShellFailurePolicy::Tab1AnyPoint;
                native.tab1.table = {{-.3,0,.3},part.failure_strain};
                native.tab1.parent_policy = tl::fea::sections::ShellTab1ParentPolicy::AnyPoint;
            }
        }
    }
};
} // namespace crash::modelio::vehicle
