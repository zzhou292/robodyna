#include "GlassSourceSupport.h"
#include "modelio/source_assembly/NativeMaterialInput.h"

namespace crash::modelio::vehicle::test {
TEST(VehicleGlassResolution, CompleteOriginalRowsPreserveV1AndResolveNineGlassParts) {
    const auto& resolution = GlassResolution();
    const auto& prior = Resolution();
    const auto& counts = resolution.counts();
    ASSERT_TRUE(resolution.includes_glass());
    EXPECT_FALSE(prior.includes_glass());
    EXPECT_EQ(counts.parts,867);
    EXPECT_EQ(counts.shells,349645);
    EXPECT_EQ(counts.existing_shells,278301);
    EXPECT_EQ(counts.failure_shells,47781);
    EXPECT_EQ(counts.glass_parts,9);
    EXPECT_EQ(counts.glass_shells,14210);
    EXPECT_EQ(counts.placed_glass_shells,8502);
    EXPECT_EQ(counts.unresolved_shells,9353);
    EXPECT_EQ(&resolution.source().canonical().data(),&prior.source().canonical().data());
    ASSERT_EQ(resolution.parents().size(),prior.parents().size());
    const std::uint64_t ids[]{2000011,2000023,2000062,2000325,2000347,2000349,2000407,2000452,2000523};
    const std::size_t shells[]{619,4251,1740,234,606,241,1367,901,4251};
    std::size_t glass_parts = 0,glass_parents = 0;
    for (std::size_t p = 0; p < resolution.parts().size(); ++p) {
        const auto& part = resolution.parts()[p];
        const auto& original = Plan().parts()[p];
        if (part.status != SectionDisposition::GlassTab1) {
            ASSERT_EQ(part.status,prior.parts()[p].status);
            if (const auto* material = prior.material(p)) {
                const auto* current = resolution.material(p);
                ASSERT_NE(current,nullptr);
                if (part.status == SectionDisposition::Existing) EXPECT_EQ(current,material);
                EXPECT_EQ(current->source.raw_text,material->source.raw_text);
                EXPECT_EQ(current->source.sha256,material->source.sha256);
                EXPECT_EQ(output::Bits(current->young_pa),output::Bits(material->young_pa));
                EXPECT_EQ(output::Bits(current->density_kg_m3),output::Bits(material->density_kg_m3));
                EXPECT_EQ(output::Bits(current->poisson_ratio),output::Bits(material->poisson_ratio));
                const auto& native = *resolution.native_material(p);
                const auto& old_native = *prior.native_material(p);
                EXPECT_EQ(native.material_id,old_native.material_id);
                EXPECT_EQ(native.curve_id,old_native.curve_id);
                EXPECT_EQ(native.law,old_native.law);
                EXPECT_EQ(native.hardening,old_native.hardening);
                EXPECT_EQ(native.continuation,old_native.continuation);
                EXPECT_EQ(native.rate.enabled,old_native.rate.enabled);
                EXPECT_EQ(native.rate.policy,old_native.rate.policy);
                const double a[]{native.young_pa,native.poisson_ratio,native.density_kg_m3,
                    native.rate.cowper_symonds_c_per_s,native.rate.cowper_symonds_p,native.rate.cutoff_hz,
                    native.linear.initial_yield_pa,native.linear.tangent_modulus_pa};
                const double b[]{old_native.young_pa,old_native.poisson_ratio,old_native.density_kg_m3,
                    old_native.rate.cowper_symonds_c_per_s,old_native.rate.cowper_symonds_p,old_native.rate.cutoff_hz,
                    old_native.linear.initial_yield_pa,old_native.linear.tangent_modulus_pa};
                for (unsigned i = 0; i < 8; ++i) EXPECT_EQ(output::Bits(a[i]),output::Bits(b[i]));
            }
            continue;
        }
        ASSERT_LT(glass_parts,9);
        EXPECT_EQ(original.part_id,ids[glass_parts]);
        EXPECT_EQ(original.shell_count,shells[glass_parts++]);
        EXPECT_EQ(prior.parts()[p].status,SectionDisposition::Unresolved);
        ASSERT_NE(resolution.material(p),nullptr);
        ASSERT_NE(resolution.section(p),nullptr);
        ASSERT_NE(resolution.native_material(p),nullptr);
        const auto& native = *resolution.native_material(p);
        EXPECT_EQ(native.material_id,original.material_id);
        EXPECT_EQ(native.curve_id,0);
        EXPECT_EQ(native.rate.policy,tl::material::ShellPlasticityRatePolicy::FilteredZeroC);
        EXPECT_EQ(output::Bits(native.rate.cowper_symonds_c_per_s),output::Bits(0.));
        EXPECT_EQ(output::Bits(native.rate.cowper_symonds_p),output::Bits(1.));
        EXPECT_EQ(output::Bits(native.rate.cutoff_hz),output::Bits(10000.));
        EXPECT_EQ(output::Bits(part.failure_strain),output::Bits(.015));
        EXPECT_EQ(resolution.material(p)->source.sha256,original.unresolved_sources[2].sha256);
        EXPECT_EQ(resolution.section(p)->source.sha256,original.unresolved_sources[1].sha256);
        const auto placement = original.part_id == 2000023 ? tl::fea::ShellReferencePlacement::BottomReferencePlane :
            (original.part_id == 2000523 ? tl::fea::ShellReferencePlacement::TopReferencePlane :
                                         tl::fea::ShellReferencePlacement::Centered);
        EXPECT_EQ(part.placement,placement);
    }
    for (std::size_t e = 0; e < resolution.parents().size(); ++e) {
        const auto& parent = resolution.parents()[e];
        const auto& old = prior.parents()[e];
        ASSERT_EQ(parent.source_parent_id,old.source_parent_id);
        ASSERT_EQ(parent.canonical_parent,old.canonical_parent);
        ASSERT_EQ(parent.part_index,old.part_index);
        ASSERT_EQ(parent.topology,old.topology);
        ASSERT_EQ(parent.topology_index,old.topology_index);
        const auto* native = resolution.native_parent(e);
        if (resolution.parts()[parent.part_index].status == SectionDisposition::Unresolved) {
            ASSERT_EQ(native,nullptr);
            continue;
        }
        ASSERT_NE(native,nullptr);
        EXPECT_EQ(native->source.source_parent_id,parent.source_parent_id);
        EXPECT_EQ(native->source.family_index,parent.topology_index);
        if (resolution.parts()[parent.part_index].status == SectionDisposition::GlassTab1) {
            ++glass_parents;
            EXPECT_EQ(native->policy,tl::fea::ShellFailurePolicy::Tab1AnyPoint);
            EXPECT_EQ(native->tab1.parent_policy,tl::fea::sections::ShellTab1ParentPolicy::AnyPoint);
            const double x[]{-.3,0,.3};
            for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(output::Bits(native->tab1.table.triaxiality[i]),output::Bits(x[i]));
            EXPECT_EQ(output::Bits(native->tab1.table.failure_strain),output::Bits(.015));
        } else {
            ASSERT_NE(prior.native_parent(e),nullptr);
            EXPECT_EQ(native->policy,prior.native_parent(e)->policy);
            EXPECT_EQ(output::Bits(native->constant.failure_strain),output::Bits(prior.native_parent(e)->constant.failure_strain));
        }
    }
    EXPECT_EQ(glass_parts,9);
    EXPECT_EQ(glass_parents,14210);
    ASSERT_EQ(resolution.failure_curves().size(),prior.failure_curves().size());
    for (std::size_t i = 0; i < prior.failure_curves().size(); ++i) {
        const auto& current = resolution.failure_curves()[i];
        const auto& old = prior.failure_curves()[i];
        EXPECT_EQ(current.source.raw_text,old.source.raw_text);
        EXPECT_EQ(current.id,old.id);
        ASSERT_EQ(current.plastic_strain.size(),old.plastic_strain.size());
        for (std::size_t p = 0; p < old.plastic_strain.size(); ++p) {
            EXPECT_EQ(output::Bits(current.plastic_strain[p]),output::Bits(old.plastic_strain[p]));
            EXPECT_EQ(output::Bits(current.stress_pa[p]),output::Bits(old.stress_pa[p]));
        }
    }
    RecordProperty("resolution_sha256",resolution.identity().sha256);
    RecordProperty("startup_budget_bytes",std::to_string(resolution.startup_budget_bytes()));
}
TEST(VehicleGlassResolution, LateSourcePolicyCountAndBudgetFailuresPreserveExistingHandle) {
    const auto& prior = GlassResolution();
    const auto* retained = prior.parents().data();
    for (unsigned fault = 0; fault < 9; ++fault) {
        SCOPED_TRACE(fault);
        const auto bytes = AlterGlass([&](output::Document& doc) {
            auto& glass = doc["glass_declarations"];
            auto& last = glass["parts"][glass["parts"].Size()-1];
            switch (fault) {
            case 0: doc["source"]["vehicle_plan_sha256"].SetString(std::string(64,'0').c_str(),doc.GetAllocator()); break;
            case 1: doc["schema"].SetString(ResolutionSchema,doc.GetAllocator()); break;
            case 2: last["material"]["source_numint"].SetDouble(2); break;
            case 3: last["material"]["cards"][1]["values"][3].SetDouble(-0.0); break;
            case 4: last["section"]["placement"].SetString("centered",doc.GetAllocator()); break;
            case 5: glass["policy"]["resolved_IFAIL_SH"].SetUint(2); break;
            case 6: doc["counts"]["glass_shells"].SetUint(14209); break;
            case 7:
                for (auto& row : doc["parts"].GetArray()) {
                    if (row["part_id"].GetUint64() == 2000523) row["status"].SetString("unresolved",doc.GetAllocator());
                }
                break;
            case 8: last["material"]["failure_strain"].SetDouble(.03); break;
            }
        });
        EXPECT_THROW(VehicleSectionResolution::ReadBytes(Plan(),bytes,Identity(bytes)),std::runtime_error);
        EXPECT_EQ(prior.parents().data(),retained);
    }
    ResolutionLimits limits;
    limits.declaration_bytes = GlassBytes().size();
    limits.parents = prior.parents().size();
    limits.parts = prior.parts().size();
    limits.host_bytes = prior.startup_budget_bytes();
    auto short_limits = limits;
    --short_limits.host_bytes;
    EXPECT_THROW(VehicleSectionResolution::ReadBytes(Plan(),"",Identity(GlassBytes()),short_limits),std::runtime_error);
    short_limits = limits;
    --short_limits.parents;
    EXPECT_THROW(VehicleSectionResolution::ReadBytes(Plan(),"",Identity(GlassBytes()),short_limits),std::runtime_error);
    short_limits = limits;
    --short_limits.declaration_bytes;
    EXPECT_THROW(VehicleSectionResolution::ReadBytes(Plan(),"",Identity(GlassBytes()),short_limits),std::runtime_error);
    const auto retry = VehicleSectionResolution::ReadBytes(Plan(),GlassBytes(),Identity(GlassBytes()),limits);
    EXPECT_EQ(retry.counts().glass_shells,14210);
    EXPECT_EQ(prior.parents().data(),retained);
}
} // namespace crash::modelio::vehicle::test
