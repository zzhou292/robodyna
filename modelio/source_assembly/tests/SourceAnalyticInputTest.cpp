#include "AnalyticTestSupport.h"
#include "lib_src/elements/ShellBatchBinding.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
#include <cmath>

namespace crash::modelio::assembly::test {
namespace {
using analytic::Identity;
using analytic::Path;
SourceAssembly LoadAnalytic(bool mixed) { return analytic::Load(mixed); }
std::string AlterAnalytic(const std::function<void(output::Document&)>& edit) {
    const auto bytes = output::ReadBounded(Path(false), Identity(false).bytes);
    output::Document d; d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(), bytes.size());
    output::Require(!d.HasParseError(), "Invalid analytic source fixture"); edit(d);
    rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(d.Accept(writer), "Invalid analytic corruption fixture");
    return {buffer.GetString(), buffer.GetSize()};
}
} // namespace

TEST(SourceAnalyticInputs, ActualOriginalPartAndMixedMaterialsPrepareEveryNativeParent) {
    namespace fe = tl::fea;
    for (bool mixed : {false, true}) {
        SCOPED_TRACE(mixed);
        const auto source = LoadAnalytic(mixed); const auto& d = source.data();
        EXPECT_EQ(d.schema, Law44InventorySchema); EXPECT_EQ(d.nodes.size(), mixed ? 538u : 421u);
        EXPECT_EQ(d.parents.size(), mixed ? 482u : 388u); EXPECT_EQ(d.curves.size(), mixed ? 1u : 0u);
        EXPECT_EQ(d.qeph_count, mixed ? 446u : 358u); EXPECT_EQ(d.t3_count, mixed ? 36u : 30u);
        EXPECT_EQ(d.parts[0].id, 2000064u); EXPECT_EQ(d.boundary.spotweld_ids.size(), 33u);
        const SourceAssemblyShellInput geometry(source);
        const SourceAssemblyMaterialInput materials(source, MaterialRatePolicy::OpenRadiossDirectImportDefault);
        fe::ShellBatchBinding binding;
        const auto bound = binding.Initialize(geometry.input(), fe::ShellHostBindingLimits{});
        ASSERT_EQ(bound.status, fe::ShellBindingStatus::Success) << bound.message;
        fe::ShellBatchPlasticityBinding catalog; const auto report = catalog.Initialize(binding, materials.input());
        ASSERT_EQ(report.status, fe::ShellPlasticityBindingStatus::Success) << report.message;
        for (const auto& p : d.parents) {
            fe::sections::PointParameters parameters;
            const auto family = p.family == ShellFamily::Qeph ? fe::ShellBindingFamily::Qeph : fe::ShellBindingFamily::T3;
            ASSERT_TRUE(catalog.Parameters(family, p.family_index, &parameters));
            const auto& m = d.materials[p.material_index];
            SameBits(parameters.young_pa, m.young_pa); SameBits(parameters.density_kg_m3, m.density_kg_m3);
            EXPECT_TRUE(parameters.rate.enabled); SameBits(parameters.rate.cutoff_hz, 10000.);
            const auto* mapped = catalog.parent(p.index); ASSERT_NE(mapped, nullptr);
            EXPECT_EQ(mapped->source_parent_id, p.source_id); EXPECT_EQ(mapped->source_part_id, p.part_id);
            if (p.part_id == 2000064) {
                EXPECT_EQ(m.hardening, MaterialHardening::LinearLaw44); EXPECT_EQ(p.curve_index, NoCurveIndex);
                EXPECT_EQ(parameters.hardening, tl::material::ShellPlasticityHardeningKind::LinearLaw44);
                EXPECT_EQ(parameters.curve.count, 0u); EXPECT_EQ(parameters.curve.plastic_strain, nullptr);
                EXPECT_EQ(parameters.continuation, tl::material::ShellPlasticityCurveContinuation::StrictDomain);
                SameBits(parameters.linear.initial_yield_pa, 20e6); SameBits(parameters.linear.tangent_modulus_pa, 10e6);
            } else {
                EXPECT_EQ(m.hardening, MaterialHardening::TabulatedLaw44);
                EXPECT_EQ(parameters.hardening, tl::material::ShellPlasticityHardeningKind::Tabulated);
                EXPECT_GT(parameters.curve.count, 1u); EXPECT_NE(p.curve_index, NoCurveIndex);
                EXPECT_EQ(parameters.continuation, tl::material::ShellPlasticityCurveContinuation::NativeLastSegment);
            }
        }
    }
}

TEST(SourceAnalyticInputs, ExplicitModeCurveAndLateSourceChangesRejectWithoutReplacingSource) {
    const auto source = LoadAnalytic(false); const auto* nodes = source.data().nodes.data();
    for (unsigned fault = 0; fault < 9; ++fault) {
        const auto bytes = AlterAnalytic([&](auto& d) {
            auto& m = d["declarations"]["materials"][0]; auto& parents = d["parent_bindings"];
            if (fault == 0) d["schema"].SetString(InventorySchema, d.GetAllocator());
            if (fault == 1) m["hardening_model"].SetString("law44_tabulated", d.GetAllocator());
            if (fault == 2) m["hardening_model"].SetString("guess", d.GetAllocator());
            if (fault == 3) m["supplied_etan_pa"].SetDouble(1e9);
            if (fault == 4) m["supplied_sigy_pa"].SetDouble(std::nextafter(20e6, INFINITY));
            if (fault == 5) parents[parents.Size()-1]["curve_index"].SetUint64(0);
            if (fault == 6) parents[parents.Size()-1]["source_curve_id"].SetUint64(2100270);
            if (fault == 7) d["law44_policy"]["rate_filter_hz"].SetDouble(1);
            if (fault == 8) m["cards"][0]["blank_field_mask"].SetUint(128);
        });
        EXPECT_THROW(SourceAssembly::ReadBytes(bytes, ExplicitTestIdentity(bytes)), std::runtime_error) << fault;
        EXPECT_EQ(source.data().nodes.data(), nodes); EXPECT_EQ(source.data().identity.sha256, Identity(false).sha256);
    }
    EXPECT_EQ(LoadAnalytic(false).data().parents.size(), 388u);
}

TEST(SourceAnalyticInputs, CopiedMixedAdapterOwnsTablesAndAnalyticScalarsTogether) {
    auto first = std::make_unique<SourceAssemblyMaterialInput>(LoadAnalytic(true), MaterialRatePolicy::OpenRadiossDirectImportDefault);
    auto copied = *first; first.reset(); auto moved = std::move(copied);
    const auto input = moved.input(); ASSERT_EQ(input.material_count, 2u); ASSERT_EQ(input.curve_count, 1u);
    EXPECT_EQ(input.materials[0].curve_id, 0u); SameBits(input.materials[0].linear.initial_yield_pa, 20e6);
    EXPECT_GT(input.curves[0].curve.count, 1u); EXPECT_GT(input.curves[0].curve.yield_stress_pa[0], 0);
    const auto legacy = Load(); const auto& old = legacy.data();
    EXPECT_EQ(old.schema, InventorySchema); EXPECT_EQ(old.curves.size(), 2u);
}
} // namespace crash::modelio::assembly::test
