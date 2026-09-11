#include "TestSupport.h"

namespace crash::modelio::vehicle::test {
TEST(VehicleSectionResolution, LateSourceAndDispositionFailuresKeepPublishedResolutionAndRetry) {
    const auto& before = Resolution();
    const auto* parents = before.parents().data();
    const auto* last = before.native_parent(before.parents().size() - 1);
    for (unsigned fault = 0; fault < 23; ++fault) {
        SCOPED_TRACE(fault);
        const auto bytes = AlterResolution([&](auto& doc) {
            auto& rows = doc["parts"];
            auto& declarations = doc["constant_failure_declarations"]["declarations"];
            auto& materials = declarations["materials"];
            auto& material = materials[materials.Size() - 1];
            switch (fault) {
            case 0: rows[rows.Size() - 1]["part_id"].SetUint64(9999999); break;
            case 1: rows[rows.Size() - 1]["shells"].SetUint64(9999999); break;
            case 2: rows.PopBack(); break;
            case 3: doc["counts"]["failure_shells"].SetUint64(47782); break;
            case 4: doc["simulation_ready"].SetBool(true); break;
            case 5: doc["native_startup_qualified"].SetBool(true); break;
            case 6: doc["source"]["vehicle_plan_sha256"].SetString(std::string(64, '0').c_str(), doc.GetAllocator()); break;
            case 7: doc["policy"]["IFAIL_SH"].SetUint64(1); break;
            case 8: doc["policy"]["table_continuation"].SetString("StrictDomain", doc.GetAllocator()); break;
            case 9: material["failure_strain"].SetDouble(2); break;
            case 10:
                material["failure_strain"].SetDouble(2);
                material["cards"][0]["values"][6].SetDouble(2);
                break;
            case 11:
                material["young_pa"].SetDouble(material["young_pa"].GetDouble() * .5);
                material["cards"][0]["values"][2].SetDouble(material["cards"][0]["values"][2].GetDouble() * .5);
                break;
            case 12: material["source"]["filename"].SetString("changed.key", doc.GetAllocator()); break;
            case 13: material["source"]["first_line"].SetUint64(1); break;
            case 14: declarations["parts"][0]["title"].SetString("changed title", doc.GetAllocator()); break;
            case 15: declarations["curves"][0]["stress_pa"][0].SetDouble(1); break;
            case 16: doc["constant_failure_declarations"]["selected_part_ids"].PopBack(); break;
            case 17:
                declarations["sections"][declarations["sections"].Size() - 1]["thickness_m"][3].SetDouble(1);
                break;
            case 18: material["source"]["sha256"].SetString(std::string(64, '0').c_str(), doc.GetAllocator()); break;
            case 19: rows[PartIndex(2000003)]["status"].SetString("unresolved", doc.GetAllocator()); break;
            case 20: rows[PartIndex(2000524)]["status"].SetString("constant_failure", doc.GetAllocator()); break;
            case 21: rows[0]["status"].SetString("unresolved", doc.GetAllocator()); break;
            case 22: doc["policy"]["tabulated_etan"].SetString("discard", doc.GetAllocator()); break;
            }
        });
        EXPECT_THROW(VehicleSectionResolution::ReadBytes(Plan(), bytes, Identity(bytes)), std::runtime_error);
        EXPECT_EQ(before.parents().data(), parents);
        EXPECT_EQ(before.native_parent(before.parents().size() - 1), last);
        EXPECT_EQ(before.counts().failure_shells, 47781);
    }
    const auto retry = VehicleSectionResolution::ReadBytes(Plan(), ResolutionBytes(), Identity(ResolutionBytes()));
    ASSERT_EQ(retry.parents().size(), before.parents().size());
    for (std::size_t e = 0; e < retry.parents().size(); ++e) {
        const auto& a = retry.parents()[e];
        const auto& b = before.parents()[e];
        ASSERT_EQ(a.source_parent_id, b.source_parent_id);
        ASSERT_EQ(a.canonical_parent, b.canonical_parent);
        ASSERT_EQ(a.topology_index, b.topology_index);
        const auto* native = retry.native_parent(e);
        ASSERT_EQ(bool(native), bool(before.native_parent(e)));
        if (native) {
            ASSERT_EQ(native->policy, before.native_parent(e)->policy);
            ASSERT_EQ(output::Bits(native->constant.failure_strain), output::Bits(before.native_parent(e)->constant.failure_strain));
        }
    }
}

TEST(VehicleSectionResolution, CountBudgetAndHashPreflightProtectInputAndPublishedPlan) {
    const auto identity = Identity(ResolutionBytes());
    for (unsigned fault = 0; fault < 8; ++fault) {
        ResolutionLimits limits;
        auto id = identity;
        switch (fault) {
        case 0: limits.parents = 349644; break;
        case 1: limits.parts = 866; break;
        case 2: limits.declaration_bytes = identity.bytes - 1; break;
        case 3: limits.host_bytes = Resolution().startup_budget_bytes() - 1; break;
        case 4: limits.parents = 524289; break;
        case 5: limits.tables = 0; break;
        case 6: limits.curve_points = 1; break;
        case 7: id.sha256 = "not a digest"; break;
        }
        try {
            VehicleSectionResolution::Read(Plan(), "/path/not/read/by/preflight", id, limits);
            ADD_FAILURE() << "Invalid resolution preflight accepted";
        } catch (const std::runtime_error& error) {
            const std::string reason = error.what();
            EXPECT_TRUE(reason == "Invalid vehicle section resolution limits" ||
                        reason == "Vehicle resolution count/content identity exceeds limits" ||
                        reason == "Vehicle resolution host cap exceeded") << reason;
        }
    }
    auto corrupt = ResolutionBytes();
    corrupt.back() = ' ';
    EXPECT_THROW(VehicleSectionResolution::ReadBytes(Plan(), corrupt, identity), std::runtime_error);
    EXPECT_THROW(VehicleSectionResolution::ReadBytes(Plan(), ResolutionBytes() + " ", identity), std::runtime_error);
    EXPECT_EQ(Resolution().counts().failure_shells, 47781);
    EXPECT_EQ(Plan().counts().supported_parents, 278301);
}
} // namespace crash::modelio::vehicle::test
