#include "../Capture.h"
#include "../Export.h"
#include "case/vehicle_dynamics/Reports.h"
#include "lib_utest/qualification/qt_mapped/OwnerFixture.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>
#include <cstring>
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::test {
namespace fe = tl::fea;
using Rig = qt_mapped_test::Rig<qt_mapped_test::Quad>;
TEST(QephRejectedBridgeCuda, RealMappedFailureSurvivesTypedThrowDiscardAndCompanionReplay) {
    output::full_shell::test::Directory temporary;
    const auto* requested = std::getenv("ROBO_QEPH_APP_CAPTURE_OUTPUT");
    const auto root = requested ? std::filesystem::path(requested) : temporary.path / "coupons";
    ASSERT_EQ(std::filesystem::symlink_status(root).type(), std::filesystem::file_type::not_found);
    ASSERT_TRUE(std::filesystem::create_directory(root));
    // Both are existing qualified owner fixtures. The latter adds explicit
    // execution roles so the app also checks its requested source IDs/law.
    for (bool execution_scope : {false, true}) {
        SCOPED_TRACE(execution_scope);
        Rig rig;
        ASSERT_TRUE(rig.Initialize(execution_scope));
        fe::NodalTrialToken token;
        fe::NodalAssemblyView assembly;
        fe::NodalPreparedView prepared;
        ASSERT_TRUE(rig.Prepare(token, assembly, prepared));
        const auto stamp = rig.owner.accepted();
        const auto accepted_values = qt_mapped_test::Values(rig.Accepted());
        const auto accepted_materials = rig.AcceptedState();
        const auto& reference = qt_mapped_test::Quad::Reference(rig.fixture, rig.config.element_count - 1);
        const auto node = rig.fixture.mechanics.domain.Find(reference.input.node_ids[3]);
        // Intentional test-only malformed prepared coordinate, matching the TL
        // qualification seam. The real QEPH evaluator supplies the rejection.
        const std::uint64_t nan_bits = UINT64_C(0x7ff800000000cafe);
        double invalid;
        std::memcpy(&invalid, &nan_bits, sizeof(invalid));
        ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(prepared.kinematics.position_xyz) + 3 * node,
            &invalid, sizeof(invalid), cudaMemcpyHostToDevice, prepared.stream), cudaSuccess);
        ASSERT_EQ(cudaStreamSynchronize(prepared.stream), cudaSuccess);
        native::BatchDiagnostics diagnostics;
        const auto failed = rig.batch.EvaluateCandidate(rig.owner, token, prepared, &diagnostics);
        ASSERT_EQ(failed.status, native::BatchStatus::ElementFailure);
        CaptureState captured;
        CaptureRejected(captured, rig.batch, rig.owner, token, prepared, failed, rig.fixture.Physical());
        ASSERT_TRUE(captured.seen);
        ASSERT_TRUE(captured.complete) << captured.message.data();
        ASSERT_EQ(captured.status, native::RejectedCaptureStatus::Captured);
        EXPECT_EQ(captured.requested.source.available, execution_scope);
        EXPECT_EQ(captured.requested.law_available, execution_scope);
        EXPECT_TRUE(captured.input.source.available);
        EXPECT_EQ(captured.input.source.parent,
            rig.fixture.Physical().shells()->qeph_source_id(failed.element));
        if (execution_scope) {
            EXPECT_EQ(captured.requested.source.parent, captured.input.source.parent);
            EXPECT_EQ(captured.requested.law, captured.input.law);
        }
        const auto frozen = Encode(captured.input);
        bool caught = false;
        try {
            vehicle_dynamics::detail::Require(failed, "QEPH candidate");
            FAIL() << "Expected the same typed error as the production stage wrapper";
        } catch (const NativeStageError& error) {
            caught = true;
            const auto& detail = std::get<QephFailure>(error.failure());
            EXPECT_EQ(detail.status, failed.status);
            EXPECT_EQ(detail.element_index, failed.element);
            EXPECT_EQ(detail.element_status, failed.element_status);
            // Matches the dynamics failure path: owner and participant discard
            // occur before any diagnostic companion is exported.
            rig.owner.Discard();
            rig.batch.DiscardTrial();
            EXPECT_NE(std::string(error.what()).find("participant=qeph"), std::string::npos);
        }
        ASSERT_TRUE(caught);
        EXPECT_TRUE(fe::trial_identity::SameStamp(stamp, rig.owner.accepted()));
        EXPECT_EQ(qt_mapped_test::Values(rig.Accepted()), accepted_values);
        EXPECT_EQ(rig.AcceptedState(), accepted_materials);
        EXPECT_EQ(Encode(captured.input), frozen);
        native::RejectedCandidateInput unavailable;
        EXPECT_EQ(rig.batch.CopyRejectedCandidate(rig.owner, token, prepared, failed, &unavailable).status,
            native::RejectedCaptureStatus::NoRejectedCandidate);
        const auto run = root / (execution_scope ? "execution-scope" : "mapped-catalog");
        ASSERT_TRUE(std::filesystem::create_directory(run));
        const auto exported = ExportForRun(captured, run);
        ASSERT_EQ(exported.status, ExportStatus::Captured) << exported.error.data();
        ASSERT_TRUE(exported.manifest);
        const auto bytes = output::ReadBounded(run / "qeph-rejection/failure.json", 32u << 10);
        output::Document document; document.Parse(bytes.data(), bytes.size());
        ASSERT_FALSE(document.HasParseError());
        EXPECT_TRUE(document["host_reproduces_original_rejection"].GetBool());
        EXPECT_EQ(document["element_index"].GetUint(), failed.element);
        EXPECT_EQ(document["element_status"].GetUint(), static_cast<unsigned>(failed.element_status));
        EXPECT_EQ(captured.original.status, failed.status);
        EXPECT_EQ(captured.original.element_status, failed.element_status);
        RecordProperty(execution_scope ? "execution_scope_manifest" : "mapped_catalog_manifest",
            (run / "qeph-rejection/failure.json").string());
    }
}
}
