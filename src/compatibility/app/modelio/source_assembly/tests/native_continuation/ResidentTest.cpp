#include "Fixture.h"
#include "lib_src/elements/ShellBatchPlasticityStorage.h"
#include "lib_src/elements/ShellMixedSectionArenaLayout.h"
#include "lib_src/elements/failure/ShellFailureArenaLayout.h"
#include <cuda_runtime_api.h>
namespace crash::modelio::assembly::continuation_test {
TEST(NativeMaterialContinuationResident, ImportedOrdinaryAndFailureTablesReachTheirActualDeviceParameters) {
    namespace storage = fe::shell_batch_plasticity_detail;
    Fixture fixture;
    for (auto family : {fe::ShellBindingFamily::Qeph, fe::ShellBindingFamily::T3}) {
        storage::HostStorage resident;
        const auto report = resident.InitializeFailureCollection(fixture.failure, fixture.geometry, family, 1,
            1u << 20, 16u << 20, {}, false);
        ASSERT_EQ(report.status, storage::SetupStatus::Success) << report.message;
        ASSERT_NE(resident.mixed_device(), nullptr);
        storage::MixedDeviceStorage header;
        ASSERT_EQ(cudaMemcpy(&header, resident.mixed_device(), sizeof(header), cudaMemcpyDeviceToHost), cudaSuccess);
        fe::sections::PointParameters actual;
        ASSERT_EQ(cudaMemcpy(&actual, header.plastic.parameters, sizeof(actual), cudaMemcpyDeviceToHost), cudaSuccess);
        const auto expected = fixture.Parameters(family);
        EXPECT_EQ(actual.continuation, Policy::NativeLastSegment);
        EXPECT_EQ(actual.rate.policy, expected.rate.policy);
        EXPECT_EQ(output::Bits(actual.rate.cowper_symonds_c_per_s), output::Bits(expected.rate.cowper_symonds_c_per_s));
        EXPECT_EQ(output::Bits(actual.rate.cowper_symonds_p), output::Bits(expected.rate.cowper_symonds_p));
        ASSERT_EQ(actual.curve.count, expected.curve.count);
        std::array<double, 3> strain, stress;
        ASSERT_EQ(cudaMemcpy(strain.data(), actual.curve.plastic_strain, sizeof(strain), cudaMemcpyDeviceToHost), cudaSuccess);
        ASSERT_EQ(cudaMemcpy(stress.data(), actual.curve.yield_stress_pa, sizeof(stress), cudaMemcpyDeviceToHost), cudaSuccess);
        for (unsigned i = 0; i < 3; ++i) {
            EXPECT_EQ(output::Bits(strain[i]), output::Bits(expected.curve.plastic_strain[i]));
            EXPECT_EQ(output::Bits(stress[i]), output::Bits(expected.curve.yield_stress_pa[i]));
        }
        storage::FailureDeviceStorage failure;
        ASSERT_EQ(cudaMemcpy(&failure, resident.failure_device(), sizeof(failure), cudaMemcpyDeviceToHost), cudaSuccess);
        fe::ShellFailurePolicy policy;
        ASSERT_EQ(cudaMemcpy(&policy, failure.policy, sizeof(policy), cudaMemcpyDeviceToHost), cudaSuccess);
        EXPECT_EQ(policy, fixture.failure.parent(family, 0)->policy);
    }
}
}
