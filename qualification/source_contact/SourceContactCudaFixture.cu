#include "SourceContactCudaFixture.h"
#include "collision/Q4IntegralMeasure.h"
#include "collision/Q4RectangularIntegration.h"
#include "collision/T3ContactIntegration.h"

namespace crash::qualification::source_contact::cuda_test {
namespace {
// Keep each large scalar integrator out of the worker loop's compiler expansion.
// These wrappers retain the same arithmetic, certificates and publication order.
__device__ __noinline__ bool EvaluateQuad(
    const ParentInput& parent, double wall_x, sc::VectorView position, sc::VectorView velocity,
    sc::LumpedTranslationMassView mass, sc::Q4RectangularScratch scratch,
    sc::Q4RectangularResult& output, sc::Q4IntegrationReport& report) {
    sc::Q4IntegrationLimits limits;
    limits.force_error = force::ForceBudget;
    limits.energy_error = force::EnergyBudget;
    const sc::Q4PrescribedNormalIntegrationInput request{
        {position, velocity, &parent.quad, 1}, mass, 0, parent.attempt,
        wall_x, parent.area.value, force::Stiffness, force::Cap};
    report = sc::IntegrateQ4NormalContactRectangular(request, limits, scratch, &output);
    return report.status == sc::Q4IntegrationStatus::Ok &&
        sc::ExpandQ4IntegralMeasure(parent.area.value, {parent.area.lower, parent.area.upper},
                                   &output.integration) &&
        sc::WithinQ4IntegralBudgets(output.integration, limits);
}
__device__ __noinline__ bool EvaluateTriangle(
    const ParentInput& parent, double wall_x, sc::VectorView position, sc::VectorView velocity,
    sc::LumpedTranslationMassView mass, sc::T3IntegrationResult& output, sc::T3IntegrationReport& report) {
    const sc::T3NormalIntegrationInput request{
        &parent.triangle_reference, {position, velocity, mass.inverse_mass, &parent.triangle, 1},
        mass, 0, parent.attempt, wall_x, force::Stiffness, force::Cap};
    report = sc::IntegrateT3NormalContact(request, {force::ForceBudget, force::EnergyBudget}, &output);
    return report.status == sc::T3IntegrationStatus::Ok;
}
__global__ void EvaluateParents(Storage* storage) {
    const unsigned worker = blockIdx.x;
    if (worker >= Workers || threadIdx.x != 0) return;
    auto& workspace = storage->scratch[worker];
    const sc::Q4RectangularScratch scratch{
        workspace.cells, workspace.heap, sc::MaxQ4IntegrationLeaves, sc::MaxQ4IntegrationLeaves};
    const auto& input = storage->input;
    const sc::LumpedTranslationMassView mass{
        input.inverse_mass, input.fixed, NodeCount, 17, sc::TranslationMassModel::kIsotropicLumped};
    #pragma unroll 1
    for (unsigned p = worker; p < ParentCount; p += Workers) {
        const auto& parent = input.parents[p];
        const sc::VectorView position{parent.position, NodeCount, 3, 1};
        const sc::VectorView velocity{parent.velocity, NodeCount, 3, 1};
        ParentOutput next;
        bool accepted = false;
        if (parent.arity == 4) {
            accepted = EvaluateQuad(parent, input.wall_x, position, velocity, mass, scratch,
                                    next.quad, storage->result.quad_reports[p]);
        } else if (parent.arity == 3) {
            accepted = EvaluateTriangle(parent, input.wall_x, position, velocity, mass,
                                        next.triangle, storage->result.triangle_reports[p]);
        }
        // Preserve every previous parent field on rejection, including the
        // inactive family. The acceptance flags do not publish a whole batch.
        storage->result.accepted[p] = accepted;
        if (accepted) storage->result.parents[p] = next;
    }
}
} // namespace

Device::Device() {
    initialization_status_ = cudaMalloc(&storage_, sizeof(Storage));
    if (initialization_status_ != cudaSuccess) return;
    initialization_status_ = cudaMemset(storage_, 0, sizeof(Storage));
    if (initialization_status_ != cudaSuccess) return;
    initialization_status_ = cudaEventCreate(&begin_);
    if (initialization_status_ != cudaSuccess) return;
    initialization_status_ = cudaEventCreate(&end_);
}
Device::~Device() {
    if (end_) cudaEventDestroy(end_);
    if (begin_) cudaEventDestroy(begin_);
    if (storage_) cudaFree(storage_);
}
cudaError_t Device::Evaluate(const Input& input, Result& output, float& elapsed_ms) {
    if (initialization_status_ != cudaSuccess) return initialization_status_;
    auto status = cudaMemcpy(&storage_->input, &input, sizeof(Input), cudaMemcpyHostToDevice);
    if (status != cudaSuccess) return status;
    status = cudaEventRecord(begin_);
    if (status != cudaSuccess) return status;
    EvaluateParents<<<Workers, 1>>>(storage_);
    status = cudaGetLastError();
    if (status != cudaSuccess) return status;
    status = cudaEventRecord(end_);
    if (status != cudaSuccess) return status;
    status = cudaEventSynchronize(end_);
    if (status != cudaSuccess) return status;
    status = cudaEventElapsedTime(&elapsed_ms, begin_, end_);
    if (status != cudaSuccess) return status;
    return cudaMemcpy(&output, &storage_->result, sizeof(Result), cudaMemcpyDeviceToHost);
}
} // namespace crash::qualification::source_contact::cuda_test
