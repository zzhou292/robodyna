#pragma once

#include "SourceContactForceFixture.h"
#include <cuda_runtime.h>

namespace crash::qualification::source_contact::cuda_test {
namespace sc = tlfea::contact;
inline constexpr unsigned Workers = 2;

// Qualification packets preserve the real global node indices. Repeated
// endpoint arrays represent independent prescribed experiments, not owners.
struct ParentInput {
    unsigned arity = 0;
    sc::SurfaceQ4 quad;
    sc::SurfaceTriangle triangle;
    sc::T3MaterialMeasure triangle_reference;
    sc::Q4CertifiedIntegral area;
    double position[3 * NodeCount]{};
    double velocity[3 * NodeCount]{};
    std::uint64_t attempt = 31;
};
struct Input {
    ParentInput parents[ParentCount];
    double inverse_mass[NodeCount]{};
    std::uint8_t fixed[NodeCount]{};
    double wall_x = 0;
};
struct ParentOutput {
    sc::Q4RectangularResult quad;
    sc::T3IntegrationResult triangle;
};
struct Result {
    ParentOutput parents[ParentCount];
    sc::Q4IntegrationReport quad_reports[ParentCount];
    sc::T3IntegrationReport triangle_reports[ParentCount];
    bool accepted[ParentCount]{};
};
struct WorkerScratch {
    sc::Q4RectangularCell cells[sc::MaxQ4IntegrationLeaves];
    std::uint32_t heap[sc::MaxQ4IntegrationLeaves];
};
struct Storage {
    Input input;
    Result result;
    WorkerScratch scratch[Workers];
};
static_assert(sizeof(Storage) < 2 * 1024 * 1024,
              "Prescribed source GPU gate must stay below its 2 MiB allocation cap");

// Pure TL arithmetic after the host fixture's actual finite-wall preflight.
// One startup allocation, two bounded scratch regions, no physical assembly.
class Device {
  public:
    Device();
    ~Device();
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    cudaError_t initialization_status() const { return initialization_status_; }
    cudaError_t Evaluate(const Input&, Result&, float& elapsed_ms);
  private:
    Storage* storage_ = nullptr;
    cudaEvent_t begin_ = nullptr, end_ = nullptr;
    cudaError_t initialization_status_ = cudaSuccess;
};
} // namespace crash::qualification::source_contact::cuda_test
