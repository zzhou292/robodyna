#pragma once
#include "SourceNodalWallFixture.h"
#include "collision/PreparedPlanarWallQuery.h"
#include <cuda_runtime.h>

namespace crash::qualification::source_contact::nodal {
static_assert(Workers==sc::MaxNodalWallNodes && Workers==sc::MaxNodalWallParents &&
              NodeCount<=Workers && ParentCount<=Workers,"One block covers every staged row exactly once");
struct Storage {
    Input input;
    sc::PreparedPlanarWallQuery query;
    sc::NodalWallPointResult shares[MaxShares];
    Report node_report[Workers],parent_report[Workers],report;
    Result trial,published;
};
static_assert(sizeof(Storage)<512*1024,"Retain room for the unchanged 1.52 MiB integral baseline");
struct Timing { float kernel_ms=0; double checked_end_to_end_ms=0; };

// Qualification object only: one allocation and two events; no owner, clock,
// device stack-limit change or production selector. Every call evaluates one
// coherent state. One block has exactly128 threads across query/law/reduction.
// Semantic failures retain device-published and caller result and allow retry;
// any CUDA failure poisons this object. Input/status diagnostics are not accepted
// state. Event timing includes face query, every law and the full device reduction.
class Device {
  public:
    Device();
    ~Device();
    Device(const Device&)=delete;
    Device& operator=(const Device&)=delete;
    cudaError_t initialization_status() const { return status_; }
    cudaError_t Evaluate(const Input&,Result&,Report&,Timing&);
  private:
    cudaError_t Fail(cudaError_t);
    Storage* storage_=nullptr;
    cudaEvent_t begin_=nullptr,end_=nullptr;
    cudaError_t status_=cudaSuccess;
};
// Safe invalid-launch injection only: no kernel is executed. Leaves the exact
// runtime error pending for the next Device::Evaluate poison check.
cudaError_t InjectInvalidLaunchForCheck();
} // namespace crash::qualification::source_contact::nodal
