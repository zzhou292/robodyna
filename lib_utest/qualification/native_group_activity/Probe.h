#pragma once
#include <cuda_runtime_api.h>
#include <cstddef>
namespace native_group_activity_test {
  struct Probe {
    bool enabled=false,capture_force=false;
    std::size_t copies=0,bytes=0,syncs=0,fail_copy=0;
    const void* force=nullptr;
  };
  extern Probe probe;
  inline void Watch(){
    probe={};
    probe.enabled=true;
  }
  inline Probe Stop(){
    auto x=probe;
    probe.enabled=false;
    probe.capture_force=false;
    return x;
  }
}
