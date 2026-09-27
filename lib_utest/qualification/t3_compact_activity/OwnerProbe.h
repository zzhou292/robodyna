// SPDX-License-Identifier: MIT
#pragma once
#include "../t3_activity_readback/OwnerSupport.h"
namespace t3_compact_owner {
struct Probe {
  std::size_t fail_copy=0,fail_sync=0,fail_error=0,error_checks=0,packet_corruption=0;
  void* packet_host=nullptr;
  bool capture_sources=false;std::size_t point_sized_copies=0;
  const void* point_device=nullptr;const void* force_device=nullptr;
};
extern Probe probe;
inline void Watch(){t3_readback_test::Watch(1);probe={};}
inline void Stop(){t3_readback_test::transfers.enabled=false;}
}
