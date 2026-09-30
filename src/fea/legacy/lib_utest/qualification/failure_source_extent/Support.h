// SPDX-License-Identifier: MIT
#pragma once
#include "lib_utest/qualification/shell_output_ranges/Fixture.h"
#include "lib_utest/qualification/resident_shell_failure/FailureResidentSource.h"
#include "SourceChecks.h"
#include <limits>
#include <utility>
namespace failure_source_extent_test {
namespace fe=tl::fea;
namespace storage=fe::shell_batch_plasticity_detail;
namespace reference=storage::extent_reference;
using Family=fe::ShellBindingFamily;
using Status=storage::SetupStatus;
inline void Compare(const fe::ShellBatchFailureBinding& binding,Family family,
    std::size_t count,bool expected,bool device=true,unsigned slab=0,std::size_t owned=SIZE_MAX) {
  if(owned==SIZE_MAX)owned=count;
  const auto old=reference::FrozenCheck(device,binding,family,owned,slab,count);
  const auto now=reference::CurrentCheck(device,binding,family,owned,slab,count);
  EXPECT_EQ(now.status,old.status);EXPECT_STREQ(now.message,old.message);
  EXPECT_EQ(now.cuda_status,old.cuda_status);
  EXPECT_EQ(now.status,expected?Status::Success:Status::InvalidInput);
}
} // namespace failure_source_extent_test
