#pragma once
#include <cstddef>
namespace host_shell_test {
// A separate host-test executable owns the global new override. Failure is
// active only around the operation under test, never around gtest assertions.
class AllocationFailure {
 public:
  explicit AllocationFailure(std::ptrdiff_t after=0) noexcept;
  ~AllocationFailure();
  AllocationFailure(const AllocationFailure&)=delete;
};
}
