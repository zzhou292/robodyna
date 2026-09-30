#include "AllocationFailure.h"
#include <cstdlib>
#include <new>
namespace { thread_local std::ptrdiff_t until_failure=-1; }
namespace host_shell_test {
AllocationFailure::AllocationFailure(std::ptrdiff_t after) noexcept { until_failure=after; }
AllocationFailure::~AllocationFailure() { until_failure=-1; }
}
void* operator new(std::size_t bytes) {
  if(until_failure==0) throw std::bad_alloc();
  if(until_failure>0) --until_failure;
  if(void* p=std::malloc(bytes?bytes:1)) return p;
  throw std::bad_alloc();
}
void* operator new[](std::size_t bytes) { return ::operator new(bytes); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p,std::size_t) noexcept { std::free(p); }
void operator delete[](void* p,std::size_t) noexcept { std::free(p); }
