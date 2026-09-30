#pragma once
#include <cstdint>
#include <cstring>
namespace solid_parallel_test {
// Diagnostic only: canonical little-endian binary64 bits, independent of padding.
struct Fingerprint {
  std::uint64_t value = 14695981039346656037ull;
  void Integer(std::uint64_t bits) noexcept {
    for (unsigned i=0;i<8;++i) { value^=(bits>>(8*i))&255u;value*=1099511628211ull; }
  }
  void Real(double x) noexcept { std::uint64_t bits=0;std::memcpy(&bits,&x,sizeof(bits));Integer(bits); }
};
} // namespace solid_parallel_test
