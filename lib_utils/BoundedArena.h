// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>

namespace tl::util {
struct ArenaRegion {
  std::size_t offset=0,count=0,bytes=0;
};
// Allocation-free layout builder. A failed append preserves both the layout
// extent and the caller's region. Alignment/count arithmetic precedes any
// borrowed reads or allocation. Regions are not a persisted serialization ABI.
class BoundedArenaLayout {
 public:
  explicit BoundedArenaLayout(std::size_t maximum_bytes) noexcept:cap_(maximum_bytes) {}
  template<class T> bool Append(std::size_t count,ArenaRegion& output) noexcept {
    static_assert(alignof(T)<=alignof(std::max_align_t),"Only default-aligned resident records are admitted");
    const auto remainder=bytes_%alignof(T);
    const auto padding=remainder?alignof(T)-remainder:0;
    if(bytes_>cap_||padding>cap_-bytes_) return false;
    const auto offset=bytes_+padding;
    if(count>(cap_-offset)/sizeof(T)) return false;
    const auto extent=count*sizeof(T);
    output={offset,count,extent}; bytes_=offset+extent; return true;
  }
  std::size_t bytes() const noexcept { return bytes_; }
 private:
  std::size_t cap_=0,bytes_=0;
};

template<class T> T* ArenaPointer(void* base,const ArenaRegion& region) noexcept {
  return reinterpret_cast<T*>(static_cast<unsigned char*>(base)+region.offset);
}
template<class T> const T* ArenaPointer(const void* base,const ArenaRegion& region) noexcept {
  return reinterpret_cast<const T*>(static_cast<const unsigned char*>(base)+region.offset);
}

// Fresh startup staging only. Typed placement construction begins the lifetime
// of every copied record; trivially destructible records require no teardown
// walk. No device allocation, clock, accepted index or per-step growth lives here.
class HostArena {
 public:
  HostArena()=default;
  ~HostArena() { ::operator delete(data_); }
  HostArena(const HostArena&)=delete;
  HostArena& operator=(const HostArena&)=delete;
  bool Initialize(std::size_t bytes) noexcept {
    if(data_||!bytes) return false;
    auto* next=::operator new(bytes,std::nothrow);
    if(!next) return false;
    std::memset(next,0,bytes); data_=next; bytes_=bytes; return true;
  }
  template<class T> T* Construct(const ArenaRegion& region) noexcept {
    static_assert(alignof(T)<=alignof(std::max_align_t),"Only default-aligned resident records are admitted");
    static_assert(std::is_trivially_copyable_v<T>&&std::is_trivially_destructible_v<T>,"Arena records must be copied values");
    static_assert(std::is_nothrow_default_constructible_v<T>,"Startup record construction cannot throw");
    if(!data_||region.offset>bytes_||region.bytes>bytes_-region.offset||region.offset%alignof(T)||
       region.count>std::numeric_limits<std::size_t>::max()/sizeof(T)||region.bytes!=region.count*sizeof(T)) return nullptr;
    auto* pointer=ArenaPointer<T>(data_,region);
    return ::new(static_cast<void*>(pointer)) T[region.count]{};
  }
  void* data() noexcept { return data_; }
  const void* data() const noexcept { return data_; }
  std::size_t bytes() const noexcept { return bytes_; }
 private:
  void* data_=nullptr;
  std::size_t bytes_=0;
};
} // namespace tl::util
