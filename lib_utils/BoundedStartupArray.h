// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
#include <cstddef>
#include <memory>

namespace tl::util {
// Caller validates count and byte caps BEFORE Resize or ExtraBytes arithmetic.
// Internal startup storage: mutate only a fresh staging object, then publish it
// behind the immutable binding/catalog API. Inline initialization and all value
// copies allocate nothing. Shared backing has exact array extent; reserve 64
// bytes per shared control allocation in owned-byte admission (allocator heap
// bookkeeping is outside this payload budget). No pointer targets caller data.
template<class T,std::size_t InlineCapacity> class BoundedStartupArray {
 public:
  void Resize(std::size_t count) {
    if(count>InlineCapacity) {
      std::shared_ptr<T[]> next(new T[count]{});
      backing_=std::move(next); extent_=count;
    }
  }
  static constexpr std::size_t ExtraBytes(std::size_t count) noexcept {
    return count>InlineCapacity?count*sizeof(T)+64:0;
  }
  std::size_t backing_bytes() const noexcept { return backing_?ExtraBytes(extent_):0; }
  std::size_t size() const noexcept { return extent_; }
  T* data() noexcept { return backing_?backing_.get():inline_.data(); }
  const T* data() const noexcept { return backing_?backing_.get():inline_.data(); }
  T& operator[](std::size_t i) noexcept { return data()[i]; }
  const T& operator[](std::size_t i) const noexcept { return data()[i]; }
 private:
  std::array<T,InlineCapacity> inline_{};
  std::shared_ptr<T[]> backing_;
  std::size_t extent_=InlineCapacity;
};

template<class T> class ConstView {
 public:
  ConstView(const T* data,std::size_t count) noexcept:data_(data),count_(count) {}
  const T* data() const noexcept { return data_; }
  std::size_t size() const noexcept { return count_; }
  const T* begin() const noexcept { return data_; }
  const T* end() const noexcept { return data_+count_; }
  const T& operator[](std::size_t i) const noexcept { return data_[i]; }
 private:
  const T* data_;
  std::size_t count_;
};
} // namespace tl::util
