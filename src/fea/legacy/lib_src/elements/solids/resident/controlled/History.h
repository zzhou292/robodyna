// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid24/controlled_distortion/Types.h"
#include "lib_src/elements/solid18/total_strain/controlled_distortion/Types.h"
#include <new>
#include <type_traits>
namespace tl::fea::solids::batch_detail::controlled {
namespace h24=solid24::controlled_distortion;
namespace foam=solid18::total_strain::controlled_distortion;
enum class HistoryKind:unsigned { Legacy,NativeControlled };
TL_BRICK_HD inline auto NativeStamp(const h24::History& h)noexcept{return h.stamp();}
TL_BRICK_HD inline auto NativeStamp(const foam::History& h)noexcept{auto stamp=h.native_history().stamp();stamp.time_s*=h.units().time_s;return stamp;}
// The active union member is explicit. Copying this trivially-copyable value
// preserves the active member; switching profile begins that member's lifetime.
template<class Legacy,class Native> class History {
  union Values {Legacy legacy;Native native;TL_BRICK_HD Values()noexcept:legacy{}{};} values_;
  HistoryKind kind_=HistoryKind::Legacy;
 public:
  TL_BRICK_HD History()noexcept=default;
  TL_BRICK_HD History& operator=(const Legacy& value)noexcept {
    if(kind_!=HistoryKind::Legacy)::new(static_cast<void*>(&values_.legacy)) Legacy(value);
    else values_.legacy=value;
    kind_=HistoryKind::Legacy;return *this;
  }
  TL_BRICK_HD void SetNative(const Native& value)noexcept {
    if(kind_!=HistoryKind::NativeControlled)::new(static_cast<void*>(&values_.native)) Native(value);
    else values_.native=value;
    kind_=HistoryKind::NativeControlled;
  }
  TL_BRICK_HD HistoryKind kind()const noexcept{return kind_;}
  TL_BRICK_HD const Legacy* legacy()const noexcept{return kind_==HistoryKind::Legacy?&values_.legacy:nullptr;}
  TL_BRICK_HD const Native* native()const noexcept{return kind_==HistoryKind::NativeControlled?&values_.native:nullptr;}
  TL_BRICK_HD auto stamp()const noexcept {
    return kind_==HistoryKind::NativeControlled?NativeStamp(values_.native):values_.legacy.stamp();
  }
};
using History24=History<solid24::History,h24::History>;
using History90=History<solid18::total_strain::History,foam::History>;
static_assert(std::is_trivially_copyable_v<History24>&&std::is_trivially_copyable_v<History90>);
static_assert(std::is_trivially_destructible_v<History24>&&std::is_trivially_destructible_v<History90>);
static_assert(std::is_nothrow_default_constructible_v<History24>&&std::is_nothrow_default_constructible_v<History90>);
} // namespace tl::fea::solids::batch_detail::controlled
