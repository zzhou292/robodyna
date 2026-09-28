// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Plan.h"
#include "../runtime/PhysicalMainSource.h"
#include "../runtime/physical_main/Index.h"
#include "lib_utils/BoundedArena.h"
#include <optional>
namespace tlfea::contact::radioss_type25::activity_source::detail {
namespace pm=runtime_detail::physical_main;
using S=TransactionStatus;
inline TransactionReport Ok(){return {S::Ok,"Activity source binding ready"};}
inline TransactionReport Fail(S s,const char* text,std::size_t row=SIZE_MAX){return {s,text,row};}
struct ParentRow { ParentIdentity identity;std::uint32_t nodes[8]{};unsigned count=0; };
using Visit=bool(*)(void*,const ParentRow&);
TransactionReport Parents(PhysicalSources,Counts&,void*,Visit) noexcept;
struct Source {
  const ContactSourceInput* ordinary=nullptr;
  const startup::MixedSidesSnapshot* mixed=nullptr;
  const startup::PostGapmTopology* post=nullptr;
  std::size_t primaries() const {return ordinary?ordinary->primary_main_count:mixed->primary_count;}
  const std::uint32_t* nodes(std::size_t i) const {return ordinary?ordinary->selection.mains[i].nodes:mixed->mains[i].nodes;}
  std::size_t mains() const {return ordinary?ordinary->selection.main_count:mixed->main_count;}
  std::size_t origins() const {return ordinary?ordinary->primary_main_count:mixed->raw_origin_count;}
  std::uint64_t generation() const {return ordinary?ordinary->selection.generation:mixed->source_generation;}
};
TransactionReport CheckSource(const tl::fea::ShellPhysicalBinding&,Source,Controls,std::size_t);
TransactionReport WriteMains(const pm::Index&,Source,std::size_t shell_count,MainSupport*,Origin*);
startup::MixedSidesSnapshot Mixed(const startup::Snapshot&);
struct Layout {
  tl::util::ArenaRegion parents,offsets,incidence,mains,origins,main_to_primary,containing_offsets,containing_parents;
  std::size_t bytes=0;
};
struct Storage {
  explicit Storage(PhysicalSources source):physical(source.binding){if(source.type45)type45.emplace(*source.type45);}
  tl::fea::ShellPhysicalBinding physical;
  std::optional<tl::fea::type45::Model> type45;
  tl::util::HostArena arena;
  View view;
  Forecast forecast;
};
Forecast Preflight(PhysicalSources,Source,Controls,Limits,Layout* =nullptr) noexcept;
TransactionReport Build(PhysicalSources,Source,Controls,Limits,
    std::unique_ptr<Storage>&) noexcept;
}
