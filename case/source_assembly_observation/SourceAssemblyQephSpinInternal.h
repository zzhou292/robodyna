#pragma once
#include "SourceAssemblyQephSpin.h"
#include "SourceAssemblyObservationInternal.h"
namespace crash::cases::source_assembly_observation::spin_detail {
struct Selection {std::size_t node=SIZE_MAX,count=0;std::array<std::size_t,MaxSpinParents> parents{},locals{};};
Report Select(const SourceAssemblyBindings&,std::uint64_t,Selection&) noexcept;
Report Check(const QephSpinInput&,const void*,std::size_t,Selection&) noexcept;
Report Parent(const QephSpinInput&,const modelio::assembly::Parent&,std::size_t,bool candidate,QephSpinParent&) noexcept;
}
