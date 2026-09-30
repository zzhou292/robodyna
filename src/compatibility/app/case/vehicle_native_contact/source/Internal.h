#pragma once
#include "OriginalSources.h"
#include "case/vehicle_run/source/OriginalSourceIO.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include "modelio/physical_domain/VehiclePhysicalDomain.h"
#include "modelio/native_spring_ids/ImportContext.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact::source::detail {
struct Members {
    std::string vehicle,auxiliary,combine,wall;
    modelio::native_spring_ids::ImportMembers Input()const;
    std::size_t bytes()const;
};
struct Inputs {
    Members members;
    output::full_shell::source::CanonicalSource canonical;
    modelio::vehicle::VehicleSectionResolution resolution;
    modelio::physical_domain::VehiclePhysicalDomain domain;
    modelio::solid_control_packets::NativePacketSource packets;
};
inline std::size_t Add(std::size_t a,std::size_t b) {
    output::Require(b<=SIZE_MAX-a,"Native V6 source construction budget overflows");return a+b;
}
// Same public identity evidence used by mixed_main/Controls.cpp. Only this
// named shared partition may be discounted; no private layouts are inferred.
struct CorrectedBacking {
    const void* coefficients=nullptr;std::size_t count=0;const void* canonical=nullptr;
};
inline std::size_t SharedCorrectedPeak(CorrectedBacking mixed,CorrectedBacking gap,
    std::size_t mixed_retained,std::size_t gap_peak,std::size_t gap_shared,std::size_t corrected_peak) {
    output::Require(mixed.coefficients&&mixed.count&&mixed.canonical&&
        mixed.coefficients==gap.coefficients&&mixed.count==gap.count&&mixed.canonical==gap.canonical,
        "Gap construction requires the exact mixed corrected source backing");
    output::Require(gap_shared==corrected_peak&&mixed_retained>=corrected_peak&&gap_peak>=gap_shared,
        "Gap construction shared source partition is inconsistent");
    return Add(gap_peak,mixed_retained-corrected_peak);
}
inline std::size_t Extras(const Forecast& f) {
    return Add(Add(f.member_storage_bytes,f.packet_authority_reservation),f.metadata_bytes);
}
inline void Admit(Forecast& f,std::size_t& phase,std::size_t existing,std::size_t producer,Limits limits,const char* stage) {
    const auto bytes=Add(Add(existing,producer),Extras(f));
    output::Require(bytes<=limits.host_bytes,(std::string("Native V6 source phase ")+stage+
        " exceeds host cap: total="+std::to_string(bytes)+" cap="+std::to_string(limits.host_bytes)+
        " existing="+std::to_string(existing)+" producer="+std::to_string(producer)+
        " extras="+std::to_string(Extras(f))).c_str());
    phase=std::max(phase,bytes);f.peak_bytes=std::max(f.peak_bytes,bytes);
}
std::shared_ptr<const Inputs> PrepareInputs(const vehicle_run::OriginalPaths&,
    const modelio::solid_control_packets::Artifact&,Limits,Forecast&);
Owner PrepareOwner(const Inputs&,const vehicle_run::OriginalPaths&,Limits,Forecast&);
struct Contact {Self self;Wall wall;Controls controls;};
Contact PrepareContact(const Inputs&,const Owner&,Limits,Forecast&);
} // namespace crash::cases::vehicle_native_contact::source::detail
