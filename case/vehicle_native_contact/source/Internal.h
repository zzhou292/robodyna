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
inline std::size_t Extras(const Forecast& f) {
    return Add(Add(f.member_storage_bytes,f.packet_authority_reservation),f.metadata_bytes);
}
inline void Admit(Forecast& f,std::size_t& phase,std::size_t existing,std::size_t producer,Limits limits) {
    const auto bytes=Add(Add(existing,producer),Extras(f));
    output::Require(bytes<=limits.host_bytes,"Complete native V6 source phase exceeds host cap");
    phase=std::max(phase,bytes);f.peak_bytes=std::max(f.peak_bytes,bytes);
}
std::shared_ptr<const Inputs> PrepareInputs(const vehicle_run::OriginalPaths&,
    const modelio::solid_control_packets::Artifact&,Limits,Forecast&);
Owner PrepareOwner(const Inputs&,const vehicle_run::OriginalPaths&,Limits,Forecast&);
struct Contact {Self self;Wall wall;Controls controls;};
Contact PrepareContact(const Inputs&,const Owner&,Limits,Forecast&);
} // namespace crash::cases::vehicle_native_contact::source::detail
