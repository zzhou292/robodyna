#pragma once
#include "WallSource.h"
#include "case/vehicle_wall/EnvelopeWall.h"
#include "modelio/source_assembly/SourceFields.h"
#include "modelio/tied_shell/Internal.h"
#include "output/BoundedArrayIO.h"
#include <stdexcept>
namespace crash::cases::vehicle_wall::native::detail {
namespace source=output::full_shell::source;
namespace springs=modelio::native_spring_ids;
using output::Require;
struct Error:std::runtime_error {
    Report report;
    explicit Error(Report value):std::runtime_error(value.reason),report(std::move(value)){}
};
[[noreturn]] inline void Reject(Status status,const std::string& why,const std::string& file={},
        std::size_t row=SIZE_MAX,std::uint64_t id=0){throw Error({status,why,file,row,id});}
void Check(const Declaration&);
void CheckLimits(Limits);
std::array<tlfea::contact::Vec3,2> Bounds(const tl::fea::NodalNodeDomain&);
std::size_t NamespaceWorkspace(const source::CanonicalData&,const springs::ImportMembers&,Limits);
std::pair<NamespaceReport,AllocatedIds> Namespace(const springs::ImportContext&,
    const springs::ImportMembers&,const std::string& original_wall_sha256,Limits);
std::pair<NamespaceReport,AllocatedIds> NamespaceValues(const source::CanonicalData&,
    const springs::ContextData&,const springs::ImportMembers&,const std::string&,Limits);
struct DomainValues {tl::fea::NodalNodeDomain domain;std::vector<std::uint8_t> fixed,rotation;};
DomainValues BuildDomain(const tl::fea::NodalNodeDomain&,const AllocatedIds&,const Geometry&,Limits);
Geometry BuildGeometry(const std::array<tlfea::contact::Vec3,2>&,const Declaration&,
    const AllocatedIds&,const source::Units&);
std::string Digest(const source::CanonicalData&,const Declaration&,const NamespaceReport&,
    const AllocatedIds&,const Geometry&,const tl::fea::NodalNodeDomain&,const VehiclePrefix&,std::size_t);
} // namespace crash::cases::vehicle_wall::native::detail
