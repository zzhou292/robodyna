#pragma once
#include "Values.h"
#include "../TopologyDigestFields.h"
#include "lib_utils/BoundedArena.h"
#include <stdexcept>
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
using output::Require;
namespace ids=modelio::native_spring_ids;
struct Failure:std::runtime_error {
    Report report;
    explicit Failure(Report r):std::runtime_error(r.reason),report(std::move(r)){}
};
[[noreturn]] inline void Reject(Status status,const char* message,const std::string& file={},std::size_t line=0) {
    throw Failure({status,message,file,line});
}
struct Namespace {
    std::vector<Interface> interfaces;
    std::uint64_t self=0,wall=0;
    std::size_t checked_blocks=0;
    PopulationRange population;
};
Namespace ResolveNamespace(const MainSource&,const ids::ImportContext&,const ids::ImportMembers&,const Wall*,Limits);
void Check(const MainSource&,const Wall*);
std::size_t RetainedNamespace(const Namespace&,std::size_t cap);
Forecast Budget(const MainSource&,const ids::ImportMembers&,const Wall*,Limits);
std::string Digest(const MainSource&,const Wall*,const RawControls&,const GapScalars&,const Namespace&,Limits);
}
