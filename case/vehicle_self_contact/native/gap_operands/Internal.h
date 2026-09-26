#pragma once
#include "../ContactGapOperands.h"
#include "output/ArtifactIO.h"
#include "modelio/source_assembly/SourceAssemblyData.h"
#include "lib_src/math/ScalarBits.h"
#include <stdexcept>
namespace crash::cases::vehicle_self_contact::native::gap_operands::detail {
using output::Require;
struct Failure:std::runtime_error {
    Report report;
    explicit Failure(Report r):std::runtime_error(r.reason),report(std::move(r)){}
};
[[noreturn]] inline void Reject(Status status,const std::string& why,std::uint64_t eid=0) {
    throw Failure({status,why,eid});
}
struct Packed {
    std::vector<values::PhysicalShell> shells;
    std::vector<values::Line> beams;
    std::vector<values::Spring> springs;
    std::vector<Binding> bindings;
    Counts counts;
    SourceProof proof;
};
Forecast Budget(const source::CorrectedNodalSource&,Limits);
SourceProof Context(const source::CorrectedNodalSource&);
std::size_t CheckKeywordInventory(const output::Value&);
void CheckMaximumTerm(double);
double PropertyThickness(const modelio::assembly::Section&,n::UnitScale);
void Shells(const source::CorrectedNodalSource&,Packed&);
void Lines(const source::CorrectedNodalSource&,Packed&);
void Springs(const source::CorrectedNodalSource&,Packed&);
std::string Digest(const Packed&,const Provenance&,std::size_t);
inline bool SameUnits(n::UnitScale a,n::UnitScale b) {
    return tl::math::SameScalarBits(a.length_m,b.length_m)&&tl::math::SameScalarBits(a.mass_kg,b.mass_kg)&&
        tl::math::SameScalarBits(a.time_s,b.time_s);
}
}
