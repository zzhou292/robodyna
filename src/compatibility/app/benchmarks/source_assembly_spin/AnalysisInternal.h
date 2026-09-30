#pragma once
#include "Analysis.h"
#include "modelio/source_assembly/SourceAssembly.h"
#include "modelio/source_assembly/SourceAssemblyMaterialInput.h"
#include "modelio/source_assembly/SourceAssemblyShellInput.h"
#include "modelio/source_assembly/JsonReader.h"
#include "benchmarks/source_assembly_pilot/Metrics.h"
#include "lib_utest/qualification/shell_layered_j2/native_recurrence/NativeLayeredReference.h"
#include <vector>

namespace crash::benchmarks::assembly_spin {
namespace source=modelio::assembly;namespace json=source::reader;
namespace native=tl::qualification::qeph;namespace layered=tl::qualification::layered_native;
using output::Document;using output::Value;using output::Require;using assembly_pilot::Metric;using assembly_pilot::Metrics;
inline constexpr std::size_t RowCap=64*1024,TraceCap=256*1024*1024,MaxRows=4094;
const Value& Numbers(const Value&,const char*,std::size_t);
template<std::size_t N> void Array(const Value& v,const char* key,std::array<double,N>& out) {
    const auto& a=Numbers(v,key,N);for(std::size_t i=0;i<N;++i)out[i]=json::Real(a[static_cast<unsigned>(i)]);
}
struct ParentSource { const source::Parent* parent=nullptr;std::size_t local=0;native::Reference reference;layered::Law44 law; };
struct Context {
    source::SourceAssembly inventory;
    source::SourceAssemblyMaterialInput materials;
    std::vector<ParentSource> parents;
    std::uint64_t source_node=0,source_instance=0,owner=0,requested=0,cadence=0;
    std::size_t global_node=0,group_count=0,member_count=0;double dt=0;
    explicit Context(const std::filesystem::path&);
    void Header(const Value&);
    void Stamp(const Value&,std::uint64_t epoch) const;
};
void CheckSharedMotion(const Context&,const Value&);
void CheckParent(const Context&,const ParentSource&,const Value&,std::uint64_t epoch,double time,double origin);
layered::QephHistory History(const native::Reference&,const Value&);
native::PrescribedInterval Interval(const Value&,double dt);
void Compare(Metrics&,const ParentSource&,const Value& candidate,const layered::QephTrial&);
Document LocalDiagnostic(const Context&,const ParentSource&,const Value& row,const Value& base,const Value& candidate,const layered::QephTrial& native_actual);
Document Parse(const char*,std::size_t);
}
