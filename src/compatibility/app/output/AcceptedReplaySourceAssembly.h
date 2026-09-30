#pragma once
#include "AcceptedReplayData.h"
#include "AcceptedReplayValues.h"
#include "modelio/source_assembly/SourceAssembly.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::output::replay_detail {
namespace source = modelio::assembly;
// Read-only declarations and recorded diagnostics. This is not a material/state
// owner and intentionally has no native mechanics or constrained recurrence.
struct AssemblyConnectorDeclaration {
    std::uint64_t element=0,property=0;
    std::array<std::size_t,2> nodes{};
    std::array<double,3> transverse{};
    double length=0;
};
struct AssemblyReplayData {
    explicit AssemblyReplayData(source::SourceAssembly input):source(std::move(input)) {}
    source::SourceAssembly source;
    std::uint64_t instance=0,asset=0,requested_steps=0,frame_every=0;
    std::size_t group_count=0,member_count=0;
    bool observe_force_stage=false;
    bool native_rotation_domain=false;
    std::vector<std::array<double,4>> native_nodes; // Shell coefficients, then total M/J after connector admission.
    std::vector<std::array<double,2>> connector_nodes; // Subtotals already included in total M/J.
    std::vector<AssemblyConnectorDeclaration> connectors;
    double connector_dt_fraction=0;
    std::vector<bool> grouped_node;
    std::vector<std::size_t> contact_source_parent;
    std::vector<std::string> interval_files;
    std::vector<std::array<double,33>> intervals; // Saved endpoints only.
    double maximum_plastic=0,minimum_thickness_ratio=0,maximum_thickness_ratio=0;
    double maximum_rotation=0,maximum_area_ratio=0,maximum_displacement=0;
    std::array<double,4> initial_native{},initial_aggregate{};
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall;
    Document final_diagnostics;
};
inline const Value& AssemblyRow(const Value& a,std::size_t size) {
    Require(a.IsArray()&&a.Size()==size,"Assembly replay row shape mismatch");return a;
}
inline std::uint64_t AssemblyId(const Value& a) {
    Require(a.IsUint64(),"Assembly replay source identifier type mismatch");return a.GetUint64();
}
inline double AssemblyNumber(const Value& a) {
    Require(a.IsNumber()&&std::isfinite(a.GetDouble()),"Assembly replay scalar is nonfinite");return a.GetDouble();
}
inline const Value& AssemblyNumbers(const Value& a,std::size_t size) {
    AssemblyRow(a,size);for(const auto& v:a.GetArray())AssemblyNumber(v);return a;
}
inline void AssemblyEqual(double a,double b) {Require(Bits(a)==Bits(b),"Assembly replay scalar identity changed");}
inline void AssemblyNear(long double actual,long double expected,std::size_t terms=1) {
    const auto bound=128.L*std::numeric_limits<double>::epsilon()*std::max({std::abs(actual),std::abs(expected),1e-300L})*terms;
    Require(std::isfinite(actual)&&std::isfinite(expected)&&std::isfinite(bound)&&std::abs(actual-expected)<=bound,
        "Assembly replay diagnostic reduction disagrees");
}
inline void AssemblyReduction(long double actual,long double expected,long double magnitude,std::size_t terms=1) {
    const auto bound=128.L*std::numeric_limits<double>::epsilon()*magnitude*terms;
    Require(std::isfinite(actual)&&std::isfinite(expected)&&std::isfinite(bound)&&magnitude>=0&&std::abs(actual-expected)<=bound,
        "Assembly replay signed diagnostic reduction disagrees");
}
inline void AssemblyIds(const Value& a,const std::vector<source::SourceId>& ids) {
    AssemblyRow(a,ids.size());for(std::size_t i=0;i<ids.size();++i)Require(AssemblyId(a[i])==ids[i],"Assembly source ID sequence changed");
}
struct AssemblyKineticChannels {
    std::array<double,6> ordinary{},members{};
    std::array<double,14> groups{};
};
inline double AssemblyQuaternionLimit(const AssemblyReplayData& a) {
    return a.native_rotation_domain?std::acos(-1.):a.maximum_rotation;
}
void CheckAssemblySourceScope(const Value& configuration,const Value& manifest,bool seven);
source::ArtifactIdentity AssemblySourceIdentity(const Value& configuration);
void ReadAssemblyConnectors(Bundle&,const Value& configuration);
void CheckAssemblyConnectors(const Bundle&,const Entry&,const Value& frame);
std::array<double,2> AssemblyConnectorKinetic(const Bundle&,const Value&);
void ReadAssemblyRotationDomain(AssemblyReplayData&,const Value&);
void CheckAssemblyRotation(const Bundle&,const Entry&,const Value&,const Value*);
AssemblyKineticChannels CheckAssemblyKineticChannels(const Bundle&,const Value&);
void CheckAssemblyForceStage(const Bundle&,const Entry&,const Value& frame);
void CheckAssemblySurface(const Bundle&,const Value&);
void ReadAssemblyDeclarations(Bundle&,const Value&);
void ReadAssemblyGroups(Bundle&,const Value&);
void ReadAssemblyWallSetup(Bundle&,const Value&);
void ReadAssemblyIntervals(Bundle&,const Document&,const Document&);
void CheckAssemblyStamp(const Bundle&,const Entry&,const Value&);
std::vector<ReplayParentScalar> CheckAssemblySections(const Bundle&,const Entry&,const Value&);
void CheckAssemblyDiagnostics(const Bundle&,const Entry&,const Value&,const Value* nodal);
void CheckAssemblyContact(const Bundle&,const Entry&,const Value&,const Value& nodal,const Value& diagnostics);
void ReadSourceAssemblyConfiguration(Bundle&,const Document&,const Document&,const Document&);
void CheckSourceAssemblyFields(const Bundle&,const Entry&,const chrono::ChTriangleMeshConnected&);
std::vector<ReplayParentScalar> ReadSourceAssemblyDisplay(const Bundle&,const Entry&);
// Shared placed wall authentication; caller provides the actual source bounds.
void CheckPlacedWall(Bundle&,const Value& setup,const std::array<double,3>& minimum,const std::array<double,3>& maximum);
} // namespace crash::output::replay_detail
