#pragma once
#include "SourceAssemblyAcceptedOutput.h"
#include "SourceAssemblyWallSchema.h"
#include "case/source_assembly_dynamics/Case.h"

namespace crash::output::assembly {
namespace dynamics=cases::source_assembly_dynamics;
// Production adapter: accepted live case + output captured by that case. Calls
// are serialized with stepping; none of these formatters advances mechanics.
Document SourceAssemblyWallFrameFields(const dynamics::SourceAssemblyWallCase&,const SourceAssemblyAcceptedOutput&);
Document SourceAssemblyWallConfiguration(const dynamics::SourceAssemblyWallCase&,const SourceAssemblySurface&,const WallArchiveRequest&);
std::string SourceAssemblyWallInterval(const tl::fea::NodalStamp& base,const dynamics::SourceAssemblyWallCase&);

namespace wall_fields {
// Formatting-only host seam. These borrowed values cannot prove acceptance and
// are not accepted by the production archive writer. Tests label synthetic
// values explicitly; only the production adapter supplies live accepted fields.
struct FrameView {
    const SourceAssemblySurface* surface=nullptr;
    const cases::source_assembly::SourceAssemblyBindings* bindings=nullptr;
    const cases::source_assembly::SourceAssemblyWallSetup* setup=nullptr;
    const tl::fea::NodalStamp* stamp=nullptr;
    tl::fea::HostNodalKinematicsView nodes;
    SectionView qeph,t3;
    const tl::fea::ShellBatchDiagnostics* captured_shells=nullptr;
    const dynamics::Diagnostics* diagnostics=nullptr;
    dynamics::ContactView contact;
};
void CheckFrame(const FrameView&);
void CheckCase(const dynamics::SourceAssemblyWallCase&);
Document FrameDocument(const FrameView&);
Document StampDocument(const tl::fea::NodalStamp&);
Document DiagnosticsDocument(const dynamics::Diagnostics&);
Document ContactDocument(const FrameView&);
Document SetupDocument(const cases::source_assembly::SourceAssemblyWallSetup&);
const source::Parent& ContactSourceParent(const cases::source_assembly::SourceAssemblyBindings&,
    const cases::ShellCollectionContactGeometry&,std::size_t weight_index);
Document InputDocument(const cases::source_assembly::SourceAssemblyBindings&);
Document ConfigurationDocument(const cases::source_assembly::SourceAssemblyBindings&,
    const cases::source_assembly::SourceAssemblyWallSetup&,const dynamics::Config&,
    const SourceAssemblySurface&,const WallArchiveRequest&);
std::string IntervalRow(const tl::fea::NodalStamp& base,const dynamics::Diagnostics&,dynamics::ContactView);
} // namespace wall_fields
} // namespace crash::output::assembly
