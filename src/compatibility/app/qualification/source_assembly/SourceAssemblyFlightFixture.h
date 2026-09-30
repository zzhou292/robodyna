#pragma once
#include "case/source_assembly/tests/SourceAssemblyBindingTestSupport.h"
#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
#include "lib_src/solvers/ExplicitTranslationStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <vector>

namespace crash::qualification::source_assembly {
namespace fe=tl::fea;
namespace q=fe::qeph;
namespace t=fe::t3;
namespace src=crash::modelio::assembly;
using cases::source_assembly::SourceAssemblyBindings;
// Four tiny intervals qualify source storage/free flight, never crash admission.
inline constexpr double TimeStep=1./67108864,Speed=8;
inline constexpr std::uint64_t Configuration=0x53414750555331ULL,Qualification=0x5341464c5431ULL;
struct Fields {
    explicit Fields(std::size_t nodes):x(3*nodes),v(3*nodes),w(3*nodes),orientation(4*nodes),reaction(3*nodes),couple(3*nodes) {}
    std::vector<double> x,v,w,orientation,reaction,couple;
    fe::NodalStamp stamp;
    fe::NodalSnapshotBuffer buffer() { return {x.data(),v.data(),x.size()/3,orientation.data(),w.data(),reaction.data(),couple.data()}; }
};
template<class Section> struct BasicShellFields {
    BasicShellFields(std::size_t quads,std::size_t triangles):quad(quads),triangle(triangles),qsection(quads),tsection(triangles) {}
    std::vector<q::ForceTrial> quad;
    std::vector<t::ForceTrial> triangle;
    std::vector<Section> qsection,tsection;
    fe::ShellBatchDiagnostics diagnostics;
};
using ShellFields=BasicShellFields<fe::ShellBatchSectionState>;
using LayeredShellFields=BasicShellFields<fe::ShellBatchLayeredSection>;
struct Prepared {
    explicit Prepared(std::size_t nodes):endpoint(nodes) {}
    fe::NodalTrialToken token;
    fe::NodalPreparedView view;
    Fields endpoint;
};
// Test composition only: no step policy, independent clock or new solver.
// Destruction order remains publication, batches, owner, immutable source.
struct Rig {
    explicit Rig(const src::SourceAssembly& source,bool attach_groups=false);
    SourceAssemblyBindings bindings;
    const bool groups_attached;
    cases::ShellCollectionContactGeometry contact_geometry;
    Fields initial;
    std::vector<double> inverse_mass,inverse_inertia;
    std::vector<std::uint8_t> free;
    fe::FENodalState owner;
    q::QephBatch qeph;
    t::T3Batch t3;
    fe::ShellBatchPublication publication;
    std::size_t nodes() const { return bindings.shells().node_count(); }
    std::size_t quads() const { return bindings.shells().qeph_count(); }
    std::size_t triangles() const { return bindings.shells().t3_count(); }
    bool Initialize();
    void Discard() { owner.Discard(); publication.DiscardTrial(); qeph.DiscardTrial(); t3.DiscardTrial(); }
};
bool Capture(Rig&,Fields&,ShellFields&);
bool Prepare(Rig&,Prepared&);
bool Evaluate(Rig&,const Prepared&,ShellFields&);
bool Publish(Rig&,const Prepared&,const ShellFields&);
bool Capture(Rig&,Fields&,LayeredShellFields&);
bool Evaluate(Rig&,const Prepared&,LayeredShellFields&);
bool Publish(Rig&,const Prepared&,const LayeredShellFields&);
void CheckSource(const Rig&);
void CheckFlight(const Rig&,const Fields&,const ShellFields&);
void CheckFlightMotion(const Rig&,const Fields&,const fe::ShellBatchDiagnostics&);
void CheckFlight(const Rig&,const Fields&,const LayeredShellFields&);
void CheckHostParents(const Rig&,const Prepared&,const ShellFields& accepted,const ShellFields& proposed);
void SameShells(const ShellFields&,const ShellFields&);
void SameShells(const LayeredShellFields&,const LayeredShellFields&);
void CheckHostParents(const Rig&,const Prepared&,const LayeredShellFields&,const LayeredShellFields&);
void SameFields(const Fields&,const Fields&);
std::array<fe::NodalAllocationInfo,4> Allocations(const Rig&);
void SameAllocations(const Rig&,const std::array<fe::NodalAllocationInfo,4>&);
}
