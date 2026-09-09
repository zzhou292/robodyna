#pragma once

#include "GuidedPlateCase.h"
#include "GuidedPlateContactIdentity.h"

namespace crash::case_data::guided_detail {
namespace fea=tl::fea;
namespace shell=tl::fea::reissner;
namespace contact=tlfea::contact;
namespace ref=crash::reference;

// Canonical metadata adapter only. Retains source identities and a stable
// host mesh for contact initialization and later rendering of the same wall.
struct ContactWall {
    std::vector<contact::PlanarWallVertex> vertices;
    std::vector<contact::PlanarWallTriangle> triangles;
    WallProvenance provenance;
    contact::PlanarWallView view() const noexcept;
};
bool CopyCanonicalWall(const CanonicalWall&,ContactWall&,std::string& diagnostic);
// Publishes the actual derived C3 preparation only after original physical
// parent/projection/area and full all-active stiffness table agree exactly.
bool PrepareTessellatedContact(const WallTessellation&,const ref::GuidedPlateModel&,
                              contact::Q4PlanarGeometry&,std::string& diagnostic);
std::string Describe(const shell::ShellBatchReport&);
std::string Describe(const contact::Q4PlanarContactReport&);
visual::Binding SurfaceBinding(const fea::NodalStamp&,const ref::GuidedPlateData&);
std::array<shell::ReissnerShellBatchElement,ref::kCouponElements> ShellElements(const ref::ElasticCouponData&);

struct InitialState {
    std::array<double,3*ref::kCouponNodes> x{},v{},omega{};
    std::array<double,4*ref::kCouponNodes> q{};
    explicit InitialState(const ref::ElasticCouponConfiguration&);
};
// Provides exception-safe discard for every failed/temporary transaction.
// Discard after a successful commit is a harmless idle-state operation.
struct TrialScope {
    fea::FENodalState& state;
    ~TrialScope() { state.Discard(); }
};
bool SameStamp(const fea::NodalStamp&,const fea::NodalStamp&);
bool Matches(const shell::ShellBatchDiagnostics&,const contact::Q4PlanarContactDiagnostics&,
             std::uint64_t owner,std::uint64_t epoch,std::uint64_t attempt,bool candidate,std::uint64_t wall_binding,
             contact::Q4PlanarIntegrationBackend,std::uint64_t qualification);
bool MatchesPrepared(const fea::NodalPreparedView&,const fea::NodalAssemblyView&,const fea::NodalStamp&);
bool MatchesAcceptedResults(const shell::ShellBatchDiagnostics&,const contact::Q4PlanarContactDiagnostics&,
                            const fea::NodalStamp&,std::uint64_t wall_binding,contact::Q4PlanarIntegrationBackend,std::uint64_t qualification);
bool MatchesContactParents(const std::array<contact::Q4PlanarParentResult,ref::kCouponElements>&,
                          const std::array<contact::SurfaceQ4,ref::kCouponElements>&,
                          const contact::Q4PlanarContactDiagnostics&,contact::Q4PlanarIntegrationBackend);
cudaError_t ReadAuditConfiguration(const fea::NodalPreparedView&,ref::ElasticCouponConfiguration&);
bool AccumulateInterval(const shell::ShellBatchDiagnostics&,const contact::Q4PlanarContactDiagnostics& base,
                        const contact::Q4PlanarContactDiagnostics& endpoint,GuidedPlateMetrics& next);
} // namespace crash::case_data::guided_detail
