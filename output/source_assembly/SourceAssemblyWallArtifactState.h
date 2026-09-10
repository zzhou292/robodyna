#pragma once
#include "SourceAssemblyWallArtifacts.h"
#include "SourceAssemblyWallFields.h"
#include "SourceAssemblyWallSequence.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <fstream>

namespace crash::output::assembly {
struct SourceAssemblyWallArtifacts::Impl {
    Impl(const std::string& path,dynamics::SourceAssemblyWallCase& c,const WallArchiveRequest& r,const WallArchivePlan& p)
        :directory(path),inventory(directory,r.limits.total_bytes-WallManifestBytes),run(&c),request(r),plan(p) {}
    std::filesystem::path directory;ArtifactInventory inventory;const dynamics::SourceAssemblyWallCase* run;
    WallArchiveRequest request;WallArchivePlan plan;SourceAssemblyAcceptedOutput output;
    std::unique_ptr<CsvLedgerWriter> intervals;std::ofstream frames;
    tl::fea::NodalStamp initial;WallArchiveSequence sequence;
    bool failed=false,finished=false;
    void Check(const dynamics::SourceAssemblyWallCase& c) const {
        wall_fields::CheckCase(c);const auto s=c.owner()->accepted();
        Require(&c==run&&!failed&&!finished&&s.owner_id==initial.owner_id&&s.node_count==initial.node_count&&
            s.fixed_dt==initial.fixed_dt&&s.has_rotations==initial.has_rotations&&s.temporal_scheme==initial.temporal_scheme&&
            tl::fea::SameRigidGroupInfo(s.rigid_groups,initial.rigid_groups),"Assembly archive run is closed or changed owner/source");
    }
    void Abort() noexcept {failed=true;if(intervals)intervals->Abort();frames.close();}
};
} // namespace crash::output::assembly
