#pragma once
#include "SourceAssemblyFlightFixture.h"
#include <type_traits>

namespace crash::qualification::source_assembly::packets {
// The shared qualification transaction is unchanged; only section readback
// selects the public typed API for explicitly heterogeneous source catalogs.
template<class Batch,class Section,class Diagnostics>
auto ReadAcceptedSections(Batch& batch,const fe::NodalStamp& stamp,std::vector<Section>& out,Diagnostics* diagnostics) {
    if constexpr(std::is_same_v<Section,fe::ShellBatchSectionState>)
        return batch.CopyAcceptedSectionHistory(stamp,out.data(),out.size(),diagnostics);
    else return batch.CopyAcceptedLayeredSectionHistory(stamp,out.data(),out.size(),diagnostics);
}
template<class Batch,class Section,class Diagnostics>
auto ReadPreparedSections(Batch& batch,const Diagnostics& diagnostics,std::vector<Section>& out) {
    if constexpr(std::is_same_v<Section,fe::ShellBatchSectionState>)
        return batch.CopyPreparedSectionHistory(diagnostics,out.data(),out.size());
    else return batch.CopyPreparedLayeredSectionHistory(diagnostics,out.data(),out.size());
}
template<class Packets> bool CapturePackets(Rig& r,Fields& state,Packets& shells) {
    EXPECT_EQ(r.owner.CopyAccepted(state.buffer(),&state.stamp).status,fe::NodalStatus::Ok);
    EXPECT_EQ(r.qeph.CopyAcceptedResults(state.stamp,shells.quad.data(),r.quads(),&shells.diagnostics.qeph).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.CopyAcceptedResults(state.stamp,shells.triangle.data(),r.triangles(),&shells.diagnostics.t3).status,t::BatchStatus::Success);
    EXPECT_EQ(ReadAcceptedSections(r.qeph,state.stamp,shells.qsection,&shells.diagnostics.qeph).status,q::BatchStatus::Success);
    EXPECT_EQ(ReadAcceptedSections(r.t3,state.stamp,shells.tsection,&shells.diagnostics.t3).status,t::BatchStatus::Success);
    EXPECT_EQ(r.publication.CopyAcceptedDiagnostics(state.stamp,&shells.diagnostics).status,fe::ShellPublicationStatus::Success);
    return !::testing::Test::HasFailure();
}

template<class Packets> bool EvaluatePackets(Rig& r,const Prepared& p,Packets& out) {
    const auto qr=r.groups_attached?r.qeph.EvaluateCandidate(r.owner,p.token,p.view,&out.diagnostics.qeph):
        r.qeph.EvaluateCandidate(p.view,&out.diagnostics.qeph);
    const auto tr=r.groups_attached?r.t3.EvaluateCandidate(r.owner,p.token,p.view,&out.diagnostics.t3):
        r.t3.EvaluateCandidate(p.view,&out.diagnostics.t3);
    EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message<<" family_index="<<qr.element;
    EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message<<" family_index="<<tr.element;
    if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success)return false;
    EXPECT_EQ(r.qeph.CopyPreparedResults(out.diagnostics.qeph,out.quad.data(),r.quads()).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.CopyPreparedResults(out.diagnostics.t3,out.triangle.data(),r.triangles()).status,t::BatchStatus::Success);
    EXPECT_EQ(ReadPreparedSections(r.qeph,out.diagnostics.qeph,out.qsection).status,q::BatchStatus::Success);
    EXPECT_EQ(ReadPreparedSections(r.t3,out.diagnostics.t3,out.tsection).status,t::BatchStatus::Success);
    fe::ShellBatchDiagnostics common;
    const auto report=r.publication.Prepare(r.owner,p.token,out.diagnostics.qeph,out.diagnostics.t3,&common);
    EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;
    if(report.status==fe::ShellPublicationStatus::Success)out.diagnostics=common;
    return !::testing::Test::HasFailure();
}
template<class Packets> bool PublishPackets(Rig& r,const Prepared& p,const Packets& out) {
    const auto& d=out.diagnostics.qeph;
    const auto report=r.publication.Commit(r.owner,p.token,out.diagnostics,{d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true});
    EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;return report.status==fe::ShellPublicationStatus::Success;
}
} // namespace crash::qualification::source_assembly::packets
