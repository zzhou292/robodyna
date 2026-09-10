#pragma once
#include "output/source_assembly/SourceAssemblySectionFields.h"
#include "modelio/source_assembly/tests/AssemblyTestSupport.h"
#include "chrono/AcceptedSurfaceMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <cstring>

namespace crash::output::assembly::test {
inline SourceAssemblySurface Surface() {
    return SourceAssemblySurface::Prepare(source::test::Load(),{17,23,31},5,0x5941524953);
}
// Distinct synthetic values exercise mapping/serialization only, never a crash
// claim or mechanical input. Each layer and family parent has a unique marker.
struct Fields {
    explicit Fields(const SourceAssemblySurface& surface)
        :q(surface.source().data().qeph_count),t(surface.source().data().t3_count),qt(q.size()),tt(t.size()) {
        for(const auto& p:surface.parents()) {
            auto& s=p.family==source::ShellFamily::Qeph?q[p.family_index]:t[p.family_index];
            auto& thickness=p.family==source::ShellFamily::Qeph?qt[p.family_index]:tt[p.family_index];
            const double marker=1+double(p.source_index);
            s.cumulative_plastic_work_J=marker+.125;
            s.diagnostics={marker+.25,marker/1024,marker/2048,.25,.5,marker*10,marker*20};
            thickness=.0001+marker/1048576;
            for(unsigned layer=0;layer<3;++layer) {
                auto& point=s.history.point[layer];
                for(unsigned component=0;component<5;++component)point.stress[component]=-(100*marker+10*layer+component+.5);
                point.plastic_strain=(marker+layer)/4096;point.filtered_rate_per_s=marker+layer+.25;
            }
        }
    }
    std::vector<tl::fea::ShellBatchSectionState> q,t;
    std::vector<double> qt,tt;
    SectionView Q() const{return {q.data(),qt.data(),q.size()};}
    SectionView T() const{return {t.data(),tt.data(),t.size()};}
};
inline std::vector<double> Positions(const SourceAssemblySurface& surface) {
    std::vector<double> x; x.reserve(3*surface.binding().tl_node_count);
    for(const auto& n:surface.source().data().nodes)for(double value:{n.position_m.x,n.position_m.y,n.position_m.z})x.push_back(value);
    return x;
}
} // namespace crash::output::assembly::test
