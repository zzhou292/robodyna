#include "RecordFields.h"
#include <cmath>
namespace crash::output::physical_frames::detail {
void StagePositions(const std::vector<std::uint32_t>& nodes,const double* xyz,std::size_t count,
    records::FrameRecord& out) {
    Require(xyz && count && out.position_xyz.size()==3*nodes.size(),"Physical position capture extent differs");
    for(std::size_t i=0;i<nodes.size();++i) {
        Require(nodes[i]<count,"Render node exceeds actual physical owner");
        for(unsigned j=0;j<3;++j) {
            const double x=xyz[3*nodes[i]+j];
            Require(std::isfinite(x),"Nonfinite accepted physical position");
            out.position_xyz[3*i+j]=x;
        }
    }
}
namespace {
void Extents(const records::Context& c,const std::vector<ParentField>& mapping,
    const records::FrameRecord& out,const std::vector<std::uint8_t>& active) {
    Require(mapping.size()==c.parents().size() && active.size()==mapping.size() &&
        out.plastic_points.size()==c.points(),"Accepted point/activity capture extent differs");
}
void Point(double value,double& output) {
    Require(std::isfinite(value) && value>=0,"Invalid accepted native accumulated plastic strain");
    output=value;
}
}
void StageLayered(const records::Context& c,const std::vector<ParentField>& mapping,std::uint32_t family,
    const tl::fea::ShellBatchLayeredSection* history,const std::uint8_t* active,std::size_t count,
    records::FrameRecord& out,std::vector<std::uint8_t>& flags) {
    Extents(c,mapping,out,flags);
    Require((family==QephFamily || family==T3Family) && history && active,"Missing native layered capture");
    std::size_t visited=0;
    for(std::size_t i=0;i<mapping.size();++i) {
        const auto& m=mapping[i];
        if(m.family!=family)continue;
        Require(m.index<count && active[m.index]<=1 && c.parents()[i].native_family==family &&
            history[m.index].law()==m.law && c.parents()[i].plastic==Plasticity(m.law),
            "Accepted layered source/availability/index differs");
        const auto begin=c.point_offsets()[i],end=c.point_offsets()[i+1];
        const auto& row=history[m.index];
        if(const auto* plastic=row.plastic()) {
            Require(end-begin==3 && c.parents()[i].native_points==3,"NIP3 native point layout differs");
            for(unsigned k=0;k<3;++k)Point(plastic->history.point[k].plastic_strain,out.plastic_points[begin+k]);
        } else if(const auto* point=row.one_point()) {
            Require(end-begin==1 && c.parents()[i].native_points==1 && family==T3Family,
                    "True T3 one-point layout differs");
            Point(point->point.saved.plastic_strain,out.plastic_points[begin]);
        } else {
            Require(begin==end && ((row.elastic() && c.parents()[i].native_points==3) ||
                (m.law==tl::fea::ShellSectionLaw::RigidSkin && c.parents()[i].native_points==0)),
                "Nonplastic role exposes fabricated native plastic points");
        }
        flags[i]=active[m.index];
        ++visited;
    }
    Require(visited==count,"Incomplete accepted layered family capture");
}
void StageQbat(const records::Context& c,const std::vector<ParentField>& mapping,
    const tl::fea::qbat::BatchResult* history,const std::uint8_t* active,std::size_t count,
    records::FrameRecord& out,std::vector<std::uint8_t>& flags) {
    Extents(c,mapping,out,flags);
    Require(history && active,"Missing native four-point capture");
    std::size_t visited=0;
    for(std::size_t i=0;i<mapping.size();++i) {
        const auto& m=mapping[i];
        if(m.family!=QbatFamily)continue;
        const auto begin=c.point_offsets()[i],end=c.point_offsets()[i+1];
        Require(m.index<count && active[m.index]<=1 && c.parents()[i].native_family==QbatFamily &&
            m.law==tl::fea::ShellSectionLaw::Law44QbatFourInPlane && c.parents()[i].native_points==4 &&
            c.parents()[i].plastic==records::PlasticField::NativeEquivalentPlasticStrain && end-begin==4,
            "Accepted QBAT source/point layout differs");
        for(unsigned k=0;k<4;++k)Point(history[m.index].history.point[k].material.plastic_strain,out.plastic_points[begin+k]);
        flags[i]=active[m.index];
        ++visited;
    }
    Require(visited==count,"Incomplete accepted QBAT family capture");
}
} // namespace crash::output::physical_frames::detail
