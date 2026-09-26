#include "EnvironmentFields.h"
#include <cmath>
namespace crash::output::physical_frames::detail {
void CheckFixedEnvironment(const FixedEnvironmentField& fixed,const double* x,const double* v,std::size_t nodes,
    const tl::fea::ShellBatchLayeredSection* history,const std::uint8_t* active,std::size_t qeph) {
    Require(x && v && nodes && nodes<=524288 && history && active && qeph && qeph<=524288 && fixed.qeph_index==qeph-1,
        "Fixed environment readback does not cover the actual family suffix");
    // Complete owner readback is validated even for physical nodes that are
    // not in the vehicle render subset. This adds work only at sampled frames.
    for(std::size_t i=0;i<3*nodes;++i)
        Require(std::isfinite(x[i]) && std::isfinite(v[i]),"Nonfinite complete accepted owner readback");
    Require(history[fixed.qeph_index].law()==tl::fea::ShellSectionLaw::GlobalLaw1Npt0 &&
        active[fixed.qeph_index]==1 && !history[fixed.qeph_index].elastic() &&
        !history[fixed.qeph_index].plastic() && !history[fixed.qeph_index].one_point(),
        "Declared fixed environment has unavailable law/activity/point semantics");
    for(unsigned k=0;k<4;++k) {
        Require(fixed.nodes[k]<nodes,"Declared environment node is outside complete readback");
        for(unsigned j=0;j<k;++j)Require(fixed.nodes[j]!=fixed.nodes[k],"Declared environment repeats a source node");
        const auto& p=fixed.reference[k];const double reference[]{p.x,p.y,p.z};
        for(unsigned axis=0;axis<3;++axis)
            Require(Bits(x[3*fixed.nodes[k]+axis])==Bits(reference[axis]) &&
                Bits(v[3*fixed.nodes[k]+axis])==Bits(0.),"Accepted environment left its genuine fixed source coordinates");
    }
}
}
