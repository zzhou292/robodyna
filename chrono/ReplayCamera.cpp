#include "ReplayVisuals.h"
#include <cmath>
namespace crash::visual {
bool MakeBoundsCamera(const std::array<double,3>& low,const std::array<double,3>& high,
        std::array<double,3> direction,double distance,ReplayVertical vertical,ReplayView view,ReplayCamera& camera) {
    ReplayCamera next;
    double extent[3];
    for (int i=0;i<3;++i) {
        const double lo=low[i],hi=high[i];
        if(!std::isfinite(lo) || !std::isfinite(hi) || hi<lo) return false;
        next.target[i]=lo*.5+hi*.5;
        extent[i]=hi-lo;
        if(!std::isfinite(extent[i])) return false;
    }
    const double diagonal=std::hypot(extent[0],extent[1],extent[2]);
    if(!(diagonal>0) || !std::isfinite(diagonal) || !(distance>0)) return false;
    if(view!=ReplayView::IncidentSide && view!=ReplayView::WallSide) return false;
    if(vertical!=ReplayVertical::Y && vertical!=ReplayVertical::Z) return false;
    next.vertical=vertical;
    next.view=view;
    if(view==ReplayView::WallSide) direction[0]=-direction[0];
    for(int i=0;i<3;++i) {
        next.position[i]=next.target[i]+distance*diagonal*direction[i];
        if(!std::isfinite(next.position[i])) return false;
    }
    camera=next;
    return true;
}
} // namespace crash::visual
