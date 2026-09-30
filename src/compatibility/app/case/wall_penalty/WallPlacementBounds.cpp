#include "WallPlacementBounds.h"
#include <cmath>
namespace crash::cases::wall_penalty {
namespace sc=tlfea::contact;
namespace qb=sc::q4_bounds;
bool ExpandProjectedMotion(const std::array<sc::Vec3,2>& bounds,double margin,sc::PlanarWallBox* output) noexcept {
    if(!output||!sc::IsFinite(bounds[0])||!sc::IsFinite(bounds[1])||!std::isfinite(margin)||margin<=0||
       bounds[0].x>bounds[1].x||bounds[0].y>bounds[1].y||bounds[0].z>bounds[1].z)return false;
    sc::PlanarWallBox next{bounds[0],bounds[1]};
    if(!qb::AddScalar(next.minimum.y,-margin,false,&next.minimum.y)||
       !qb::AddScalar(next.minimum.z,-margin,false,&next.minimum.z)||
       !qb::AddScalar(next.maximum.y,margin,true,&next.maximum.y)||
       !qb::AddScalar(next.maximum.z,margin,true,&next.maximum.z))return false;
    *output=next;return true;
}
bool EncloseLeadingGap(double wall,double source,sc::Q4IntegralInterval* output) noexcept {
    if(!output)return false;
    sc::Q4IntegralInterval next;
    if(!qb::Difference(wall,source,&next)||next.lower<=0)return false;
    *output=next;return true;
}
} // namespace crash::cases::wall_penalty
