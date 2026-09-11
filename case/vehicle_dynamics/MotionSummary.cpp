#include "MotionSummary.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::vehicle_dynamics {
tl::fea::NodalSnapshotBuffer Fields::buffer() noexcept {
    return {position.data(),velocity.data(),position.size()/3,orientation.data(),spin.data()};
}
MotionSummary ObserveUniformMotion(const tl::fea::NodalCoefficientLedger& source,
    const Fields& fields,double speed,double time) {
    const auto n=source.nodes().size();
    output::Require(n && fields.position.size()==3*n && fields.velocity.size()==3*n &&
        fields.orientation.size()==4*n && fields.spin.size()==3*n &&
        std::isfinite(speed) && std::isfinite(time) && time>=0,"Uniform-motion observation has invalid extent or time");
    MotionSummary result;result.nodes=n;
    const auto maximum=[](double actual,double expected,double& value) {
        output::Require(std::isfinite(actual) && std::isfinite(expected),"Nonfinite complete-owner motion field");
        const auto difference=std::abs(actual-expected);
        output::Require(std::isfinite(difference),"Motion difference overflow");
        value=std::max(value,difference);
    };
    for(std::size_t i=0;i<n;++i) {
        const auto x=source.domain()->nodes()[i].position;
        const double expected[]{x.x+speed*time,x.y,x.z};
        for(unsigned axis=0;axis<3;++axis) {
            maximum(fields.position[3*i+axis],expected[axis],result.maximum_position_error);
            maximum(fields.velocity[3*i+axis],axis==0?speed:0,result.maximum_velocity_error);
            maximum(fields.spin[3*i+axis],0,result.maximum_spin);
        }
        for(unsigned axis=0;axis<4;++axis)
            maximum(fields.orientation[4*i+axis],axis==0?1:0,result.maximum_orientation_error);
    }
    return result;
}
} // namespace crash::cases::vehicle_dynamics
