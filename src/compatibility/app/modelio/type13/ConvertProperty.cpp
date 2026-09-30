// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted OpenRadioss converter; revision/hash recorded in SOURCE_CONTRACT.md.
#include "ConvertProperty.h"
#include <cmath>

namespace crash::modelio::type13 {
native::PropertyInput ConvertedProperty::input() const noexcept {
    native::PropertyInput out;out.units={1000,.001,1};out.controls={1,1,0,0,0};
    out.mass_per_length=mass_per_length;out.inertia_per_length=inertia_per_length;
    for(unsigned c=0;c<6;++c)out.channels[c]=channels[c];
    for(unsigned c=0;c<4;++c)out.curves[c]={curves[c].points,5};
    return out;
}
native::Status ConvertProperty(const SourceProperty& in,ConvertedProperty& output) {
    const auto positive=[](double x){return std::isfinite(x)&&x>0;};
    if(!positive(in.density)||!positive(in.young)||!positive(in.yield)||!positive(in.tangent)||
       in.tangent>=in.young||!std::isfinite(in.poisson)||in.poisson<0||in.poisson>=.5||
       !positive(in.outer1)||!positive(in.outer2)||in.inner1!=0||in.inner2!=0||
       !positive(in.failure_deformation))return native::Status::InvalidInput;
    ConvertedProperty next;
    const double meanTS=(in.outer1+in.outer2)/2.0;
    const double meanTT=(in.inner1+in.inner2)/2.0;
    const double piVal=3.14159265359;
    const double diam=meanTS-meanTT;
    const double area=piVal*(std::pow(meanTS,2)-std::pow(meanTT,2))/4.0;
    const double Ixx=piVal*(std::pow(meanTS,4)-std::pow(meanTT,4))/32.0;
    const double torsionMoment=(std::pow(meanTS,3)-std::pow(meanTT,3))/(2*piVal);
    const double bendingMoment=(std::pow(meanTS,3)-std::pow(meanTT,3))*piVal/16.0;
    const double radShear=7.0/8.0;
    const double radG=in.young/(2*(1.0+in.poisson));
    next.mass_per_length=in.density*area;next.inertia_per_length=in.density*Ixx;
    const double stiffness[6]={area*in.young,area*in.young/(2*(1+in.poisson)),
        area*in.young/(2*(1+in.poisson)),in.young*(std::pow(meanTS,4)-std::pow(meanTT,4))/(9.0*piVal),
        in.young*(std::pow(meanTS,4)-std::pow(meanTT,4))*3.0/64.0,
        in.young*(std::pow(meanTS,4)-std::pow(meanTT,4))*3.0/64.0};
    const unsigned mapping[6]={0,1,1,2,3,3};
    for(unsigned c=0;c<6;++c)next.channels[c]={mapping[c],stiffness[c],1,1,0,
        -in.failure_deformation,in.failure_deformation,1,2,1};
    double x=in.yield/in.young;
    next.curves[0]={{{-x-1,-1*(in.yield+in.tangent)*area},{-x,-1*in.yield*area},
                    {0,0},{x,in.yield*area},{x+1,(in.yield+in.tangent)*area}}};
    x=(in.yield*.5)/(radG*radShear);
    double y=(in.yield*area*.5)/radShear;
    next.curves[1]={{{-x-1,-1*y-in.tangent*area*5.0/16.0},{-x,-1*y},
                    {0,0},{x,y},{x+1,y+in.tangent*area*5.0/16.0}}};
    x=in.yield/in.young;y=in.yield*torsionMoment;
    next.curves[2]={{{(-x-1)*9.0/(2.0*diam),-1*y-in.tangent*torsionMoment},
                    {-x*9.0/(2*diam),-1*y},{0,0},{(x*9.0)/(2*diam),y},
                    {(x+1)*9.0/(2.0*diam),y+in.tangent*torsionMoment}}};
    x=(in.yield*4.0*piVal)/(in.young*3.0*diam);y=in.yield*bendingMoment;
    next.curves[3]={{{-1*x-(piVal/diam),-1*y-in.tangent*bendingMoment},{-1*x,-1*y},
                    {0,0},{x,y},{x+(piVal/diam),y+in.tangent*bendingMoment}}};
    // Reuse the TL admission leaf for all resulting curve/coefficient checks.
    native::Property checked;
    const auto status=native::InitializeProperty(next.input(),checked);
    if(status!=native::Status::Success)return status;
    output=next;return native::Status::Success;
}
}
