#pragma once
#include "AcceptedReplaySourceAssembly.h"
namespace crash::output::replay_detail {
inline constexpr const char* AssemblyConnectorKind="openradioss_type25_linear_finite_offset_v1";
inline constexpr const char* AssemblyConnectorPolicy="openradioss_tonne_millimetre_second_direct_import";
inline constexpr const char* AssemblyConnectorScope="Original seven-part Yaris component, native TYPE25 spotweld and nodal rigid groups active, external connections explicitly released";
inline constexpr const char* AssemblyConnectorWorkScope="Native signed TYPE25 channel work is separate from native_internal_work_J, which retains shell and stabilization work; sampled connector increments describe only their final accepted interval";
using ConnectorVector=chrono::ChVector3<long double>;
inline ConnectorVector ConnectorVectorValue(const Value& v) {
    AssemblyNumbers(v,3);return {v[0].GetDouble(),v[1].GetDouble(),v[2].GetDouble()};
}
inline ConnectorVector ConnectorColumn(const Value& matrix,unsigned column) {
    return {matrix[column].GetDouble(),matrix[column+3].GetDouble(),matrix[column+6].GetDouble()};
}
inline void ConnectorUnit(ConnectorVector v) {
    Require(std::abs(v.Dot(v)-1)<=1e-12L,"Connector frame direction is not unit");
}
inline void ConnectorAxes(const Value& v) {
    AssemblyNumbers(v,9);const auto x=ConnectorColumn(v,0),y=ConnectorColumn(v,1),z=ConnectorColumn(v,2);
    ConnectorUnit(x);ConnectorUnit(y);ConnectorUnit(z);
    Require(std::abs(x.Dot(y))<=1e-12L&&std::abs(x.Dot(z))<=1e-12L&&std::abs(y.Dot(z))<=1e-12L&&
        std::abs(x.Cross(y).Dot(z)-1)<=1e-12L,"Connector axes are not a proper orthonormal frame");
}
inline void ConnectorDirection(ConnectorVector actual,ConnectorVector expected) {
    Require((actual-expected).Length()<=1e-12L,"Connector source/geometric direction differs");
}
inline ConnectorVector ConnectorSourcePosition(const source::Node& n) {return {n.position_m.x,n.position_m.y,n.position_m.z};}
inline ConnectorVector ConnectorNodalVector(const Value& array,std::size_t n) {
    return {array[3*n].GetDouble(),array[3*n+1].GetDouble(),array[3*n+2].GetDouble()};
}
void CheckAssemblyConnectorProperty(const Value&,std::uint64_t,const source::Data&);
void CheckAssemblyConnectorElements(const Bundle&,const Entry&,const Value&,const Value&);
}
