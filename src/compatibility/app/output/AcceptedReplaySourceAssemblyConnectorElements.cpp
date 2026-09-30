#include "AcceptedReplaySourceAssemblyConnectorValues.h"
namespace crash::output::replay_detail {
namespace {
ConnectorVector World(const Value& axes,ConnectorVector local) {
    return ConnectorColumn(axes,0)*local.x()+ConnectorColumn(axes,1)*local.y()+ConnectorColumn(axes,2)*local.z();
}
void SameVector(ConnectorVector actual,ConnectorVector expected,long double magnitude) {
    for(unsigned j=0;j<3;++j)AssemblyReduction(actual[j],expected[j],magnitude,12);
}
void CheckWrenches(const Value& row,ConnectorVector chord,const Value& axes,const Value& history) {
    const auto& w=WallArray(row,"endpoint_wrenches",2);AssemblyNumbers(w[0],6);AssemblyNumbers(w[1],6);
    ConnectorVector f[2],m[2];for(unsigned e=0;e<2;++e) {
        f[e]={w[e][0].GetDouble(),w[e][1].GetDouble(),w[e][2].GetDouble()};
        m[e]={w[e][3].GetDouble(),w[e][4].GetDouble(),w[e][5].GetDouble()};
    }
    const auto local_force=ConnectorVectorValue(Member(history,"local_force_N"));
    const auto local_couple=ConnectorVectorValue(Member(history,"local_couple_N_m"));
    const auto force=World(axes,local_force),couple=World(axes,local_couple);
    SameVector(f[0],force,f[0].Length()+local_force.Length());
    SameVector(f[1],-f[0],f[0].Length()+f[1].Length());
    // Independent force pair, torque balance and local-couple decomposition.
    // These recorded-value identities do not evaluate a spring constitutive law.
    const auto arm=chord.Cross(f[0]);
    SameVector(m[0]+m[1],arm,m[0].Length()+m[1].Length()+chord.Length()*f[0].Length());
    SameVector(m[0]-m[1],couple*2,m[0].Length()+m[1].Length()+2*local_couple.Length());
}
}
void CheckAssemblyConnectorElements(const Bundle& b,const Entry& e,const Value& record,const Value& nodal) {
    const auto& a=*b.assembly;const auto& rows=WallArray(record,"elements",a.connectors.size());
    const auto& x=WallNumbers(nodal,"position_xyz_m",3*b.info.node_count);
    const auto& velocity=WallNumbers(nodal,"velocity_xyz_m_per_s",3*b.info.node_count);
    std::array<long double,4> work{},magnitude{};std::size_t active=0;double minimum=std::numeric_limits<double>::max();
    for(std::size_t i=0;i<rows.Size();++i) {
        const auto& row=rows[i];const auto& expected=a.connectors[i];
        Require(row.IsObject()&&row.MemberCount()==8&&Unsigned(row,"source_element_id")==expected.element&&Unsigned(row,"generated_property_id")==expected.property,
            "Accepted connector element/property order differs from source");
        const auto& h=Member(row,"history");const auto& frame=Member(row,"frame");
        Require(h.IsObject()&&h.MemberCount()==8&&frame.IsObject()&&frame.MemberCount()==4,"Incomplete or unknown connector history/frame fields");
        const auto& axes=Member(frame,"axes_row_major");const auto& middle=Member(frame,"midpoint_axes_row_major");
        ConnectorAxes(axes);ConnectorAxes(middle);
        const auto chord=ConnectorNodalVector(x,expected.nodes[1])-ConnectorNodalVector(x,expected.nodes[0]);
        const auto dv=ConnectorNodalVector(velocity,expected.nodes[1])-ConnectorNodalVector(velocity,expected.nodes[0]);
        const auto midpoint=e.epoch?chord-dv*(.5L*b.fixed_dt):chord;
        const double length=Real(frame,"length_m"),mid_length=Real(frame,"midpoint_length_m");
        Require(length>0&&mid_length>0&&chord.Length()>0&&midpoint.Length()>0,"Connector frame has degenerate length");
        AssemblyNear(length,chord.Length());AssemblyNear(mid_length,midpoint.Length());
        ConnectorDirection(ConnectorColumn(axes,0),chord/chord.Length());ConnectorDirection(ConnectorColumn(middle,0),midpoint/midpoint.Length());
        const auto transverse=ConnectorVectorValue(Member(h,"transverse_axis"));ConnectorUnit(transverse);
        ConnectorDirection(transverse,ConnectorColumn(axes,1));
        const auto& criterion=Member(h,"active");Require(criterion.IsBool(),"Connector active state is not Boolean");
        const bool on=criterion.GetBool();active+=on;const double failure=Real(h,"failure_criterion");
        Require(failure>=0&&failure<=1&&(on||failure==1),"Connector native coupled-failure state is invalid");
        for(const char* key:{"displacement_m","rotation_rad","local_force_N","local_couple_N_m"})WallNumbers(h,key,3);
        const auto& channels=WallNumbers(h,"internal_work_J",4);
        for(unsigned j=0;j<4;++j){work[j]+=channels[j].GetDouble();magnitude[j]+=std::abs(channels[j].GetDouble());}
        const auto dt=Real(row,"critical_dt_s");Require(dt>0&&b.fixed_dt<=dt*a.connector_dt_fraction,
            "Accepted connector violates its declared native timestep guard");minimum=std::min(minimum,dt);
        Require(Real(row,"translation_stiffness_N_per_m")>0&&Real(row,"rotation_stiffness_N_m_per_rad")>0,
            "Connector stiffness diagnostic is not positive");
        CheckWrenches(row,chord,axes,h);
        if(!e.epoch) {
            Require(on&&failure==0,"Initial connector contains failure history");AssemblyEqual(length,expected.length);AssemblyEqual(mid_length,expected.length);
            ConnectorDirection(transverse,{expected.transverse[0],expected.transverse[1],expected.transverse[2]});
            for(const char* key:{"displacement_m","rotation_rad","local_force_N","local_couple_N_m","internal_work_J"})
                for(const auto& value:Member(h,key).GetArray())Require(value.GetDouble()==0,"Initial connector contains interval history");
        }
    }
    const auto& d=Member(record,"diagnostics");const auto& total=WallNumbers(d,"internal_work_J",4);
    for(unsigned j=0;j<4;++j)AssemblyReduction(total[j].GetDouble(),work[j],magnitude[j],rows.Size());
    Require(Unsigned(d,"active_count")==active,"Connector active-count summary differs from complete history");
    AssemblyEqual(Real(d,"minimum_native_dt_s"),minimum);
}
}
