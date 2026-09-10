#include "accepted_replay_source_assembly_connector_fixture.h"
#include <cstdlib>
namespace crash::output::connector_test {
using namespace replay_detail;
namespace {
Value Ids(Document& d,std::initializer_list<std::uint64_t> values) {
    Value row(rapidjson::kArrayType);for(auto v:values)row.PushBack(v,d.GetAllocator());return row;
}
Value Numbers(Document& d,std::initializer_list<double> values) {
    Value row(rapidjson::kArrayType);for(auto v:values)row.PushBack(v,d.GetAllocator());return row;
}
Value Vector(Document& d,ConnectorVector v) {return Numbers(d,{double(v.x()),double(v.y()),double(v.z())});}
Value Matrix(Document& d,ConnectorVector x,ConnectorVector y) {
    const auto z=x.Cross(y);return Numbers(d,{double(x.x()),double(y.x()),double(z.x()),double(x.y()),double(y.y()),double(z.y()),double(x.z()),double(y.z()),double(z.z())});
}
void Add(Document& d,const char* key,Document& value) {Value copy;copy.CopyFrom(value,d.GetAllocator());Put(d,key,std::move(copy));}
}
Fixture::Fixture() {
    const auto* inventory=std::getenv("ROBO_DYNA_SEVEN_PART_SOURCE_INVENTORY");
    Require(inventory&&*inventory,"Explicit authenticated seven-part source fixture required");
    bundle.assembly=std::make_shared<AssemblyReplayData>(source::SourceAssembly::Read(inventory,source::PinnedYarisSevenPartInventory()));
    auto& a=*bundle.assembly;const auto& s=a.source.data();a.instance=17;bundle.info.owner_id=23;
    bundle.info.node_count=s.nodes.size();bundle.source_configuration_id=31;bundle.qualification_id=37;bundle.fixed_dt=1./67108864;
    a.native_nodes.assign(s.nodes.size(),{1,2,1,1});a.grouped_node.assign(s.nodes.size(),false);
    for(const auto& g:s.nodal_rigid_groups)if(g.internal)for(auto n:g.selected_global_nodes)a.grouped_node[n]=true;
    const auto& source=s.internal_spotwelds.at(0);const std::uint64_t property=383348001108;
    const auto x0=ConnectorSourcePosition(s.nodes[source.nodes[0]]),x1=ConnectorSourcePosition(s.nodes[source.nodes[1]]),chord=x1-x0;
    const auto axis=chord/chord.Length();auto transverse=chord.Cross({0,1,0}).Cross(chord);transverse/=transverse.Length();
    configuration.SetObject();Document input;input.SetObject();Document c;c.SetObject();
    String(c,"kind",AssemblyConnectorKind);String(c,"policy",AssemblyConnectorPolicy);Integer(c,"source_instance_id",a.instance);
    Integer(c,"connection_count",1);Integer(c,"property_count",1);Put(c,"source_units_to_SI",Numbers(c,{1000,.001,1}));
    Document p;p.SetObject();Integer(p,"generated_property_id",property);Number(p,"mass_kg",.001e-3*1000.);
    Number(p,"isotropic_inertia_kg_m2",.01e-3*(1000.*.001*.001));
    Put(p,"stiffness",Numbers(p,{1e8,1e8,1000,1000}));Put(p,"damping",Numbers(p,{0,0,0,0}));
    Put(p,"failure_positive",Numbers(p,{1e30,1e30,1e30*.001,1e30*.001}));
    Put(p,"failure_negative",Numbers(p,{-1e30,-1e30,-(1e30*.001),-(1e30*.001)}));
    Put(p,"failure_weight",Numbers(p,{1,1,1,1}));Put(p,"failure_exponent",Numbers(p,{2,2,2,2}));
    Value properties(rapidjson::kArrayType),pr;pr.CopyFrom(p,c.GetAllocator());properties.PushBack(pr,c.GetAllocator());Put(c,"properties",std::move(properties));
    Document r;r.SetObject();Integer(r,"source_element_id",source.record.id);Integer(r,"generated_property_id",property);
    Put(r,"source_node_ids",Ids(r,{source.record.node_ids[0],source.record.node_ids[1]}));Put(r,"global_nodes",Ids(r,{source.nodes[0],source.nodes[1]}));
    Put(r,"source_card_lines",Ids(r,{source.record.cards[0].source_line,source.record.cards[1].source_line}));
    Put(r,"reference_positions_m",Numbers(r,{double(x0.x()),double(x0.y()),double(x0.z()),double(x1.x()),double(x1.y()),double(x1.z())}));
    Put(r,"transverse_axis",Vector(r,transverse));Number(r,"reference_length_m",double(chord.Length()));
    Value connections(rapidjson::kArrayType),connection;connection.CopyFrom(r,c.GetAllocator());connections.PushBack(connection,c.GetAllocator());Put(c,"connections",std::move(connections));
    String(c,"endpoint_columns","source_element_id,generated_property_id,local_endpoint,global_node,source_node_id,mass_kg,inertia_kg_m2,total_node_mass_kg,total_node_inertia_kg_m2");
    Value endpoints(rapidjson::kArrayType);
    for(unsigned e=0;e<2;++e){auto row=Ids(c,{source.record.id,property,e,source.nodes[e],source.record.node_ids[e]});
        for(double v:{.0005,5e-9,1+.0005,2+5e-9})row.PushBack(v,c.GetAllocator());endpoints.PushBack(row,c.GetAllocator());}
    Put(c,"endpoint_contributions",std::move(endpoints));Add(input,"connectors",c);Add(configuration,"input",input);
    String(configuration,"connector_work_scope",AssemblyConnectorWorkScope);Document limits;limits.SetObject();
    Integer(limits,"max_connections",1024);Integer(limits,"max_device_bytes",2*1024*1024);Integer(limits,"max_host_bytes",8*1024*1024);
    Add(configuration,"connector_storage_limits",limits);Document deformation;deformation.SetObject();Number(deformation,"maximum_native_dt_fraction",.5);
    Add(configuration,"deformation_limits",deformation);
    frame.SetObject();Document nodal;nodal.SetObject();Value positions(rapidjson::kArrayType),velocity(rapidjson::kArrayType);
    for(const auto& n:s.nodes)for(double v:{n.position_m.x,n.position_m.y,n.position_m.z}) {positions.PushBack(v,nodal.GetAllocator());velocity.PushBack(0.,nodal.GetAllocator());}
    Put(nodal,"position_xyz_m",std::move(positions));Put(nodal,"velocity_xyz_m_per_s",std::move(velocity));Add(frame,"nodal_fields",nodal);
    Document recorded;recorded.SetObject();String(recorded,"kind",AssemblyConnectorKind);Document d;d.SetObject();
    String(d,"phase","accepted");Integer(d,"source_instance_id",a.instance);Integer(d,"owner_id",23);Integer(d,"configuration_id",31);Integer(d,"qualification_id",37);
    for(const char* key:{"epoch","base_epoch","attempt","newly_failed_count"})Integer(d,key,0);
    for(const char* key:{"time_s","base_time_s","velocity_time_s","base_velocity_time_s","kick_dt_s","internal_kick_work_J","internal_drift_work_J"})Number(d,key,0);
    Boolean(d,"valid",true);Boolean(d,"has_completed_interval",false);Boolean(d,"accepted_force_assembled",false);
    Integer(d,"element_count",1);Integer(d,"active_count",1);Number(d,"minimum_native_dt_s",1e-6);
    Put(d,"internal_work_J",Numbers(d,{0,0,0,0}));Put(d,"internal_work_increment_J",Numbers(d,{0,0,0,0}));Add(recorded,"diagnostics",d);
    Document e;e.SetObject();Integer(e,"source_element_id",source.record.id);Integer(e,"generated_property_id",property);Document h;h.SetObject();
    Put(h,"transverse_axis",Vector(h,transverse));for(const char* key:{"displacement_m","rotation_rad","local_force_N","local_couple_N_m"})Put(h,key,Numbers(h,{0,0,0}));
    Put(h,"internal_work_J",Numbers(h,{0,0,0,0}));Number(h,"failure_criterion",0);Boolean(h,"active",true);Add(e,"history",h);
    Document geometry;geometry.SetObject();Put(geometry,"axes_row_major",Matrix(geometry,axis,transverse));Put(geometry,"midpoint_axes_row_major",Matrix(geometry,axis,transverse));
    Number(geometry,"length_m",double(chord.Length()));Number(geometry,"midpoint_length_m",double(chord.Length()));Add(e,"frame",geometry);
    Value wrenches(rapidjson::kArrayType);for(unsigned n=0;n<2;++n)wrenches.PushBack(Numbers(e,{0,0,0,0,0,0}),e.GetAllocator());Put(e,"endpoint_wrenches",std::move(wrenches));
    Number(e,"critical_dt_s",1e-6);Number(e,"translation_stiffness_N_per_m",1e8);Number(e,"rotation_stiffness_N_m_per_rad",1e3);
    Value elements(rapidjson::kArrayType),element;element.CopyFrom(e,recorded.GetAllocator());elements.PushBack(element,recorded.GetAllocator());Put(recorded,"elements",std::move(elements));Add(frame,"connectors",recorded);
    Entry entry;entry.owner=23;bundle.entries.push_back(entry);
}
void Fixture::Admit() {ReadAssemblyConnectors(bundle,configuration);}
}
