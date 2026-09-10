#include "SourceAssemblyConnectorFields.h"
#include "WallFieldValues.h"

namespace crash::output::assembly::wall_fields {
namespace {
Document Diagnostics(const tl::fea::type25::BatchDiagnostics& a) {
    Document d;d.SetObject();String(d,"phase","accepted");
    Integer(d,"source_instance_id",a.source_instance_id);Integer(d,"owner_id",a.owner_id);
    Integer(d,"configuration_id",a.configuration_id);Integer(d,"qualification_id",a.qualification_id);
    Integer(d,"epoch",a.epoch);Integer(d,"base_epoch",a.base_epoch);Integer(d,"attempt",a.attempt);
    Number(d,"time_s",a.time);Number(d,"base_time_s",a.base_time);Number(d,"velocity_time_s",a.velocity_time);
    Number(d,"base_velocity_time_s",a.base_velocity_time);Number(d,"kick_dt_s",a.kick_dt);
    Boolean(d,"valid",a.valid);Boolean(d,"has_completed_interval",a.has_completed_interval);
    Boolean(d,"accepted_force_assembled",a.accepted_force_assembled);Integer(d,"element_count",a.element_count);
    Integer(d,"active_count",a.active_count);Integer(d,"newly_failed_count",a.newly_failed_count);
    FiniteArray(d,"internal_work_J",a.internal_work_J,4);FiniteArray(d,"internal_work_increment_J",a.internal_work_increment_J,4);
    Number(d,"internal_kick_work_J",a.internal_kick_work);Number(d,"internal_drift_work_J",a.internal_drift_work);
    Number(d,"minimum_native_dt_s",a.minimum_native_dt);return d;
}
Document Element(const tl::fea::type25::Evaluation& v,std::uint64_t source,std::uint64_t property) {
    Document d;d.SetObject();Integer(d,"source_element_id",source);Integer(d,"generated_property_id",property);
    const auto& a=v.history;Document h;h.SetObject();Put(h,"transverse_axis",Vector(h,a.transverse_axis));
    Put(h,"displacement_m",Vector(h,a.displacement_m));Put(h,"rotation_rad",Vector(h,a.rotation_rad));
    Put(h,"local_force_N",Vector(h,a.local_force_N));Put(h,"local_couple_N_m",Vector(h,a.local_couple_Nm));
    FiniteArray(h,"internal_work_J",a.internal_work_J,4);Number(h,"failure_criterion",a.failure_criterion);Boolean(h,"active",a.active);
    Child(d,"history",h);Document frame;frame.SetObject();Put(frame,"axes_row_major",Matrix(frame,v.frame.axes));
    Put(frame,"midpoint_axes_row_major",Matrix(frame,v.frame.midpoint_axes));Number(frame,"length_m",v.frame.length_m);
    Number(frame,"midpoint_length_m",v.frame.midpoint_length_m);Child(d,"frame",frame);
    Value endpoints(rapidjson::kArrayType);for(const auto& w:v.endpoints)
        endpoints.PushBack(Values(d,{w.force_N.x,w.force_N.y,w.force_N.z,w.couple_Nm.x,w.couple_Nm.y,w.couple_Nm.z}),d.GetAllocator());
    Put(d,"endpoint_wrenches",std::move(endpoints));Number(d,"critical_dt_s",v.critical_dt_s);
    Number(d,"translation_stiffness_N_per_m",v.translation_stiffness_N_per_m);
    Number(d,"rotation_stiffness_N_m_per_rad",v.rotation_stiffness_Nm_per_rad);return d;
}
}
Document ConnectorFrameDocument(const FrameView& v) {
    CheckConnectorFrame(v);
    Require(v.bindings->connectors(),"Connector fields require an explicit source connector model");const auto& model=*v.bindings->connectors();
    Document d;d.SetObject();String(d,"kind",ConnectorKind);Child(d,"diagnostics",Diagnostics(*v.connectors.diagnostics));
    Value elements(rapidjson::kArrayType);
    for(std::size_t i=0;i<v.connectors.count;++i) {
        const auto& c=model.connections()[i];Value row;
        row.CopyFrom(Element(v.connectors.elements[i],c.source_element_id,model.properties()[c.property_index].source_property_id),d.GetAllocator());
        elements.PushBack(row,d.GetAllocator());
    }
    Put(d,"elements",std::move(elements));return d;
}
} // namespace crash::output::assembly::wall_fields
