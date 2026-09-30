#include "Storage.h"
namespace crash::cases::vehicle_startup::joints {
VehicleJointModel VehicleJointModel::Prepare(const Physical& physical,const Source& source,Limits limits) {
    const auto forecast=Preflight(physical,source,limits);
    auto next=std::make_shared<Storage>(physical,source,forecast);
    const auto& data=source.data();
    std::vector<tl::fea::type45::JointInput> input;
    input.reserve(data.required);next->rows.reserve(data.required);
    for(std::size_t i=0;i<data.rows.size();++i) {
        if(data.rows[i].disposition==modelio::type45::Disposition::OmittedAssemblyBoundary) continue;
        input.push_back(detail::Pack(data.rows[i],data));next->rows.push_back(static_cast<std::uint32_t>(i));
    }
    output::Require(input.size()==data.required,"Retained original joint coverage differs");
    const auto checked=next->model.Initialize(physical.rigid_assembly(),
        {physical.source_domain().domain().source_instance_id(),{input.data(),input.size()}},limits.model);
    output::Require(bool(checked),checked.message);
    output::Require(next->model.startup_payload_bytes()<=forecast.model_reservation,
        "Native joint startup exceeded its complete reservation");
    return VehicleJointModel(std::move(next));
}
const Physical& VehicleJointModel::physical() const noexcept {return storage_->physical;}
const Source& VehicleJointModel::source() const noexcept {return storage_->source;}
const tl::fea::type45::Model& VehicleJointModel::model() const noexcept {return storage_->model;}
const std::vector<std::uint32_t>& VehicleJointModel::source_rows() const noexcept {return storage_->rows;}
const Forecast& VehicleJointModel::forecast() const noexcept {return storage_->forecast;}
} // namespace crash::cases::vehicle_startup::joints
