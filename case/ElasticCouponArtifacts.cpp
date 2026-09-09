#include "ElasticCouponArtifacts.h"
#include "ElasticCouponFields.h"
#include "output/ArtifactIO.h"
#include "output/MeshArchive.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace crash::case_data {
namespace fs=std::filesystem;
using namespace crash::output;
struct ElasticCouponArtifacts::Impl {
    struct Artifact {std::string file,hash; std::size_t bytes;};
    fs::path directory;
    std::ofstream intervals,frames;
    std::vector<Artifact> inventory;
    tl::fea::NodalStamp last_interval;
    std::uint64_t last_frame_epoch=0,frame_count=0,required_steps=0;
    bool finished=false,failed=false;
    explicit Impl(const std::string& path):directory(path) {}
    void Inventory(const std::string& file) {
        const auto bytes=ReadBounded(directory/file,32*1024*1024);
        inventory.push_back({file,Sha256(bytes),bytes.size()});
    }
    void CheckOwner(const tl::fea::NodalStamp& stamp) const {
        Require(stamp.owner_id==last_interval.owner_id && stamp.node_count==last_interval.node_count &&
                stamp.fixed_dt==last_interval.fixed_dt && stamp.has_rotations,"Coupon artifacts belong to a different owner/configuration");
    }
};

ElasticCouponArtifacts::ElasticCouponArtifacts(const std::string& path,const ElasticCouponCase& run,unsigned frame_every)
    :impl_(std::make_unique<Impl>(path)) {
    Require(run.metrics()&&run.modal()&&run.output(),"Coupon artifacts require an initialized run");
    auto& s=*impl_; s.last_interval=run.metrics()->stamp; s.required_steps=run.metrics()->required_steps;
    Require(s.last_interval.epoch==0 && s.last_interval.time==0 && frame_every>0 &&
            1+(s.required_steps+frame_every-1)/frame_every<=1000,"Coupon output must start at epoch zero within the frame cap");
    Require(fs::create_directory(s.directory),"Coupon output directory must be new");
    WriteJson(s.directory/"configuration.json",CouponConfiguration(run,frame_every)); s.Inventory("configuration.json");
    s.intervals.open(s.directory/"accepted-intervals.csv",std::ios::binary);
    s.frames.open(s.directory/"accepted-frames.csv",std::ios::binary);
    Require(bool(s.intervals)&&bool(s.frames),"Could not create coupon indices");
    s.intervals<<std::setprecision(17); s.frames<<std::setprecision(17);
    s.frames<<"owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization\n";
    s.intervals<<"owner_id,base_epoch,attempt,force_eval_time_s,accepted_epoch,accepted_time_s,elastic_energy_J,bending_energy_J,kinetic_translation_J,kinetic_physical_rotation_J,kinetic_artificial_drilling_J,base_elastic_energy_J,base_kinetic_energy_J,elastic_increment_J,kinetic_increment_J,kinetic_midpoint_work_J,kinetic_work_residual_J,force_coordinate_work_J,force_potential_defect_J,mass_weighted_increment,maximum_displacement_m,maximum_director_departure_rad,maximum_pair_angle_rad,maximum_membrane_shear_strain,maximum_thickness_curvature,minimum_signed_area_ratio,minimum_area_norm_ratio,maximum_area_norm_ratio,minimum_display_triangle_area_ratio,maximum_relative_energy_error,last_operator_norm_per_s2,last_operator_epoch\n";
}
ElasticCouponArtifacts::~ElasticCouponArtifacts()=default;

void ElasticCouponArtifacts::RecordInterval(const tl::fea::NodalStamp& base,const ElasticCouponMetrics& m) {
    auto& s=*impl_; s.CheckOwner(base); s.CheckOwner(m.stamp);
    const auto& d=m.diagnostics;
    Require(!s.finished&&!s.failed&&s.frame_count&&base.epoch==s.last_interval.epoch&&base.time==s.last_interval.time&&
        m.stamp.epoch==base.epoch+1&&m.stamp.time==base.time+base.fixed_dt&&d.valid&&d.owner_id==base.owner_id&&
        d.base_epoch==base.epoch&&d.phase==tl::fea::reissner::ShellBatchPhase::kPreparedCandidate,
        "Coupon interval is not the next committed step");
    const double values[]{d.elastic_energy,d.bending_energy,d.kinetic_translation,d.kinetic_physical_rotation,
        d.kinetic_artificial_drilling,d.base_elastic_energy,d.base_kinetic_energy,d.elastic_energy_increment,
        d.kinetic_energy_increment,d.kinetic_midpoint_work,d.kinetic_work_residual,d.force_coordinate_work,
        d.conservative_force_coordinate_defect,d.mass_weighted_increment_squared,d.maximum_displacement,
        d.maximum_director_departure,d.maximum_pair_angle,d.maximum_membrane_strain,d.maximum_thickness_curvature,
        d.minimum_signed_area_ratio,d.minimum_area_norm_ratio,d.maximum_area_norm_ratio,d.minimum_display_triangle_area_ratio,
        m.maximum_relative_energy_error,m.last_operator_norm};
    for(double value:values) Require(std::isfinite(value),"Nonfinite coupon interval output");
    s.intervals<<base.owner_id<<','<<base.epoch<<','<<d.attempt<<','<<base.time<<','<<m.stamp.epoch<<','<<m.stamp.time;
    for(double value:values)s.intervals<<','<<value;
    s.intervals<<','<<m.last_operator_epoch<<'\n'; Require(bool(s.intervals),"Coupon interval output failed");
    s.last_interval=m.stamp;
}

void ElasticCouponArtifacts::WriteFrame(ElasticCouponCase& run) {
    auto& s=*impl_; Require(!s.finished&&!s.failed&&s.frame_count<1000,"Coupon output is closed or exceeds frame capacity");
    ElasticCouponFrame frame; const auto captured=run.Capture(frame);
    if(captured.status!=CouponStatus::Ok)throw std::runtime_error(captured.diagnostic);
    s.CheckOwner(frame.stamp);
    Require(frame.stamp.epoch==s.last_interval.epoch&&frame.stamp.time==s.last_interval.time&&
        (!s.frame_count||frame.stamp.epoch>s.last_frame_epoch),"Coupon output is not the recorded accepted state");
    const auto* visible=run.output()->surface().frame(); const auto mesh=run.output()->surface().mesh();
    Require(visible&&mesh&&visible->epoch==frame.stamp.epoch&&visible->time==frame.stamp.time&&
            visible->identity.owner==frame.stamp.owner_id,"Coupon visible mesh provenance mismatch");
    std::ostringstream name; name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<frame.stamp.epoch;
    const auto stem=name.str(); WriteMeshFiles(s.directory,stem,*mesh);
    WriteJson(s.directory/(stem+".fields.json"),CouponFrameFields(frame));
    for(const char* suffix:{".mesh.json",".obj",".fields.json"})s.Inventory(stem+suffix);
    s.frames<<frame.stamp.owner_id<<','<<frame.stamp.epoch<<','<<frame.stamp.time<<','<<stem<<".mesh.json,"<<stem<<".obj\n";
    s.frames.flush(); s.intervals.flush(); Require(bool(s.frames)&&bool(s.intervals),"Coupon frame index failed");
    s.last_frame_epoch=frame.stamp.epoch; ++s.frame_count;
}

void ElasticCouponArtifacts::Finish(const ElasticCouponCase& run,double elapsed_seconds) {
    auto& s=*impl_; Require(run.metrics()&&run.modal(),"Missing final coupon state");
    const auto& m=*run.metrics(); s.CheckOwner(m.stamp);
    Require(!s.finished&&!s.failed&&s.frame_count&&m.stamp.epoch==s.required_steps&&
            s.last_interval.epoch==s.required_steps&&s.last_frame_epoch==s.required_steps&&
            std::abs(m.stamp.time-run.modal()->horizon)<=1e-11*run.modal()->horizon,
            "Coupon full half-period or final output is incomplete");
    s.intervals.close(); s.frames.close(); Require(!s.intervals.fail()&&!s.frames.fail(),"Coupon index close failed");
    s.Inventory("accepted-intervals.csv"); s.Inventory("accepted-frames.csv");
    Document final; final.SetObject(); Integer(final,"accepted_epoch",m.stamp.epoch); Number(final,"accepted_time_s",m.stamp.time);
    Number(final,"initial_energy_J",m.initial_energy); Number(final,"final_elastic_energy_J",m.diagnostics.elastic_energy);
    Number(final,"final_kinetic_energy_J",CouponKineticEnergy(m.diagnostics));
    Number(final,"maximum_relative_energy_error",m.maximum_relative_energy_error);
    Number(final,"last_operator_norm_per_s2",m.last_operator_norm); Integer(final,"last_operator_epoch",m.last_operator_epoch);
    Integer(final,"full_state_audit_reads",m.full_state_audit_reads); Integer(final,"saved_frames",s.frame_count);
    Integer(final,"state_owned_device_bytes",run.state_allocations().device_bytes);
    Integer(final,"element_owned_device_bytes",run.element_allocations().device_bytes); Number(final,"elapsed_wall_seconds",elapsed_seconds);
    WriteJson(s.directory/"final-metrics.json",final); s.Inventory("final-metrics.json");
    Document manifest; manifest.SetObject(); String(manifest,"schema","robo_dyna.elastic_coupon_artifacts.v1");
    String(manifest,"status","completed"); String(manifest,"scope","Synthetic force-driven two-Q4 elastic release through one modal half-period");
    Boolean(manifest,"shell_model",true); Boolean(manifest,"vehicle_model",false); Boolean(manifest,"contact",false);
    Integer(manifest,"accepted_epoch",m.stamp.epoch); Number(manifest,"accepted_time_s",m.stamp.time);
    String(manifest,"completion_meaning","Admitted fixed-step horizon committed and output verified; refinement and rendering are separate gates");
    Value inventory(rapidjson::kArrayType);
    for(const auto& entry:s.inventory) {
        Value item(rapidjson::kObjectType),file(entry.file.c_str(),manifest.GetAllocator()),hash(entry.hash.c_str(),manifest.GetAllocator());
        item.AddMember("file",file,manifest.GetAllocator()); item.AddMember("sha256",hash,manifest.GetAllocator());
        item.AddMember("bytes",Value().SetUint64(entry.bytes),manifest.GetAllocator()); inventory.PushBack(item,manifest.GetAllocator());
    }
    manifest.AddMember("artifacts",inventory,manifest.GetAllocator()); WriteJson(s.directory/"manifest.pending.json",manifest);
    Require(!fs::exists(s.directory/"manifest.json"),"Refusing to overwrite completed coupon output");
    fs::rename(s.directory/"manifest.pending.json",s.directory/"manifest.json"); s.finished=true;
}
void ElasticCouponArtifacts::Fail(const std::string& diagnostic) noexcept {
    try {
        auto& s=*impl_; if(s.finished||s.failed)return; s.failed=true; s.intervals.flush(); s.frames.flush();
        Document doc; doc.SetObject(); String(doc,"status","failed"); String(doc,"message",diagnostic);
        Integer(doc,"last_recorded_epoch",s.last_interval.epoch); Number(doc,"last_recorded_time_s",s.last_interval.time);
        WriteJson(s.directory/"failure.json",doc);
    } catch(...) { /* Absent completed manifest continues to mark partial output. */ }
}
}  // namespace crash::case_data
