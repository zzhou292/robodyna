#include "Capture.h"
#include "output/BoundedArrayJson.h"
namespace crash::viewer::physical_run {
output::Document CaptureMetadata(const Options& options,const Input& input,const visual::physical_run::Scene& scene,
        FixedReplayVisual& visual,ReplayLighting light,std::size_t image_bytes,const std::string& index_bytes) {
    using namespace output;
    const auto& replay=*scene.replay();const auto& index=replay.index();
    Document d;d.SetObject();String(d,"schema","robo_dyna.physical_replay_capture.v1");
    array_json::Child(d,"input_receipt",output::physical_run::FileDocument(input.receipt));
    array_json::Child(d,"input_archive_manifest",output::physical_run::FileDocument(input.values.manifest));
    String(d,"source_mapping_sha256",input.values.mapping_sha256);
    String(d,"input_authority","caller-selected controller receipt; hashes verify coherence, not physical validity");
    Boolean(d,"complete_capture",true);Boolean(d,"input_horizon_complete",index.horizon_complete);
    String(d,"input_stop_reason",index.stop_reason);Boolean(d,"simulation_executed_by_viewer",false);
    Boolean(d,"interpolated_frames",false);Number(d,"deformation_scale",1);
    Integer(d,"owner_id",replay.context().identity().owner);Integer(d,"frames",index.frames.size());
    Integer(d,"final_epoch",index.final.epoch);Number(d,"final_time_s",index.final.time);
    Number(d,"requested_duration_s",replay.configuration().request.requested_duration);
    array_json::Child(d,"observation_profile",output::physical_run::ProfileDocument(replay.configuration().profile));
    Integer(d,"presentation_peak_host_bytes",scene.forecast()->peak_host_bytes);
    String(d,"host_forecast_scope","application reader/geometry/sample workspace; renderer and PNG buffers measured separately");
    Integer(d,"png_bytes",image_bytes);Integer(d,"capture_forecast_bytes",CaptureForecast(index.frames.size(),options.capture_bytes));
    String(d,"frame_index_sha256",Sha256(index_bytes));
    Boolean(d,"all_png_decoded",true);Integer(d,"renders_per_sample",2);Integer(d,"initial_warmup_renders",1);
    Number(d,"recorded_samples_per_second",options.frames_per_second);
    String(d,"surface_color_mode",visual::ReplayColorModeName(scene.geometry()->color_mode()));
    if(const auto* legend=scene.geometry()->part_legend()) {
        String(d,"part_palette",visual::ReplayPartPaletteName);
        Integer(d,"part_palette_seed",visual::ReplayPartPaletteSeed);
        Value rows(rapidjson::kArrayType);
        for(const auto& entry:*legend) {
            Value row(rapidjson::kArrayType);row.PushBack(Value().SetUint64(entry.part_id),d.GetAllocator());
            for(float c:{entry.color.R,entry.color.G,entry.color.B}) row.PushBack(Value().SetDouble(c),d.GetAllocator());
            rows.PushBack(row,d.GetAllocator());
        }
        d.AddMember("part_id_rgb_legend",rows,d.GetAllocator());
    }
    Number(d,"plastic_strain_color_maximum",scene.plastic_strain_maximum());
    String(d,"plastic_scale_policy","fixed maximum over all archived native point values; all-zero field uses display maximum one");
    String(d,"activity_policy","only actually active original source parents contribute display triangles");
    String(d,"wall_display",replay.wall()?"authenticated selected mesh, gray wireframe":"none");
    if(replay.wall()) array_json::Child(d,"wall_receipt",output::physical_run::WallDocument(*replay.wall()));
    const auto& camera=*scene.camera();
    FiniteArray(d,"camera_position",camera.position.data(),3);FiniteArray(d,"camera_target",camera.target.data(),3);
    String(d,"camera_view",visual::ReplayViewName(camera.view));String(d,"camera_vertical","Z");
    Number(d,"camera_vertical_fov_degrees",camera.vertical_fov_degrees);
    Number(d,"light_azimuth",light.azimuth);Number(d,"light_elevation",light.elevation);
    const auto dimensions=visual.FramebufferSize();Integer(d,"width",dimensions[0]);Integer(d,"height",dimensions[1]);
    const auto device=visual.GetWindow()->getPhysicalDevice()->getProperties();
    String(d,"vulkan_device_name",device.deviceName);Integer(d,"vulkan_vendor_id",device.vendorID);
    Integer(d,"vulkan_device_id",device.deviceID);Integer(d,"loading_workers",visual.GetLoadingThreadCount());
    return d;
}
} // namespace crash::viewer::physical_run
