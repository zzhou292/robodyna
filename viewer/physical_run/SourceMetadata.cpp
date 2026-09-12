#include "Options.h"
#include "output/BoundedArrayJson.h"
namespace crash::viewer::physical_run {
output::Document SourceMetadata(const Input& input,const visual::physical_run::SampleSource& source) {
    using namespace output;
    Document d;d.SetObject();
    if (const auto* replay=source.normal()) {
        Require(!input.recovered,"Normal capture requires a normal viewer receipt");
        const auto& index=replay->index();
        String(d,"schema","robo_dyna.physical_replay_capture.v1");
        array_json::Child(d,"input_receipt",output::physical_run::FileDocument(input.receipt));
        array_json::Child(d,"input_archive_manifest",output::physical_run::FileDocument(input.values.manifest));
        String(d,"source_mapping_sha256",input.values.mapping_sha256);
        String(d,"input_authority","caller-selected controller receipt; hashes verify coherence, not physical validity");
        Boolean(d,"complete_capture",true);Boolean(d,"input_horizon_complete",index.horizon_complete);
        String(d,"input_stop_reason",index.stop_reason);Boolean(d,"simulation_executed_by_viewer",false);
        Boolean(d,"interpolated_frames",false);Number(d,"deformation_scale",1);
        Integer(d,"owner_id",replay->context().identity().owner);Integer(d,"frames",index.frames.size());
        Integer(d,"final_epoch",index.final.epoch);Number(d,"final_time_s",index.final.time);
        Number(d,"requested_duration_s",replay->configuration().request.requested_duration);
        array_json::Child(d,"observation_profile",output::physical_run::ProfileDocument(replay->configuration().profile));
    } else {
        const auto& recovered=*source.recovered();
        const auto& descriptor=recovered.descriptor();
        Require(input.recovered && input.receipt.file==descriptor.file &&
            input.receipt.sha256==descriptor.sha256 && input.receipt.bytes==descriptor.bytes,
            "Recovered capture differs from the authenticated descriptor");
        Require(!source.frames().empty(),"Recovered capture has no saved samples");
        String(d,"schema","robo_dyna.recovered_sample_capture.v1");
        array_json::Child(d,"input_recovery_descriptor",output::physical_run::FileDocument(descriptor));
        String(d,"source_mapping_sha256",source.context().identity().source_mapping_sha256);
        String(d,"input_authority","caller-selected recovery descriptor; interrupted saved samples only");
        Boolean(d,"complete_capture",true);String(d,"input_horizon_completion","unknown");
        String(d,"input_stop_reason",recovered.stop_reason());
        Boolean(d,"interval_ledger_available",false);Boolean(d,"continuous_accepted_history_available",false);
        Boolean(d,"simulation_executed_by_viewer",false);Boolean(d,"interpolated_frames",false);
        Number(d,"deformation_scale",1);Integer(d,"owner_id",source.context().identity().owner);
        Integer(d,"frames",source.frames().size());
        Integer(d,"final_saved_epoch",source.frames().back().stamp.epoch);
        Number(d,"final_saved_time_s",source.frames().back().stamp.time);
        Number(d,"recorded_requested_duration_s",source.configuration().request.requested_duration);
        array_json::Child(d,"declared_observation_profile",output::physical_run::ProfileDocument(source.configuration().profile));
    }
    return d;
}
} // namespace crash::viewer::physical_run
