#include "Environment.h"
namespace crash::output::recovered_frames {
run::EnvironmentReceipt InspectEnvironment(const std::filesystem::path& root,const run::Configuration& config) {
    Require(config.environment && !config.wall,"Environment recovery requires the declared exclusive profile");
    run::EnvironmentReceipt receipt;
    receipt.source_instance_id=config.identity.source_instance;
    receipt.source_mapping_sha256=config.identity.source_mapping_sha256;
    for(std::size_t i=0;i<receipt.files.size();++i)
        receipt.files[i]=InspectFile(root,run::EnvironmentFiles[i],run::EnvironmentFileCap);
    const auto doc=array_json::Parse(run::ReadFile(root,receipt.files[2],run::EnvironmentFileCap),run::EnvironmentFileCap);
    Require(doc.IsObject() && doc.HasMember("wall_binding_id") && doc.HasMember("part_id"),
        "Declared environment lacks its binding and part identity");
    receipt.wall_binding_id=array_json::UInt(doc["wall_binding_id"]);
    receipt.part_id=array_json::UInt(doc["part_id"]);
    run::EnvironmentDocument(receipt);
    return receipt;
}
void CheckStaticProfiles(const run::Configuration& config,const Description& description) {
    Require(config.wall==bool(description.wall) && config.environment==bool(description.environment) &&
        !(description.wall && description.environment),"Recovered static profile/receipt presence differs");
}
} // namespace crash::output::recovered_frames
