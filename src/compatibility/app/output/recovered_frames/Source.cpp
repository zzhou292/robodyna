#include "Environment.h"

namespace crash::output::recovered_frames {
LoadedSource LoadSource(const std::filesystem::path& root,const Description& description,Limits limits) {
    Require(limits.host_bytes && limits.host_bytes<=512u<<20 && limits.source.host_bytes<=limits.host_bytes &&
        (128u<<20)<=limits.host_bytes-limits.source.host_bytes,"Recovery source/record workspace exceeds host cap");
    auto config=run::ReadConfiguration(array_json::Parse(
        run::ReadFile(root,description.configuration,run::MetadataCap),run::MetadataCap));
    CheckStaticProfiles(config,description);
    Require(config.identity.source_mapping_sha256==description.mapping_sha256,
        "Recovered configuration/source mapping differs");
    auto mapping=records::source::ReadSourceBundle(root,description.source_bundle,description.source,
        description.mapping_sha256,limits.source);
    auto context=mapping.MakeFrameContext(config.identity,config.request.fixed_dt,limits.records);
    Require(context.point_layout_sha256()==config.point_layout_sha256 && context.nodes()==config.request.nodes &&
        context.points()==config.request.plastic_points && context.parents().size()==config.request.parents,
        "Recovery frame layout differs from source mapping");
    const auto bytes=Budget(context,config,limits);
    records::activity::ReadDeclaration(root,context,description.activity_declaration);
    return {std::move(mapping),std::move(context),std::move(config),bytes};
}
} // namespace crash::output::recovered_frames
