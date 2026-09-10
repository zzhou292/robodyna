#include "ReplayColorMetadata.h"
#include "chrono/AcceptedReplayScene.h"
#include "output/AcceptedReplay.h"

namespace crash::viewer {
void AppendReplayColorMetadata(output::Document& manifest, const output::ReplayInfo& info,
                               const visual::AcceptedReplayScene& scene) {
    using namespace output;
    using visual::ReplayColorMode;
    Require(scene.stamp()!=nullptr,"Color capture requires an initialized replay scene");
    String(manifest,"surface_color_mode",visual::ReplayColorModeName(scene.color_mode()));
    if (scene.color_mode() == ReplayColorMode::PlasticStrain) {
        String(manifest,"surface_color_quantity","maximum accepted layer equivalent plastic strain per original source parent");
        String(manifest,"surface_color_palette","piecewise linear blue(0.12,0.64,0.94), yellow(0.98,0.84,0.16), red(0.90,0.12,0.10)");
        String(manifest,"surface_color_scale_policy","fixed 0..max(0.001, accepted final maximum plastic strain); endpoint saturation; no frame autoscale");
        Number(manifest,"surface_color_min",0);
        Number(manifest,"surface_color_max",info.plastic_strain_color_max);
        String(manifest,"surface_color_units","dimensionless; visible legend in percent");
        Boolean(manifest,"surface_color_flat_per_source_parent",true);
    } else if (scene.color_mode() == ReplayColorMode::PartId) {
        String(manifest,"surface_color_quantity","original source part ID; categorical, no stress or material meaning");
        String(manifest,"surface_color_palette",visual::ReplayPartPaletteName);
        Integer(manifest,"surface_color_palette_seed",visual::ReplayPartPaletteSeed);
        String(manifest,"surface_color_scale_policy","fixed original-PID mapping; independent of frame, order and subset");
        Boolean(manifest,"surface_color_flat_per_source_part",true);
        String(manifest,"surface_color_legend_columns","original_part_id,R,G,B");
        Value legend(rapidjson::kArrayType);
        for (const auto& entry : *scene.part_legend()) {
            Value row(rapidjson::kArrayType);
            row.PushBack(Value().SetUint64(entry.part_id),manifest.GetAllocator());
            for (float channel : {entry.color.R,entry.color.G,entry.color.B})
                row.PushBack(Value().SetDouble(channel),manifest.GetAllocator());
            legend.PushBack(row,manifest.GetAllocator());
        }
        manifest.AddMember("surface_color_legend",legend,manifest.GetAllocator());
    } else String(manifest,"surface_color_quantity","uniform moving-surface display color");
    const auto* fields=scene.scalar_legend();
    if (fields && (fields->not_applicable || fields->unavailable)) {
        Integer(manifest,"native_plastic_parent_count",fields->native);
        Integer(manifest,"plastic_not_applicable_parent_count",fields->not_applicable);
        Integer(manifest,"plastic_unavailable_parent_count",fields->unavailable);
        String(manifest,"non_native_plastic_value_policy","explicit applicability; no interpreted numeric value");
        String(manifest,"plastic_missing_color_policy","not applicable: gray(0.48,0.50,0.52); unavailable: purple(0.72,0.30,0.74)");
    }
}
} // namespace crash::viewer
