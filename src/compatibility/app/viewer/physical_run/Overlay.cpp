#include "Playback.h"
#include "chrono/ReplayParentScalarColors.h"
#include <algorithm>
namespace crash::viewer::physical_run {
namespace {
class Overlay final:public chrono::vsg3d::ChGuiComponentVSG {
  public:
    Overlay(const visual::physical_run::Scene& s,Playback& p,bool c,double fps):scene(s),playback(p),capture(c),fps(fps) {}
    void render(vsg::CommandBuffer&) override {
        ImGui::SetNextWindowPos(ImVec2(12,12),ImGuiCond_Always);
        const auto flags=ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse|
            ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoSavedSettings;
        if(ImGui::Begin("robo-dyna | Physical simulation replay",nullptr,flags)) {
            const auto& replay=*scene.samples();const auto& stamp=*scene.stamp();
            ImGui::Text("Time %.3f us (%.6f ms) | sample %zu / %zu",stamp.time*1e6,stamp.time*1000,stamp.index+1,replay.frames().size());
            if (replay.recovered()) {
                ImGui::TextUnformatted("Recovered interrupted saved samples");
                ImGui::TextUnformatted("Interval ledger unavailable | completion unknown");
            } else ImGui::TextUnformatted(replay.normal()->index().horizon_complete?
                "Requested duration complete":"Diagnostic prefix | trajectory incomplete");
            ImGui::TextUnformatted("Original shell assembly | deformation scale 1x");
            ImGui::TextUnformatted(replay.environment()?"Declared fixed environment mesh":replay.wall()?"Gray wireframe: mesh wall":"No additional wall mesh");
            if(scene.geometry()->color_mode()==visual::ReplayColorMode::PartId) {
                ImGui::TextUnformatted("Colors: original parts, consistent across frames");
            } else if(scene.geometry()->color_mode()==visual::ReplayColorMode::PlasticStrain) {
                ImGui::TextUnformatted("Colors: maximum equivalent plastic strain per shell");
                ImGui::Text("Fixed scale 0 .. %.5g %%",100*scene.plastic_strain_maximum());
                const auto* legend=scene.geometry()->scalar_legend();
                ImGui::Text("Not applicable: %zu | unavailable: %zu",legend->not_applicable,legend->unavailable);
            } else ImGui::TextUnformatted("Uniform surface display color");
            if(!capture) {
                ImGui::Text("Playback %.1f recorded samples/s",fps);
                if(ImGui::Button(playback.paused?"Play":"Pause")) playback.paused=!playback.paused;
                ImGui::SameLine();
                if(ImGui::Button("Next sample")) {playback.next=true;playback.paused=true;}
                ImGui::SameLine();
                if(ImGui::Button("Close")) playback.close=true;
                if(ImGui::CollapsingHeader("Model details")) {
                    ImGui::Text("%zu source shells | %zu active triangles",replay.context().parents().size(),
                        scene.geometry()->triangle_source_parents()->size());
                    ImGui::Text("Accepted epoch %llu",static_cast<unsigned long long>(stamp.epoch));
                    if(!replay.stop_reason().empty()) ImGui::TextWrapped("%s",replay.stop_reason().c_str());
                    if(const auto* legend=scene.geometry()->part_legend()) {
                        for(std::size_t i=0;i<std::min<std::size_t>(8,legend->size());++i) {
                            if(i%4) ImGui::SameLine();
                            const auto& e=(*legend)[i];
                            ImGui::TextColored(ImVec4(e.color.R,e.color.G,e.color.B,1),"PID %llu",
                                static_cast<unsigned long long>(e.part_id));
                        }
                    }
                }
            }
        }
        ImGui::End();
    }
  private:
    const visual::physical_run::Scene& scene;
    Playback& playback;
    bool capture;
    double fps;
};
}
std::shared_ptr<chrono::vsg3d::ChGuiComponentVSG> MakeOverlay(
        const visual::physical_run::Scene& scene,Playback& playback,bool capture,double fps) {
    return std::make_shared<Overlay>(scene,playback,capture,fps);
}
} // namespace crash::viewer::physical_run
