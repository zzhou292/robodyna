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
        if(ImGui::Begin("robo-dyna | physical accepted replay",nullptr,flags)) {
            const auto& replay=*scene.replay();const auto& index=replay.index();const auto& stamp=*scene.stamp();
            ImGui::TextUnformatted("Original Yaris | selected physical model | physical scale");
            ImGui::Text("Accepted time %.6f ms | epoch %llu",stamp.time*1000,static_cast<unsigned long long>(stamp.epoch));
            ImGui::Text("Recorded sample %zu / %zu",stamp.index+1,index.frames.size());
            ImGui::TextUnformatted(index.horizon_complete?"Requested archive horizon complete":"Accepted prefix | requested horizon incomplete");
            if(!index.horizon_complete && !index.stop_reason.empty())
                ImGui::TextUnformatted("Stop reason retained in archive and capture metadata");
            ImGui::TextUnformatted(replay.wall()?"Gray wireframe: authenticated selected finite wall":"No wall participant in this archive");
            ImGui::Text("%zu original shell parents | active faces %zu",replay.context().parents().size(),
                scene.geometry()->triangle_source_parents()->size());
            if(scene.geometry()->color_mode()==visual::ReplayColorMode::PartId) {
                ImGui::TextUnformatted("Color: original part ID | fixed palette across samples");
                const auto& legend=*scene.geometry()->part_legend();
                for(std::size_t i=0;i<std::min<std::size_t>(8,legend.size());++i) {
                    if(i%4) ImGui::SameLine();
                    const auto& e=legend[i];
                    ImGui::TextColored(ImVec4(e.color.R,e.color.G,e.color.B,1),"PID %llu",
                        static_cast<unsigned long long>(e.part_id));
                }
            } else if(scene.geometry()->color_mode()==visual::ReplayColorMode::PlasticStrain) {
                ImGui::TextUnformatted("Color: maximum native equivalent plastic strain per parent");
                ImGui::Text("Fixed scale 0 .. %.5g %% | source 1/3/4-point values",100*scene.plastic_strain_maximum());
                const auto* legend=scene.geometry()->scalar_legend();
                ImGui::Text("Not applicable: %zu | unavailable: %zu",legend->not_applicable,legend->unavailable);
            } else ImGui::TextUnformatted("Uniform surface display color");
            ImGui::TextUnformatted("Kinetic/energy/contact work channels unavailable in this archive profile");
            if(capture) ImGui::TextUnformatted("Indexed PNG capture | exact archived samples | fixed camera");
            else {
                ImGui::Text("Playback %.1f recorded samples/s",fps);
                if(ImGui::Button(playback.paused?"Play":"Pause")) playback.paused=!playback.paused;
                ImGui::SameLine();
                if(ImGui::Button("Next sample")) {playback.next=true;playback.paused=true;}
                ImGui::SameLine();
                if(ImGui::Button("Close")) playback.close=true;
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
