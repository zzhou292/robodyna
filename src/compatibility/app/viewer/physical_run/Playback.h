#pragma once
#include "chrono/physical_run/Scene.h"
#include "chrono_vsg/ChGuiComponentVSG.h"
namespace crash::viewer::physical_run {
struct Playback {bool paused=false,next=false,close=false;};
std::shared_ptr<chrono::vsg3d::ChGuiComponentVSG> MakeOverlay(
    const visual::physical_run::Scene&,Playback&,bool capture,double fps);
} // namespace crash::viewer::physical_run
