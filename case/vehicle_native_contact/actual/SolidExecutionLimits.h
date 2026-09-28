#pragma once
#include "lib_src/elements/solids/resident/Batch.h"
#include "output/ArtifactIO.h"
#include <cstring>
namespace crash::cases::vehicle_native_contact::test {
// An explicit execution-resource request, independent of physical source data.
// The batch may choose fewer workers to fit both ceilings; the whole case's
// existing host/device forecasts and workstation guard remain authoritative.
inline tl::fea::solids::BatchLimits SolidExecutionLimits(const char* text) {
    auto limits=tl::fea::solids::SourceControlledBatchLimits();
    if(!text||!*text)return limits;
    if(std::strcmp(text,"4")==0)limits.max_controlled_packet_blocks=4;
    else if(std::strcmp(text,"8")==0)limits.max_controlled_packet_blocks=8;
    else if(std::strcmp(text,"16")==0)limits.max_controlled_packet_blocks=16;
    else if(std::strcmp(text,"32")==0)limits.max_controlled_packet_blocks=32;
    else output::Require(false,"Solid worker request must be 4,8,16 or32");
    if(limits.max_controlled_packet_blocks>8){
        limits.max_device_bytes=256u<<20;
        limits.max_host_bytes=384u<<20;
    }
    return limits;
}
}
