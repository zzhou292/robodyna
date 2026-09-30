#pragma once
#include "../TiedCinWitnessRoster.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::cin_witness_detail {
TiedCinWitnessForecast Budget(std::size_t retained,std::size_t nodes,std::size_t parents,
    std::size_t attachments,std::size_t fixed,TiedCinWitnessLimits);
TiedCinWitnessData Build(const tl::fea::ShellBatchBinding&,const std::vector<ReferenceRow>&,
    native_search::ClassificationView<native_search::CinAttachmentRow>,const tl::fea::NodalNodeDomain&,
    TiedCinWitnessLimits);
void CheckActualCapacity(const TiedCinWitnessData&,const TiedCinWitnessForecast&,TiedCinWitnessLimits);
}
