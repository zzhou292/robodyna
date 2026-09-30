#pragma once
#include "SourcePartElasticFields.h"
#include "source_part_wall/SourcePartWallContact.h"
#include "source_part_elastic/SourcePartElasticWallMetrics.h"
namespace crash::cases::source_part_wall {
namespace elastic=source_part_elastic;
output::Document SourcePartWallConfiguration(const elastic::SourcePartElasticCase&,const visual::Binding&,
                                            std::uint64_t steps,unsigned frame_every);
output::Document SourcePartWallFrameFields(const elastic::Snapshot&,const SourcePartWallSetup&,
    const contact::NodalWallDeviceResults*,const elastic::WallMetrics&);
output::Document SourcePartWallSetupFields(const SourcePartWallSetup&);
output::Document SourcePartWallContactFields(const contact::NodalWallDeviceResults&);
void AppendSourcePartWallMetrics(output::Document&,const elastic::WallMetrics&);
unsigned StrictlySeparatedNodes(const contact::NodalWallDeviceResults*);
void CheckAcceptedWallContact(const tl::fea::NodalStamp&,const elastic::Diagnostics&,
                             const contact::NodalWallDeviceResults*);
}
