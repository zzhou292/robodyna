#pragma once
#include "source_part_elastic/SourcePartElasticCase.h"
#include "output/ArtifactIO.h"
namespace crash::cases::source_part_plastic {
bool PlasticEnabled(const source_part_elastic::Config&);
const char* MaterialModelName(const source_part_elastic::Config&);
std::string MaterialPolicy(const source_part_elastic::Config&);
void AppendSourcePartPlasticConfiguration(output::Document&,const source_part_elastic::SourcePartElasticCase&);
void AppendSourcePartPlasticSummary(output::Document&,const PlasticSummary&);
void AppendSourcePartPlasticFrame(output::Document&,source_part_elastic::SourcePartElasticCase&,
                                 const source_part_elastic::Snapshot&);
}
