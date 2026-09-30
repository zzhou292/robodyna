#pragma once
#include "../TiedCinAttachments.h"
namespace crash::cases::vehicle_startup::tied_cin_detail {
using Inputs = std::vector<native_search::CinAttachmentDeclaration>;
Inputs Pack(const tied::source::CanonicalData&,const tied::Data&,const tied::PackingData&,
    const tied::SearchGeometryData&,const native_search::FinalizationMaps&,const TiedClassificationData&,
    const native_search::PostKinChkResult&);
TiedCinAttachmentForecast Budget(const TiedPostKinChkForecast&,const native_search::PostKinChkResult&,
    const tl::fea::NodalNodeDomain&,std::size_t canonical_nodes,std::size_t fixed_bytes,TiedCinAttachmentLimits);
}
