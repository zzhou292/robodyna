#pragma once
#include "SourceAssemblyWallFields.h"

namespace crash::output::assembly::wall_fields {
inline constexpr char ConnectorScope[]="Original seven-part Yaris component, native TYPE25 spotweld and nodal rigid groups active, external connections explicitly released";
inline constexpr const char* ConnectorKind="openradioss_type25_linear_finite_offset_v1";
inline constexpr const char* ConnectorPolicy="openradioss_tonne_millimetre_second_direct_import";
inline constexpr std::size_t MaxArchivedConnectors=128;
Document ConnectorInputDocument(const cases::source_assembly::SourceAssemblyBindings&);
void CheckConnectorFrame(const FrameView&);
Document ConnectorFrameDocument(const FrameView&);
} // namespace crash::output::assembly::wall_fields
