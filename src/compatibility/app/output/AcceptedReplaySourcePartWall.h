#pragma once
#include "AcceptedReplayValues.h"
namespace crash::output::replay_detail {
void ReadSourcePartWallIntervals(Bundle&,const Document&,const Document&);
void CheckPlacedSourcePartWall(Bundle&,const Document& configuration);
unsigned CheckSourcePartWallContact(const Bundle&,const Entry&,const Value&);
}
