%{
#include "chrono/physics/ChObject.h"
using namespace chrono;
%}

%shared_ptr(chrono::ChObj)

%ignore chrono::ChObj::Clone;
%ignore chrono::ChObj::ArchiveContainerName;

// Native-only additions; retain the qualified legacy binding surface.
%ignore chrono::ChObj::GetTime;
%ignore chrono::ChObj::SetTime;

%include "../../../chrono/physics/ChObject.h"    

