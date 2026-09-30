#pragma once
#include "DeclaredSource.h"
#include "output/BoundedArrayJson.h"
namespace crash::modelio::native_scene::detail {
void BindTiedPatch(DeclaredData&,const output::Value& exported,const output::Value& original);
}
