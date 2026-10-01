// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Copyright (c) 2014 projectchrono.org
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in LICENSES/Chrono-BSD-3-Clause.txt at the Robodyna root and at
// http://projectchrono.org/license-chrono.txt.
//
// Authors: Radu Serban
//
// Robodyna adaptation: FE attachment/update definitions extracted from
// ChObject.cpp (2014 notice) and ChVisualModel.cpp (2022 notice, author above).
// Geometry and ownership semantics retained.
// =============================================================================

#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/physics/ChObject.h"

#ifdef CHRONO_FEA
namespace chrono {

void ChVisualModel::AddShapeFEA(std::shared_ptr<ChVisualShapeFEA> shapeFEA) {
    // Install before the original append sequence, including its exception
    // behavior: a partially appended FE vector must still have its updater.
    m_fea_updater = &ChVisualModel::UpdateFEAShapes;
    m_shapesFEA.push_back(shapeFEA);
    m_shapes.push_back({shapeFEA->m_trimesh_shape, ChFramed(), false});
    m_shapes.push_back({shapeFEA->m_glyphs_shape, ChFramed(), false});
}

void ChVisualModel::UpdateFEAShapes(ChVisualModel& model, ChObj* owner) {
    for (auto& shapeFEA : model.m_shapesFEA) {
        // Intentionally nonvirtual. The FE visual uses its stored object owner
        // and already produces world-coordinate geometry.
        shapeFEA->Update(owner, ChFrame<>());
    }
}

void ChObj::AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape) {
    shape->obj = this;
    if (!vis_model_instance) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        AddVisualModel(model);
    }
    vis_model_instance->GetModel()->AddShapeFEA(shape);
}

}  // namespace chrono
#endif
