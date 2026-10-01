// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in LICENSES/Chrono-BSD-3-Clause.txt at the Robodyna root and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban, Arman Pazouki
// Robodyna adaptation: canonical body-family implementation; legacy archive tags and equations retained.
// =============================================================================
//
// Classes for creating easy-to-use bodies that optionally include contact and
// visualization shapes.
//
// =============================================================================

#include "robodyna/mbd/RbBodyEasy.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChMassProperties.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"

namespace chrono {
// Keep the exact inherited factory and casting identities through reverse aliases.
CH_FACTORY_REGISTER(ChBodyEasySphere)
CH_UPCASTING(ChBodyEasySphere, ChBody)
CH_FACTORY_REGISTER(ChBodyEasyEllipsoid)
CH_UPCASTING(ChBodyEasyEllipsoid, ChBody)
CH_FACTORY_REGISTER(ChBodyEasyCylinder)
CH_UPCASTING(ChBodyEasyCylinder, ChBody)
CH_FACTORY_REGISTER(ChBodyEasyBox)
CH_UPCASTING(ChBodyEasyBox, ChBody)
CH_FACTORY_REGISTER(ChBodyEasyConvexHull)
CH_UPCASTING(ChBodyEasyConvexHull, ChBody)
CH_FACTORY_REGISTER(ChBodyEasyConvexHullAuxRef)
CH_UPCASTING(ChBodyEasyConvexHullAuxRef, ChBodyAuxRef)
CH_FACTORY_REGISTER(ChBodyEasyMesh)
CH_UPCASTING(ChBodyEasyMesh, ChBodyAuxRef)
CH_FACTORY_REGISTER(ChBodyEasyClusterOfSpheres)
CH_UPCASTING(ChBodyEasyClusterOfSpheres, ChBody)
}

namespace robodyna::mbd {
// Implementation-only lookup keeps inherited helper resolution and arithmetic unchanged.
using namespace ::chrono;
RbBodyEasySphere::RbBodyEasySphere(double radius, double density, bool create_visualization, bool create_collision, std::shared_ptr<ChContactMaterial> material) : RbBody() {
    SetupBody(radius, density, create_visualization, create_collision, material);
}

RbBodyEasySphere::RbBodyEasySphere(double radius, double density, std::shared_ptr<ChContactMaterial> material) : RbBody() {
    SetupBody(radius, density, true, true, material);
}

void RbBodyEasySphere::SetupBody(double radius, double density, bool create_visualization, bool create_collision, std::shared_ptr<ChContactMaterial> material) {
    double mmass = density * (CH_4_3 * CH_PI * std::pow(radius, 3));
    double inertia = (2.0 / 5.0) * mmass * std::pow(radius, 2);

    SetMass(mmass);
    SetInertiaXX(ChVector3d(inertia, inertia, inertia));

    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radius);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeSphere>(radius);
        AddVisualShape(vshape);
    }
}

void RbBodyEasySphere::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasySphere>();
}

void* RbBodyEasySphere::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasySphere>();

    RbBodyEasySphere* new_obj = new RbBodyEasySphere();

    return new_obj;
}

// -----------------------------------------------------------------------------
RbBodyEasyEllipsoid::RbBodyEasyEllipsoid(const ChVector3d& axes, double density, bool create_visualization, bool create_collision, std::shared_ptr<ChContactMaterial> material)
    : RbBody() {
    SetupBody(axes, density, create_visualization, create_collision, material);
}

RbBodyEasyEllipsoid::RbBodyEasyEllipsoid(const ChVector3d& axes, double density, std::shared_ptr<ChContactMaterial> material) : RbBody() {
    SetupBody(axes, density, true, true, material);
}

void RbBodyEasyEllipsoid::SetupBody(const ChVector3d& axes, double density, bool create_visualization, bool create_collision, std::shared_ptr<ChContactMaterial> material) {
    double mmass = density * ((1 / 6.0) * CH_PI * axes.x() * axes.y() * axes.z());
    double inertiax = (1 / 20.0) * mmass * (std::pow(axes.y(), 2) + std::pow(axes.z(), 2));
    double inertiay = (1 / 20.0) * mmass * (std::pow(axes.x(), 2) + std::pow(axes.z(), 2));
    double inertiaz = (1 / 20.0) * mmass * (std::pow(axes.x(), 2) + std::pow(axes.y(), 2));

    SetMass(mmass);
    SetInertiaXX(ChVector3d(inertiax, inertiay, inertiaz));

    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeEllipsoid>(material, axes);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeEllipsoid>(axes);
        AddVisualShape(vshape);
    }
}

void RbBodyEasyEllipsoid::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasyEllipsoid>();
}

void* RbBodyEasyEllipsoid::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasyEllipsoid>();

    RbBodyEasyEllipsoid* new_obj = new RbBodyEasyEllipsoid();

    return new_obj;
}

// -----------------------------------------------------------------------------
RbBodyEasyCylinder::RbBodyEasyCylinder(ChAxis direction,
                                       double radius,
                                       double height,
                                       double density,
                                       bool create_visualization,
                                       bool create_collision,
                                       std::shared_ptr<ChContactMaterial> material)
    : RbBody() {
    SetupBody(direction, radius, height, density, create_visualization, create_collision, material);
}

RbBodyEasyCylinder::RbBodyEasyCylinder(ChAxis direction, double radius, double height, double density, std::shared_ptr<ChContactMaterial> material) : RbBody() {
    SetupBody(direction, radius, height, density, true, true, material);
}

void RbBodyEasyCylinder::SetupBody(ChAxis direction,
                                   double radius,
                                   double height,
                                   double density,
                                   bool create_visualization,
                                   bool create_collision,
                                   std::shared_ptr<ChContactMaterial> material) {
    double mass = density * (CH_PI * std::pow(radius, 2) * height);
    double I_axis = 0.5 * mass * std::pow(radius, 2);
    double I_orth = (1 / 12.0) * mass * (3 * std::pow(radius, 2) + std::pow(height, 2));
    ChQuaternion<> rot;

    SetMass(mass);

    switch (direction) {
        case ChAxis::X:
            rot = QuatFromAngleY(CH_PI_2);
            SetInertiaXX(ChVector3d(I_axis, I_orth, I_orth));
            break;
        case ChAxis::Y:
            rot = QuatFromAngleX(CH_PI_2);
            SetInertiaXX(ChVector3d(I_orth, I_axis, I_orth));
            break;
        case ChAxis::Z:
            rot = QUNIT;
            SetInertiaXX(ChVector3d(I_orth, I_orth, I_axis));
            break;
    }

    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, height);
        AddCollisionShape(cshape, ChFrame<>(VNULL, rot));
        EnableCollision(true);
    }

    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, height);
        AddVisualShape(vshape, ChFrame<>(VNULL, rot));
    }
}

void RbBodyEasyCylinder::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasyCylinder>();
}

void* RbBodyEasyCylinder::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasyCylinder>();

    RbBodyEasyCylinder* new_obj = new RbBodyEasyCylinder();

    return new_obj;
}

// -----------------------------------------------------------------------------

RbBodyEasyBox::RbBodyEasyBox(double Xsize,
                             double Ysize,
                             double Zsize,
                             double density,
                             bool create_visualization,
                             bool create_collision,
                             std::shared_ptr<ChContactMaterial> material)
    : RbBody() {
    SetupBody(Xsize, Ysize, Zsize, density, create_visualization, create_collision, material);
}

RbBodyEasyBox::RbBodyEasyBox(double Xsize, double Ysize, double Zsize, double density, std::shared_ptr<ChContactMaterial> material) : RbBody() {
    SetupBody(Xsize, Ysize, Zsize, density, true, true, material);
}

void RbBodyEasyBox::SetupBody(double Xsize,
                              double Ysize,
                              double Zsize,
                              double density,
                              bool create_visualization,
                              bool create_collision,
                              std::shared_ptr<ChContactMaterial> material) {
    double mmass = density * (Xsize * Ysize * Zsize);

    SetMass(mmass);
    SetInertiaXX(ChVector3d((1.0 / 12.0) * mmass * (std::pow(Ysize, 2) + std::pow(Zsize, 2)), (1.0 / 12.0) * mmass * (std::pow(Xsize, 2) + std::pow(Zsize, 2)),
                            (1.0 / 12.0) * mmass * (std::pow(Xsize, 2) + std::pow(Ysize, 2))));
    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeBox>(material, Xsize, Ysize, Zsize);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeBox>(Xsize, Ysize, Zsize);
        AddVisualShape(vshape);
    }
}

void RbBodyEasyBox::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasyBox>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* RbBodyEasyBox::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasyBox>();

    RbBodyEasyBox* new_obj = new RbBodyEasyBox();

    return new_obj;
}

// -----------------------------------------------------------------------------

RbBodyEasyConvexHull::RbBodyEasyConvexHull(const std::vector<ChVector3d>& points,
                                           double density,
                                           bool create_visualization,
                                           bool create_collision,
                                           std::shared_ptr<ChContactMaterial> material)
    : RbBody() {
    SetupBody(points, density, create_visualization, create_collision, material);
}

RbBodyEasyConvexHull::RbBodyEasyConvexHull(const std::vector<ChVector3d>& points, double density, std::shared_ptr<ChContactMaterial> material) : RbBody() {
    SetupBody(points, density, true, true, material);
}

void RbBodyEasyConvexHull::SetupBody(const std::vector<ChVector3d>& points,
                                     double density,
                                     bool create_visualization,
                                     bool create_collision,
                                     std::shared_ptr<ChContactMaterial> material) {
    auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();

    bool success = bt_utils::ChConvexHullLibraryWrapper::ComputeHull(points, *vshape->GetMesh());
    if (!success) {
        std::cerr << "ChBodyEasyConvexHull::SetupBody: Unable to create convex hull\n";
        return;
    }

    if (create_visualization) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        AddVisualShape(vshape);
    }

    double mass = 0.0;
    ChVector3d barycenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, barycenter, inertia);
    SetMass(mass * density);
    SetInertia(inertia * density);

    // Translate the convex hull barycenter so that body origin is also barycenter
    for (unsigned int i = 0; i < vshape->GetMesh()->GetCoordsVertices().size(); ++i) {
        vshape->GetMesh()->GetCoordsVertices()[i] -= barycenter;
    }

    if (create_collision) {
        assert(material);
        // Avoid passing to collision the inner points discarded by convex hull
        // processor, so use mesh vertices instead of all argument points
        std::vector<ChVector3d> points_reduced;
        points_reduced.resize(vshape->GetMesh()->GetCoordsVertices().size());
        for (size_t i = 0; i < vshape->GetMesh()->GetCoordsVertices().size(); ++i) {
            points_reduced[i] = vshape->GetMesh()->GetCoordsVertices()[i];
        }

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points_reduced);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }

    m_mesh = vshape->GetMesh();
}

void RbBodyEasyConvexHull::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasyConvexHull>();

    archive_out << CHNVP(m_mesh);
}

void* RbBodyEasyConvexHull::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasyConvexHull>();

    std::shared_ptr<ChTriangleMeshConnected> mesh;
    archive_in >> CHNVP(mesh);

    RbBodyEasyConvexHull* new_obj = new RbBodyEasyConvexHull(mesh);

    return new_obj;
}

// -----------------------------------------------------------------------------

RbBodyEasyConvexHullAuxRef::RbBodyEasyConvexHullAuxRef(const std::vector<ChVector3d>& points,
                                                       double density,
                                                       bool create_visualization,
                                                       bool create_collision,
                                                       std::shared_ptr<ChContactMaterial> material)
    : RbBodyAuxRef() {
    SetupBody(points, density, create_visualization, create_collision, material);
}

RbBodyEasyConvexHullAuxRef::RbBodyEasyConvexHullAuxRef(const std::vector<ChVector3d>& points, double density, std::shared_ptr<ChContactMaterial> material) : RbBodyAuxRef() {
    SetupBody(points, density, true, true, material);
}

void RbBodyEasyConvexHullAuxRef::SetupBody(const std::vector<ChVector3d>& points,
                                           double density,
                                           bool create_visualization,
                                           bool create_collision,
                                           std::shared_ptr<ChContactMaterial> material) {
    auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();

    bool success = bt_utils::ChConvexHullLibraryWrapper::ComputeHull(points, *vshape->GetMesh());
    if (!success) {
        std::cerr << "ChBodyEasyConvexHullAuxRef::SetupBody: Unable to create convex hull\n";
        return;
    }

    if (create_visualization) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        AddVisualShape(vshape);
    }

    double mass = 0.0;
    ChVector3d barycenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, barycenter, inertia);

    ChMatrix33<> principal_inertia_csys;
    ChVectorN<double, 3> principal_I;
    inertia.SelfAdjointEigenSolve(principal_inertia_csys, principal_I);
    if (principal_inertia_csys.determinant() < 0)
        principal_inertia_csys.col(0) *= -1;

    SetMass(mass * density);
    ////SetInertia(inertia * density);
    SetInertiaXX(ChVector3d(principal_I) * density);

    // Set the COG coordinates to barycenter, without displacing the REF reference
    SetFrameCOMToRef(ChFrame<>(barycenter, principal_inertia_csys));

    if (create_collision) {
        assert(material);
        // Avoid passing to collision the inner points discarded by convex hull
        // processor, so use mesh vertices instead of all argument points
        std::vector<ChVector3d> points_reduced;
        points_reduced.resize(vshape->GetMesh()->GetCoordsVertices().size());
        for (size_t i = 0; i < vshape->GetMesh()->GetCoordsVertices().size(); ++i) {
            points_reduced[i] = vshape->GetMesh()->GetCoordsVertices()[i];
        }

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points_reduced);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }

    m_mesh = vshape->GetMesh();
}

void RbBodyEasyConvexHullAuxRef::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasyConvexHullAuxRef>();

    archive_out << CHNVP(m_mesh);
}

void* RbBodyEasyConvexHullAuxRef::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasyConvexHullAuxRef>();

    std::shared_ptr<ChTriangleMeshConnected> mesh;
    archive_in >> CHNVP(mesh);

    RbBodyEasyConvexHullAuxRef* new_obj = new RbBodyEasyConvexHullAuxRef(mesh);

    return new_obj;
}

// -----------------------------------------------------------------------------

RbBodyEasyMesh::RbBodyEasyMesh(const std::string& filename,
                               double density,
                               bool compute_mass,
                               bool create_visualization,
                               bool create_collision,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept)
    : RbBodyAuxRef() {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, compute_mass, create_visualization, create_collision, material, sphere_swept);
}

RbBodyEasyMesh::RbBodyEasyMesh(std::shared_ptr<ChTriangleMeshConnected> mesh,
                               double density,
                               bool compute_mass,
                               bool create_visualization,
                               bool create_collision,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept)
    : RbBodyAuxRef() {
    SetupBody(mesh, "EasyMesh", density, compute_mass, create_visualization, create_collision, material, sphere_swept);
}

RbBodyEasyMesh::RbBodyEasyMesh(const std::string& filename, double density, std::shared_ptr<ChContactMaterial> material, double sphere_swept) : RbBodyAuxRef() {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, true, true, true, material, sphere_swept);
}

RbBodyEasyMesh::RbBodyEasyMesh(std::shared_ptr<ChTriangleMeshConnected> mesh, double density, std::shared_ptr<ChContactMaterial> material, double sphere_swept) : RbBodyAuxRef() {
    SetupBody(mesh, "EasyMesh", density, true, true, true, material, sphere_swept);
}

void RbBodyEasyMesh::SetupBody(std::shared_ptr<ChTriangleMeshConnected> trimesh,
                               const std::string& name,
                               double density,
                               bool compute_mass,
                               bool create_visualization,
                               bool create_collision,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept) {
    if (!trimesh) {
        std::cerr << "ChBodyEasyMesh::SetupBody: Unable to create trimesh\n";
        return;
    }

    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vshape->SetMesh(trimesh);
        vshape->SetName(name);
        AddVisualShape(vshape);
    }

    if (compute_mass) {
        double mass = 0.0;
        ChVector3d cog;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
        SetMass(mass * density);
        SetInertiaXX(density * principal_I);
    }

    if (create_collision) {
        assert(material);
        // coll.model is respect to REF c.sys
        auto cshape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, trimesh, false, false, sphere_swept);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
}

void RbBodyEasyMesh::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasyMesh>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* RbBodyEasyMesh::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasyMesh>();

    RbBodyEasyMesh* new_obj = new RbBodyEasyMesh();

    return new_obj;
}

// -----------------------------------------------------------------------------

RbBodyEasyClusterOfSpheres::RbBodyEasyClusterOfSpheres(const std::vector<ChVector3d>& positions,
                                                       const std::vector<double>& radii,
                                                       double density,
                                                       bool create_visualization,
                                                       bool create_collision,
                                                       std::shared_ptr<ChContactMaterial> material)
    : RbBody() {
    SetupBody(positions, radii, density, create_visualization, create_collision, material);
}

RbBodyEasyClusterOfSpheres::RbBodyEasyClusterOfSpheres(const std::vector<ChVector3d>& positions,
                                                       const std::vector<double>& radii,
                                                       double density,
                                                       std::shared_ptr<ChContactMaterial> material)
    : RbBody() {
    SetupBody(positions, radii, density, true, true, material);
}

void RbBodyEasyClusterOfSpheres::SetupBody(const std::vector<ChVector3d>& positions,
                                           const std::vector<double>& radii,
                                           double density,
                                           bool create_visualization,
                                           bool create_collision,
                                           std::shared_ptr<ChContactMaterial> material) {
    assert(positions.size() == radii.size());

    double totmass = 0;
    ChMatrix33<> totinertia;
    ChVector3d barycenter = VNULL;
    totinertia.setZero();
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = density * (CH_4_3 * CH_PI * std::pow(radii[i], 3));
        barycenter = (barycenter * totmass + positions[i] * sphmass) / (totmass + sphmass);
        totmass += sphmass;
    }
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = density * (CH_4_3 * CH_PI * std::pow(radii[i], 3));
        double sphinertia = (2.0 / 5.0) * sphmass * std::pow(radii[i], 2);

        // Huygens-Steiner parallel axis theorem:
        ChVector3d dist = positions[i] - barycenter;
        totinertia(0, 0) += sphinertia + sphmass * (dist.Length2() - dist.x() * dist.x());
        totinertia(1, 1) += sphinertia + sphmass * (dist.Length2() - dist.y() * dist.y());
        totinertia(2, 2) += sphinertia + sphmass * (dist.Length2() - dist.z() * dist.z());
        totinertia(0, 1) += sphmass * (-dist.x() * dist.y());
        totinertia(0, 2) += sphmass * (-dist.x() * dist.z());
        totinertia(1, 2) += sphmass * (-dist.y() * dist.z());
        totinertia(1, 0) = totinertia(0, 1);
        totinertia(2, 0) = totinertia(0, 2);
        totinertia(2, 1) = totinertia(1, 2);
    }

    SetMass(totmass);
    SetInertia(totinertia);

    // Translate the cluster barycenter so that body origin is also barycenter
    std::vector<ChVector3d> offset_positions = positions;
    for (unsigned int i = 0; i < positions.size(); ++i)
        offset_positions[i] -= barycenter;

    if (create_collision) {
        assert(material);
        auto model = chrono_types::make_shared<ChCollisionModel>();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radii[i]);
            model->AddShape(cshape, ChFrame<>(offset_positions[i], QUNIT));
        }
        AddCollisionModel(model);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto vshape = chrono_types::make_shared<ChVisualShapeSphere>(radii[i]);
            vmodel->AddShape(vshape, ChFrame<>(offset_positions[i]));
        }
        AddVisualModel(vmodel);
    }
}

void RbBodyEasyClusterOfSpheres::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<RbBodyEasyClusterOfSpheres>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* RbBodyEasyClusterOfSpheres::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<RbBodyEasyClusterOfSpheres>();

    RbBodyEasyClusterOfSpheres* new_obj = new RbBodyEasyClusterOfSpheres();

    return new_obj;
}

}  // namespace robodyna::mbd
