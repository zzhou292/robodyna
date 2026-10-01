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
// Robodyna adaptation: canonical body-family ownership; inherited behavior and identities retained.
// =============================================================================
//
// Classes for creating easy-to-use bodies that optionally include contact and
// visualization shapes.
//
// =============================================================================

#ifndef ROBODYNA_MBD_RBBODYEASY_H
#define ROBODYNA_MBD_RBBODYEASY_H

#include "chrono/core/ChApiCE.h"
#include "robodyna/mbd/RbBody.h"
#include "robodyna/mbd/RbBodyAuxRef.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace robodyna::mbd {

/// Create rigid bodies with a spherical shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
class ChApi RbBodyEasySphere : public RbBody {
  public:
    /// Create a rigid body with optional sphere visualization and/or collision shape.
    /// The sphere is created at the center of mass. Mass and inertia are set automatically depending on density.
    RbBodyEasySphere(double radius,                                         ///< radius of the sphere
                     double density,                                        ///< density of the body
                     bool create_visualization = true,                      ///< create visualization asset
                     bool create_collision = false,                         ///< enable collision
                     std::shared_ptr<chrono::ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a sphere visualization and collision shape.
    /// The sphere is created at the center of mass. Mass and inertia are set automatically depending on density.
    RbBodyEasySphere(double radius,                               ///< radius of the sphere
                     double density,                              ///< density of the body
                     std::shared_ptr<chrono::ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(double radius, double density, bool create_visualization, bool create_collision, std::shared_ptr<chrono::ChContactMaterial> material);

    RbBodyEasySphere() {}
};

/// Create rigid bodies with an ellipsoid shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
class ChApi RbBodyEasyEllipsoid : public RbBody {
  public:
    /// Create a rigid body with optional ellipsoid visualization and/or collision shape.
    /// The ellipsoid is created at the center of mass. Mass and inertia are set automatically depending on density.
    RbBodyEasyEllipsoid(const chrono::ChVector3d& axes,                                ///< ellipsoid axis lengths
                        double density,                                        ///< density of the body
                        bool create_visualization = true,                      ///< create visualization asset
                        bool create_collision = false,                         ///< enable collision
                        std::shared_ptr<chrono::ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with an ellipsoid visualization and collision shape.
    /// The ellipsoid is created at the center of mass. Mass and inertia are set automatically depending on density.
    RbBodyEasyEllipsoid(const chrono::ChVector3d& axes,                      ///< ellipsoid axis lengths
                        double density,                              ///< density of the body
                        std::shared_ptr<chrono::ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(const chrono::ChVector3d& axes, double density, bool create_visualization, bool create_collision, std::shared_ptr<chrono::ChContactMaterial> material);

    RbBodyEasyEllipsoid() {}
};

/// Create rigid bodies with a cylinder shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
class ChApi RbBodyEasyCylinder : public RbBody {
  public:
    /// Create a rigid body with optional cylinder visualization and/or collision shape.
    /// The cylinder is created along the specified axis and centered at the center of mass.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyCylinder(chrono::ChAxis direction,                                      ///< cylinder direction
                       double radius,                                         ///< radius of the cylinder
                       double height,                                         ///< height of the cylinder
                       double density,                                        ///< density of the body
                       bool create_visualization = true,                      ///< create visualization asset
                       bool create_collision = false,                         ///< enable collision
                       std::shared_ptr<chrono::ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a cylinder visualization and collision shape.
    /// The cylinder is created along the specified axis and centered at the center of mass.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyCylinder(chrono::ChAxis direction,                            ///< cylinder direction
                       double radius,                               ///< radius of the cylinder
                       double height,                               ///< height of the cylinder
                       double density,                              ///< density of the body
                       std::shared_ptr<chrono::ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(chrono::ChAxis direction, double radius, double height, double density, bool create_visualization, bool create_collision, std::shared_ptr<chrono::ChContactMaterial> material);

    RbBodyEasyCylinder() {}
};

/// Create rigid bodies with a box shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
class ChApi RbBodyEasyBox : public RbBody {
  public:
    /// Create a rigid body with optional box visualization and/or collision shape.
    /// The box is created at the center of mass. Mass and inertia are set automatically depending on density.
    RbBodyEasyBox(double Xsize,                                          ///< size along the X dimension
                  double Ysize,                                          ///< size along the Y dimension
                  double Zsize,                                          ///< size along the Z dimension
                  double density,                                        ///< density of the body
                  bool create_visualization = true,                      ///< create visualization asset
                  bool create_collision = false,                         ///< enable collision
                  std::shared_ptr<chrono::ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a box visualization and collision shape.
    /// The box is created at the center of mass. Mass and inertia are set automatically depending on density.
    RbBodyEasyBox(double Xsize,                                ///< size along the X dimension
                  double Ysize,                                ///< size along the Y dimension
                  double Zsize,                                ///< size along the Z dimension
                  double density,                              ///< density of the body
                  std::shared_ptr<chrono::ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(double Xsize, double Ysize, double Zsize, double density, bool create_visualization, bool create_collision, std::shared_ptr<chrono::ChContactMaterial> material);

    RbBodyEasyBox() {}
};

/// Create rigid bodies with a convex hull shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
class ChApi RbBodyEasyConvexHull : public RbBody {
  public:
    /// Create a rigid body with optional convex hull visualization and/or collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame.
    /// Mass and inertia are set automatically depending on density.
    /// NB: The convex hull vertices are translated so that the barycenter coincides with the center of mass.
    RbBodyEasyConvexHull(const std::vector<chrono::ChVector3d>& points,                 ///< points of the convex hull
                         double density,                                        ///< density of the body
                         bool create_visualization = true,                      ///< create visualization asset
                         bool create_collision = false,                         ///< enable collision
                         std::shared_ptr<chrono::ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a convex hull visualization and collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame.
    /// Mass and inertia are set automatically depending on density.
    /// NB: The convex hull vertices are translated so that the barycenter coincides with the center of mass.
    RbBodyEasyConvexHull(const std::vector<chrono::ChVector3d>& points,       ///< points of the convex hull
                         double density,                              ///< density of the body
                         std::shared_ptr<chrono::ChContactMaterial> material  ///< surface contact material
    );

    /// Get body mesh.
    std::shared_ptr<chrono::ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(const std::vector<chrono::ChVector3d>& points, double density, bool create_visualization, bool create_collision, std::shared_ptr<chrono::ChContactMaterial> material);

    std::shared_ptr<chrono::ChTriangleMeshConnected> m_mesh;

    RbBodyEasyConvexHull(std::shared_ptr<chrono::ChTriangleMeshConnected> mesh) : m_mesh(mesh) {}
};

/// Create rigid body with a convex hull shape, with a reference frame distinct from the centroidal frame.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
class ChApi RbBodyEasyConvexHullAuxRef : public RbBodyAuxRef {
  public:
    /// Create a ChBodyAuxRef with optional convex hull visualization and/or collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame.
    /// Mass and inertia are set automatically depending on density.
    /// The center of mass is set at the barycenter.
    RbBodyEasyConvexHullAuxRef(const std::vector<chrono::ChVector3d>& points,                 ///< convex hull points
                               double density,                                        ///< density of the body
                               bool create_visualization = true,                      ///< create visualization asset
                               bool create_collision = false,                         ///< enable collision
                               std::shared_ptr<chrono::ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a ChBodyAuxRef with a convex hull visualization and collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame.
    /// Mass and inertia are set automatically depending on density.
    /// The center of mass is set at the barycenter.
    RbBodyEasyConvexHullAuxRef(const std::vector<chrono::ChVector3d>& points,       ///< convex hull points
                               double density,                              ///< density of the body
                               std::shared_ptr<chrono::ChContactMaterial> material  ///< surface contact material
    );

    /// Get body mesh.
    std::shared_ptr<chrono::ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(const std::vector<chrono::ChVector3d>& points, double density, bool create_visualization, bool create_collision, std::shared_ptr<chrono::ChContactMaterial> material);

    std::shared_ptr<chrono::ChTriangleMeshConnected> m_mesh;

    RbBodyEasyConvexHullAuxRef(std::shared_ptr<chrono::ChTriangleMeshConnected> mesh) : m_mesh(mesh) {}
};

/// Create rigid bodies with a mesh shape, with a reference frame distinct from the centroidal frame.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
class ChApi RbBodyEasyMesh : public RbBodyAuxRef {
  public:
    /// Create a ChBodyAuxRef with optional mesh visualization and/or collision shape.
    /// The mesh is assumed to be provided in a Wavefront OBJ file and defined with respect to the body reference frame.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyMesh(const std::string& filename,                            ///< name of the Wavefront OBJ file
                   double density,                                         ///< density of the body
                   bool compute_mass = true,                               ///< automatic evaluation of inertia properties
                   bool create_visualization = true,                       ///< create visualization asset
                   bool create_collision = false,                          ///< enable collision
                   std::shared_ptr<chrono::ChContactMaterial> material = nullptr,  ///< surface contact material
                   double sphere_swept = 0.001                             ///< thickness (collision detection robustness)
    );

    /// Create a ChBodyAuxRef with optional mesh visualization and/or collision shape.
    /// The mesh is defined with respect to the body reference frame.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyMesh(std::shared_ptr<chrono::ChTriangleMeshConnected> mesh,          ///< triangular mesh
                   double density,                                         ///< density of the body
                   bool compute_mass = true,                               ///< automatic evaluation of inertia properties
                   bool create_visualization = true,                       ///< create visualization asset
                   bool create_collision = false,                          ///< enable collision
                   std::shared_ptr<chrono::ChContactMaterial> material = nullptr,  ///< surface contact material
                   double sphere_swept = 0.001                             ///< thickness (collision detection robustness)
    );

    /// Create a ChBodyAuxRef with a mesh visualization and collision shape.
    /// The mesh is assumed to be provided in a Wavefront OBJ file and defined with respect to the body reference frame.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyMesh(const std::string& filename,                  ///< name of the Wavefront OBJ file
                   double density,                               ///< density of the body
                   std::shared_ptr<chrono::ChContactMaterial> material,  ///< surface contact material
                   double sphere_swept                           ///< thickness (collision detection robustness)
    );

    /// Create a ChBodyAuxRef with a convex hull visualization and collision shape.
    /// The mesh is defined with respect to the body reference frame.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyMesh(std::shared_ptr<chrono::ChTriangleMeshConnected> mesh,  ///< triangular mesh
                   double density,                                 ///< density of the body
                   std::shared_ptr<chrono::ChContactMaterial> material,    ///< surface contact material
                   double sphere_swept                             ///< thickness (collision detection robustness)
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(std::shared_ptr<chrono::ChTriangleMeshConnected> trimesh,
                   const std::string& name,
                   double density,
                   bool compute_mass,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<chrono::ChContactMaterial> material,
                   double sphere_swept);

    RbBodyEasyMesh() {}
};

/// Create rigid bodies with a shape made of a cluster of spheres.
/// Optionally sets the visualization and/or collision geometry and automatically calculates inertia properties based on the geometry.
/// Note that mass and inertia are computed as if spheres are not intersecting.
/// If a more precise mass/inertia estimation is needed when spheres are intersecting, change mass and inertia after creation using more advanced formulas.
class ChApi RbBodyEasyClusterOfSpheres : public RbBody {
  public:
    /// Create a rigid body with optional sphere cluster mesh visualization and/or collision shapes.
    /// The cluster of spheres will be displaced so that their center of mass corresponds to the origin of the body.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyClusterOfSpheres(const std::vector<chrono::ChVector3d>& positions,              ///< position of the spheres
                               const std::vector<double>& radii,                      ///< sphere radius
                               double density,                                        ///< density of the body
                               bool create_visualization = true,                      ///< create visualization asset
                               bool create_collision = false,                         ///< enable collision
                               std::shared_ptr<chrono::ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a ChBody with a sphere cluster mesh visualization and collision shapes.
    /// The cluster of spheres will be displaced so that their center of mass corresponds to the origin of the ChBody.
    /// Mass and inertia are set automatically depending on density.
    RbBodyEasyClusterOfSpheres(const std::vector<chrono::ChVector3d>& positions,    ///< position of the spheres
                               const std::vector<double>& radii,            ///< sphere radius
                               double density,                              ///< density of the body
                               std::shared_ptr<chrono::ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(chrono::ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(chrono::ChArchiveIn& archive_in);

  private:
    void SetupBody(const std::vector<chrono::ChVector3d>& positions,
                   const std::vector<double>& radii,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<chrono::ChContactMaterial> material);

    RbBodyEasyClusterOfSpheres() {}
};

}  // namespace robodyna::mbd

#endif
