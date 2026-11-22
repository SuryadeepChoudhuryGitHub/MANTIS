#pragma once
/**
 * PROJECT MANTIS — Terrain Map Interface & Data Structures.
 *
 * Defines the contract for planetary simulation maps (Earth Proving Ground,
 * photogrammetric Mars site meshes, and HiRISE Digital Terrain Models).
 */

#include <memory>
#include <string>
#include <vector>
#include "physics/Math3D.h"
#include "physics/PlanetaryEnvironment.h"
#include "physics/SurfaceSystem.h"

namespace torquelab {

class PhysicsWorld;
class TrackRendererGL;

struct TerrainSpawnPoint {
    std::string id;
    std::string label;
    std::string description;
    Vec3 position{0.0, 0.55, 0.0};
    double yawDeg{0.0};
};

struct TerrainContactInfo {
    bool hasContact{false};
    double elevation{0.0};
    Vec3 surfaceNormal{0.0, 1.0, 0.0};
    Vec3 contactPointWorld{0.0, 0.0, 0.0};
    SurfaceType surfaceType{SurfaceType::MarsRegolith};
    double penetrationDepth{0.0};
    bool isInsidePlayableBounds{true};
    double slopeAngleRad{0.0};
};

enum class TerrainResolutionLevel {
    Level1_RegionalDTM,  // Measured orbital altimetry (HiRISE DTMs, 1.0 m/px)
    Level2_LocalMesh,    // Photogrammetric 3D surface mesh (Navcam/Mastcam, cm-scale)
    Level3_WheelContact  // Terramechanics soil contact (sub-cm sinkage/grousers)
};

class ITerrainMap {
public:
    virtual ~ITerrainMap() = default;

    virtual std::string getId() const = 0;
    virtual std::string getDisplayName() const = 0;
    virtual std::string getDescription() const = 0;
    virtual TerrainResolutionLevel getResolutionLevel() const = 0;
    virtual PlanetaryEnvironment getEnvironment() const = 0;
    virtual std::vector<TerrainSpawnPoint> getSpawnPoints() const = 0;

    /**
     * Constructs the physical collision shapes in Bullet Physics (PhysicsWorld).
     */
    virtual bool buildPhysics(PhysicsWorld& world) = 0;

    /**
     * Builds the visual rendering meshes and PBR materials in TrackRendererGL.
     */
    virtual bool buildGraphics(TrackRendererGL& renderer) = 0;

    /**
     * Authoritative elevation and surface normal query at world coordinates (worldX, worldZ).
     * Returns true if coordinates are within valid terrain bounds; outElevation and outNormal are set.
     */
    virtual bool getElevationAt(double worldX, double worldZ, double& outElevation, Vec3& outNormal) const = 0;

    /**
     * Returns surface material/regolith classification at (worldX, worldZ).
     */
    virtual SurfaceType getSurfaceMaterialAt(double worldX, double worldZ) const = 0;

    /**
     * Checks whether world coordinates (worldX, worldZ) are inside the active playable terrain area.
     */
    virtual bool isPositionInsidePlayableTerrain(double worldX, double worldZ) const = 0;

    /**
     * Evaluates volumetric or point wheel-ground contact at world position.
     */
    virtual TerrainContactInfo getTerrainContactAt(const Vec3& worldPos, double probeRadius = 0.0) const = 0;

    /**
     * Queries authoritative terrain surface along a line ray from rayOrigin in direction rayDir.
     */
    virtual bool queryRayContact(const Vec3& rayOrigin, const Vec3& rayDir, double maxDistance, TerrainContactInfo& outContact) const = 0;
};

} // namespace torquelab
