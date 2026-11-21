#pragma once
/**
 * PROJECT ARES — Terrain Map Interface & Data Structures.
 *
 * Defines the contract for planetary simulation maps (Earth Proving Ground,
 * photogrammetric Mars site meshes, and HiRISE Digital Terrain Models).
 */

#include <memory>
#include <string>
#include <vector>
#include "physics/Math3D.h"
#include "physics/PlanetaryEnvironment.h"

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
};

} // namespace torquelab
