#pragma once
/**
 * PROJECT ARES — Procedural Planetary Map Implementation.
 *
 * Implements the ITerrainMap interface for dynamically generated procedural Mars,
 * Moon, and scenario test worlds, unifying Bullet physics collision, OpenGL rendering,
 * spatial chunking, and rock obstacles.
 */

#include <memory>
#include <string>
#include <vector>
#include "terrain/ProceduralTerrainGenerator.h"
#include "terrain/TerrainMap.h"

namespace torquelab {

class ProceduralPlanetMap : public ITerrainMap {
public:
    explicit ProceduralPlanetMap(
        const std::string& id,
        const std::string& displayName,
        const std::string& description,
        const ProceduralTerrainConfig& config
    );

    ~ProceduralPlanetMap() override = default;

    std::string getId() const override { return m_id; }
    std::string getDisplayName() const override { return m_displayName; }
    std::string getDescription() const override { return m_description; }
    TerrainResolutionLevel getResolutionLevel() const override {
        return TerrainResolutionLevel::Level1_RegionalDTM;
    }
    PlanetaryEnvironment getEnvironment() const override {
        return m_config.environment;
    }
    std::vector<TerrainSpawnPoint> getSpawnPoints() const override;

    bool buildPhysics(PhysicsWorld& world) override;
    bool buildGraphics(TrackRendererGL& renderer) override;

    bool getElevationAt(double worldX, double worldZ, double& outElevation, Vec3& outNormal) const override;
    SurfaceType getSurfaceMaterialAt(double worldX, double worldZ) const override;
    bool isPositionInsidePlayableTerrain(double worldX, double worldZ) const override;
    TerrainContactInfo getTerrainContactAt(const Vec3& worldPos, double probeRadius = 0.0) const override;
    bool queryRayContact(const Vec3& rayOrigin, const Vec3& rayDir, double maxDistance, TerrainContactInfo& outContact) const override;

    /** Access to the underlying authoritative procedural data container */
    std::shared_ptr<ProceduralTerrainData> getData() const { return m_data; }
    const ProceduralTerrainConfig& getConfig() const { return m_config; }

    /** Regenerates the procedural world with a new random seed or modified config */
    void regenerate(const ProceduralTerrainConfig& newConfig);

private:
    void ensureGenerated();

    std::string m_id;
    std::string m_displayName;
    std::string m_description;
    ProceduralTerrainConfig m_config;
    std::shared_ptr<ProceduralTerrainData> m_data;

    // Bullet physics references
    std::vector<int> m_colliderHandles;
};

} // namespace torquelab
