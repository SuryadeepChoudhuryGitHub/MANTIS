#pragma once
/**
 * PROJECT ARES — Central Terrain Manager & Multi-Map Registry.
 *
 * Manages registration, discovery, and runtime hot-swapping between planetary
 * environments (Earth Proving Ground, Mars Site Alpha, Mars Site Beta, and HiRISE DTM).
 */

#include <memory>
#include <string>
#include <vector>
#include "terrain/TerrainMap.h"

namespace torquelab {

class PhysicsWorld;
class TrackRendererGL;
class SkyRendererGL;

class TerrainManager {
public:
    TerrainManager() = default;
    ~TerrainManager() = default;

    TerrainManager(const TerrainManager&) = delete;
    TerrainManager& operator=(const TerrainManager&) = delete;

    /**
     * Registers all available maps (Earth Proving Ground, Mars sites, HiRISE DTM).
     */
    void init();

    /**
     * Returns all registered terrain maps.
     */
    const std::vector<std::shared_ptr<ITerrainMap>>& getAvailableMaps() const { return m_maps; }

    /**
     * Returns the currently active terrain map.
     */
    std::shared_ptr<ITerrainMap> getActiveMap() const;

    /**
     * Returns the unique identifier of the active map.
     */
    std::string getActiveMapId() const;

    /**
     * Switches the active terrain map at runtime:
     * 1. Clears Bullet static colliders and loads the new map's collision shape.
     * 2. Clears rendering items and builds the new visual meshes/materials.
     * 3. Synchronizes world gravity, air density, and atmospheric sky colors.
     * 4. Teleports the vehicle to the map's designated primary spawn point.
     */
    bool switchMap(
        const std::string& mapId,
        PhysicsWorld& world,
        TrackRendererGL& trackRenderer,
        SkyRendererGL& skyRenderer
    );

private:
    std::vector<std::shared_ptr<ITerrainMap>> m_maps;
    size_t m_activeMapIndex{0};
};

} // namespace torquelab
