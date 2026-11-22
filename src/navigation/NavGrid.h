#pragma once
/**
 * PROJECT ARES — Autonomous Planetary Navigation Grid & Traversability Analysis.
 *
 * Implements a discrete 2D spatial grid over planetary terrain, evaluating:
 * - Slope inclination and rollover risk (pitch and roll limits).
 * - Surface roughness and belly ground clearance.
 * - Solid rock obstacle dilation using actual NASA Perseverance rover dimensions
 *   (track width = 2.65m, wheelbase = 2.26m, overall length = 3.1m, width = 2.7m).
 * - Multi-criteria traversability cost formulation for A* pathfinding.
 */

#include <cmath>
#include <memory>
#include <vector>
#include "physics/Math3D.h"
#include "terrain/TerrainMap.h"

namespace torquelab {

enum class CellTraversability {
    Traversable,
    NonTraversable_SteepSlope,
    NonTraversable_ObstacleCollision,
    NonTraversable_RoughnessStep,
    NonTraversable_OutOfBounds
};

struct NavNode {
    int gridX{0};
    int gridZ{0};
    Vec3 worldPosition{0.0, 0.0, 0.0};
    Vec3 surfaceNormal{0.0, 1.0, 0.0};

    double elevation{0.0};
    double slopeAngleRad{0.0};       // Inclination relative to horizontal
    double roughnessStepM{0.0};      // Height disparity across adjacent cells
    double obstacleDistanceM{999.0}; // Distance to nearest solid rock / non-traversable obstacle

    CellTraversability traversability{CellTraversability::Traversable};
    double costMultiplier{1.0};      // Traversability cost weight >= 1.0
};

struct NavGridConfig {
    double cellSizeM{1.0};           // Metric spatial resolution (1.0m per grid cell)
    double worldSizeM{512.0};        // Total world dimension covered by grid

    // Perseverance Rover Footprint & Mechanical Limits
    double roverFootprintRadiusM{1.65};  // Circumscribed radius for 2.65m track x 2.26m wheelbase
    double maxTraversableSlopeRad{0.4363}; // 25.0 degrees maximum safe incline (rollover limit)
    double maxStepHeightM{0.26};         // Wheel radius is 0.2625m, clearance is 0.35m

    // Weighted Cost Coefficients
    double weightSlope{4.0};         // Penalty exponent for slope
    double weightRoughness{5.0};     // Penalty for local height variation
    double weightObstacleProximity{3.5}; // Cost penalty for hugging rocks
};

class NavGrid {
public:
    explicit NavGrid(const NavGridConfig& config = NavGridConfig{});

    /** Rebuilds the navigation grid by sampling authoritative elevation and obstacles from terrainMap */
    bool buildFromTerrain(const ITerrainMap& terrainMap);

    const NavGridConfig& getConfig() const { return m_config; }
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    double getCellSize() const { return m_config.cellSizeM; }
    double getWorldSize() const { return m_config.worldSizeM; }
    double getWorldWidth() const { return m_config.worldSizeM; }
    double getWorldDepth() const { return m_config.worldSizeM; }
    double getElevationAt(double worldX, double worldZ) const {
        const NavNode* node = getNodeAtWorld(worldX, worldZ);
        return node ? node->elevation : 0.0;
    }

    /** Coordinate conversion */
    bool worldToGrid(double worldX, double worldZ, int& outGridX, int& outGridZ) const;
    Vec3 gridToWorld(int gridX, int gridZ) const;

    /** Safe node query */
    const NavNode* getNode(int gridX, int gridZ) const;
    const NavNode* getNodeAtWorld(double worldX, double worldZ) const;

    /** Checks if a specific world position is safe and traversable */
    bool isTraversable(double worldX, double worldZ) const;

    /** Footprint collision check: verifies if rover body can safely occupy pose at (worldX, worldZ) */
    bool isFootprintSafe(double worldX, double worldZ, double headingRad) const;

    /** Returns all valid, traversable 8-connected neighbors of a node */
    std::vector<const NavNode*> getNeighbors(int gridX, int gridZ) const;

    /** Calculates transition movement cost from node A to node B */
    double getMovementCost(const NavNode& from, const NavNode& to) const;

private:
    void computeObstacleDistanceTransform();

    NavGridConfig m_config;
    int m_width{512};
    int m_height{512};
    std::vector<NavNode> m_nodes;
};

} // namespace torquelab
