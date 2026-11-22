#include "navigation/NavGrid.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include "terrain/ProceduralPlanetMap.h"

namespace torquelab {

NavGrid::NavGrid(const NavGridConfig& config)
    : m_config(config) {
    m_width = static_cast<int>(std::round(m_config.worldSizeM / m_config.cellSizeM));
    m_height = m_width;
    m_nodes.resize(static_cast<size_t>(m_width * m_height));
}

bool NavGrid::worldToGrid(double worldX, double worldZ, int& outGridX, int& outGridZ) const {
    const double halfSize = m_config.worldSizeM * 0.5;
    const double gx = (worldX + halfSize) / m_config.cellSizeM;
    const double gz = (worldZ + halfSize) / m_config.cellSizeM;

    if (gx < 0.0 || gx >= static_cast<double>(m_width) ||
        gz < 0.0 || gz >= static_cast<double>(m_height)) {
        outGridX = clamp(static_cast<int>(std::round(gx)), 0, m_width - 1);
        outGridZ = clamp(static_cast<int>(std::round(gz)), 0, m_height - 1);
        return false;
    }

    outGridX = static_cast<int>(std::floor(gx));
    outGridZ = static_cast<int>(std::floor(gz));
    return true;
}

Vec3 NavGrid::gridToWorld(int gridX, int gridZ) const {
    const double halfSize = m_config.worldSizeM * 0.5;
    const double wx = static_cast<double>(gridX) * m_config.cellSizeM - halfSize + m_config.cellSizeM * 0.5;
    const double wz = static_cast<double>(gridZ) * m_config.cellSizeM - halfSize + m_config.cellSizeM * 0.5;

    double wy = 0.0;
    if (gridX >= 0 && gridX < m_width && gridZ >= 0 && gridZ < m_height) {
        wy = m_nodes[static_cast<size_t>(gridZ * m_width + gridX)].elevation;
    }
    return Vec3{wx, wy, wz};
}

const NavNode* NavGrid::getNode(int gridX, int gridZ) const {
    if (gridX < 0 || gridX >= m_width || gridZ < 0 || gridZ >= m_height) {
        return nullptr;
    }
    return &m_nodes[static_cast<size_t>(gridZ * m_width + gridX)];
}

const NavNode* NavGrid::getNodeAtWorld(double worldX, double worldZ) const {
    int gx = 0, gz = 0;
    if (!worldToGrid(worldX, worldZ, gx, gz)) return nullptr;
    return getNode(gx, gz);
}

bool NavGrid::isTraversable(double worldX, double worldZ) const {
    const NavNode* node = getNodeAtWorld(worldX, worldZ);
    return node && (node->traversability == CellTraversability::Traversable);
}

bool NavGrid::buildFromTerrain(const ITerrainMap& terrainMap) {
    m_width = static_cast<int>(std::round(m_config.worldSizeM / m_config.cellSizeM));
    m_height = m_width;
    m_nodes.resize(static_cast<size_t>(m_width * m_height));

    const double halfSize = m_config.worldSizeM * 0.5;

    // 1. First Pass: Elevation, Slope & Initial Obstacle Classification
    for (int gz = 0; gz < m_height; ++gz) {
        const double wz = static_cast<double>(gz) * m_config.cellSizeM - halfSize + m_config.cellSizeM * 0.5;
        for (int gx = 0; gx < m_width; ++gx) {
            const double wx = static_cast<double>(gx) * m_config.cellSizeM - halfSize + m_config.cellSizeM * 0.5;

            NavNode& node = m_nodes[static_cast<size_t>(gz * m_width + gx)];
            node.gridX = gx;
            node.gridZ = gz;

            double elev = 0.0;
            Vec3 norm{0.0, 1.0, 0.0};
            const bool inside = terrainMap.getElevationAt(wx, wz, elev, norm);

            node.elevation = elev;
            node.surfaceNormal = norm;
            node.worldPosition = Vec3{wx, elev, wz};

            if (!inside) {
                node.traversability = CellTraversability::NonTraversable_OutOfBounds;
                node.costMultiplier = 1e6;
                continue;
            }

            const double cosTheta = clamp(norm.y, -1.0, 1.0);
            node.slopeAngleRad = std::acos(cosTheta);

            // Hard slope threshold: > 25 degrees is non-traversable rollover hazard
            if (node.slopeAngleRad > m_config.maxTraversableSlopeRad) {
                node.traversability = CellTraversability::NonTraversable_SteepSlope;
                node.costMultiplier = 1e6;
            } else {
                node.traversability = CellTraversability::Traversable;
                node.costMultiplier = 1.0;
            }
        }
    }

    // 2. Second Pass: Step Height / Roughness Analysis across Adjacent Neighbors
    for (int gz = 0; gz < m_height; ++gz) {
        for (int gx = 0; gx < m_width; ++gx) {
            NavNode& node = m_nodes[static_cast<size_t>(gz * m_width + gx)];
            if (node.traversability != CellTraversability::Traversable) continue;

            double maxStep = 0.0;
            const int dx[4] = {1, -1, 0, 0};
            const int dz[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
                const int nx = gx + dx[k];
                const int nz = gz + dz[k];
                if (nx >= 0 && nx < m_width && nz >= 0 && nz < m_height) {
                    const double diff = std::abs(node.elevation - m_nodes[static_cast<size_t>(nz * m_width + nx)].elevation);
                    if (diff > maxStep) maxStep = diff;
                }
            }

            node.roughnessStepM = maxStep;
            if (maxStep > m_config.maxStepHeightM) {
                node.traversability = CellTraversability::NonTraversable_RoughnessStep;
                node.costMultiplier = 1e6;
            }
        }
    }

    // 3. Mark Solid Rock Obstacles from Procedural Planet Map if available
    const auto* procMap = dynamic_cast<const ProceduralPlanetMap*>(&terrainMap);
    if (procMap && procMap->getData()) {
        for (const auto& rock : procMap->getData()->getRocks()) {
            const double rRadius = std::max(rock.halfExtents.x, rock.halfExtents.z);
            int minGx = 0, minGz = 0, maxGx = 0, maxGz = 0;
            worldToGrid(rock.position.x - rRadius, rock.position.z - rRadius, minGx, minGz);
            worldToGrid(rock.position.x + rRadius, rock.position.z + rRadius, maxGx, maxGz);

            for (int rz = minGz; rz <= maxGz; ++rz) {
                for (int rx = minGx; rx <= maxGx; ++rx) {
                    NavNode* rNode = const_cast<NavNode*>(getNode(rx, rz));
                    if (!rNode) continue;
                    const double d2 = (rNode->worldPosition.x - rock.position.x) * (rNode->worldPosition.x - rock.position.x) +
                                      (rNode->worldPosition.z - rock.position.z) * (rNode->worldPosition.z - rock.position.z);
                    if (d2 <= rRadius * rRadius) {
                        rNode->traversability = CellTraversability::NonTraversable_ObstacleCollision;
                        rNode->costMultiplier = 1e6;
                    }
                }
            }
        }
    }

    // 4. Compute Distance Transform and Dilate Obstacles by Rover Footprint Radius
    computeObstacleDistanceTransform();

    return true;
}

void NavGrid::computeObstacleDistanceTransform() {
    // Breadth-First Multi-Source Flood Fill for exact Euclidean obstacle distance transform
    struct BFSNode {
        int gx;
        int gz;
        double dist;
    };

    std::queue<std::pair<int, int>> q;
    for (int gz = 0; gz < m_height; ++gz) {
        for (int gx = 0; gx < m_width; ++gx) {
            NavNode& node = m_nodes[static_cast<size_t>(gz * m_width + gx)];
            if (node.traversability != CellTraversability::Traversable) {
                node.obstacleDistanceM = 0.0;
                q.push({gx, gz});
            } else {
                node.obstacleDistanceM = 999.0;
            }
        }
    }

    // 8-neighbor offsets
    const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dz[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    const double dd[8] = {
        m_config.cellSizeM, m_config.cellSizeM, m_config.cellSizeM, m_config.cellSizeM,
        m_config.cellSizeM * 1.41421356, m_config.cellSizeM * 1.41421356,
        m_config.cellSizeM * 1.41421356, m_config.cellSizeM * 1.41421356
    };

    while (!q.empty()) {
        const auto [cx, cz] = q.front();
        q.pop();

        const double curDist = m_nodes[static_cast<size_t>(cz * m_width + cx)].obstacleDistanceM;
        // Optimization: stop expanding beyond 12 meters of obstacle proximity
        if (curDist >= 12.0) continue;

        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dx[k];
            const int nz = cz + dz[k];
            if (nx >= 0 && nx < m_width && nz >= 0 && nz < m_height) {
                NavNode& neighbor = m_nodes[static_cast<size_t>(nz * m_width + nx)];
                const double newDist = curDist + dd[k];
                if (newDist < neighbor.obstacleDistanceM) {
                    neighbor.obstacleDistanceM = newDist;
                    q.push({nx, nz});
                }
            }
        }
    }

    // 5. Footprint Inflation & Multi-Criteria Cost Evaluation
    const double footprintR = m_config.roverFootprintRadiusM;
    for (int gz = 0; gz < m_height; ++gz) {
        for (int gx = 0; gx < m_width; ++gx) {
            NavNode& node = m_nodes[static_cast<size_t>(gz * m_width + gx)];
            if (node.traversability != CellTraversability::Traversable) continue;

            // If cell center is inside the dilated obstacle footprint, rover would collide!
            if (node.obstacleDistanceM < footprintR) {
                node.traversability = CellTraversability::NonTraversable_ObstacleCollision;
                node.costMultiplier = 1e6;
                continue;
            }

            // Continuous cost multiplier >= 1.0 for A* path optimization
            const double slopeFactor = std::pow(node.slopeAngleRad / m_config.maxTraversableSlopeRad, 2.0);
            const double roughFactor = node.roughnessStepM / m_config.maxStepHeightM;
            // Clear buffer between obstacle distance and rover footprint
            const double clearDist = std::max(0.01, node.obstacleDistanceM - footprintR);
            const double obsProximityCost = 1.0 / (clearDist * clearDist + 0.5);

            node.costMultiplier = 1.0 +
                m_config.weightSlope * slopeFactor +
                m_config.weightRoughness * roughFactor +
                m_config.weightObstacleProximity * obsProximityCost;
        }
    }
}

bool NavGrid::isFootprintSafe(double worldX, double worldZ, double headingRad) const {
    const double cosYaw = std::cos(headingRad);
    const double sinYaw = std::sin(headingRad);

    // 6 Wheel Positions for Perseverance (Track=2.65m, Wheelbase=2.26m)
    // Local: FL(+1.06, +1.10), FR(-1.06, +1.10), ML(+1.18, -0.09), MR(-1.18, -0.09), RL(+1.06, -1.16), RR(-1.06, -1.16)
    const Vec2 wheelLocals[6] = {
        Vec2{ 1.06,  1.10}, Vec2{-1.06,  1.10},
        Vec2{ 1.18, -0.09}, Vec2{-1.18, -0.09},
        Vec2{ 1.06, -1.16}, Vec2{-1.06, -1.16}
    };

    for (int i = 0; i < 6; ++i) {
        const double lx = wheelLocals[i].x;
        const double lz = wheelLocals[i].y;
        // World coordinates of wheel contact footprint
        const double wx = worldX + (lx * cosYaw + lz * sinYaw);
        const double wz = worldZ + (-lx * sinYaw + lz * cosYaw);

        const NavNode* node = getNodeAtWorld(wx, wz);
        if (!node || node->traversability != CellTraversability::Traversable) {
            return false;
        }
    }
    return true;
}

std::vector<const NavNode*> NavGrid::getNeighbors(int gridX, int gridZ) const {
    std::vector<const NavNode*> result;
    result.reserve(8);

    const int dx[8] = { 0,  1,  0, -1,  1,  1, -1, -1};
    const int dz[8] = { 1,  0, -1,  0,  1, -1,  1, -1};

    for (int k = 0; k < 8; ++k) {
        const int nx = gridX + dx[k];
        const int nz = gridZ + dz[k];

        if (nx < 0 || nx >= m_width || nz < 0 || nz >= m_height) continue;

        const NavNode* neighbor = &m_nodes[static_cast<size_t>(nz * m_width + nx)];
        if (neighbor->traversability != CellTraversability::Traversable) continue;

        // Diagonal corner-cutting check: do not cut diagonally between two blocking obstacles
        if (k >= 4) {
            const NavNode* side1 = getNode(gridX + dx[k], gridZ);
            const NavNode* side2 = getNode(gridX, gridZ + dz[k]);
            if ((side1 && side1->traversability != CellTraversability::Traversable) ||
                (side2 && side2->traversability != CellTraversability::Traversable)) {
                continue;
            }
        }

        result.push_back(neighbor);
    }
    return result;
}

double NavGrid::getMovementCost(const NavNode& from, const NavNode& to) const {
    const double dx = static_cast<double>(to.gridX - from.gridX);
    const double dz = static_cast<double>(to.gridZ - from.gridZ);
    const double baseDist = std::sqrt(dx * dx + dz * dz) * m_config.cellSizeM;

    // Symmetric arithmetic mean of cell traversal cost multipliers
    const double meanMultiplier = 0.5 * (from.costMultiplier + to.costMultiplier);
    return baseDist * meanMultiplier;
}

} // namespace torquelab
