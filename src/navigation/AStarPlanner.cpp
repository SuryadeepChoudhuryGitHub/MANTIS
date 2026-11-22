#include "navigation/AStarPlanner.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>

namespace torquelab {

AStarPlanner::AStarPlanner(const PlannerConfig& config)
    : m_config(config) {}

double AStarPlanner::computeHeuristic(const NavNode& current, const NavNode& goal, double weight) {
    const double dx = static_cast<double>(goal.gridX - current.gridX);
    const double dz = static_cast<double>(goal.gridZ - current.gridZ);
    // Admissible Euclidean distance in meters
    const double euclideanMeters = std::sqrt(dx * dx + dz * dz);
    return euclideanMeters * weight;
}

PlanResult AStarPlanner::plan(
    const NavGrid& grid,
    const Vec3& startWorld,
    const Vec3& goalWorld
) const {
    const auto startTime = std::chrono::high_resolution_clock::now();
    PlanResult result;

    int startGx = 0, startGz = 0;
    int goalGx = 0, goalGz = 0;

    if (!grid.worldToGrid(startWorld.x, startWorld.z, startGx, startGz)) {
        result.failureReason = "Start position is outside navigation grid bounds";
        return result;
    }
    if (!grid.worldToGrid(goalWorld.x, goalWorld.z, goalGx, goalGz)) {
        result.failureReason = "Goal destination is outside navigation grid bounds";
        return result;
    }

    const NavNode* startNode = grid.getNode(startGx, startGz);
    const NavNode* goalNode  = grid.getNode(goalGx, goalGz);

    if (!startNode || startNode->traversability != CellTraversability::Traversable) {
        // If rover currently sits on border cell, try 8-neighbor fallback
        for (const auto* n : grid.getNeighbors(startGx, startGz)) {
            if (n && n->traversability == CellTraversability::Traversable) {
                startNode = n;
                startGx = n->gridX;
                startGz = n->gridZ;
                break;
            }
        }
    }

    if (!startNode || startNode->traversability != CellTraversability::Traversable) {
        result.failureReason = "Rover start location is non-traversable or inside an obstacle";
        return result;
    }

    // If goal node is inside an obstacle or steep slope, find closest valid traversable neighbor
    if (!goalNode || goalNode->traversability != CellTraversability::Traversable) {
        double bestDist = 1e9;
        const NavNode* alternativeGoal = nullptr;
        constexpr int kSearchR = 12;
        for (int rz = -kSearchR; rz <= kSearchR; ++rz) {
            for (int rx = -kSearchR; rx <= kSearchR; ++rx) {
                const NavNode* cand = grid.getNode(goalGx + rx, goalGz + rz);
                if (cand && cand->traversability == CellTraversability::Traversable) {
                    const double d2 = static_cast<double>(rx * rx + rz * rz);
                    if (d2 < bestDist) {
                        bestDist = d2;
                        alternativeGoal = cand;
                    }
                }
            }
        }
        if (alternativeGoal) {
            goalNode = alternativeGoal;
            goalGx = alternativeGoal->gridX;
            goalGz = alternativeGoal->gridZ;
        } else {
            result.failureReason = "Selected goal destination is unreachable (inside deep obstacle/cliff)";
            return result;
        }
    }

    // Trivial path (already at goal)
    if (startGx == goalGx && startGz == goalGz) {
        PathWaypoint wp;
        wp.positionWorld = goalNode->worldPosition;
        wp.surfaceNormal = goalNode->surfaceNormal;
        wp.targetSpeedMPerSec = 0.0;
        wp.slopeAngleRad = goalNode->slopeAngleRad;
        result.waypoints.push_back(wp);
        result.rawGridPath.push_back(wp.positionWorld);
        result.success = true;
        return result;
    }

    const int W = grid.getWidth();
    const int H = grid.getHeight();
    const size_t totalCells = static_cast<size_t>(W * H);

    struct NodeEntry {
        int index;
        double fCost;
        bool operator>(const NodeEntry& other) const {
            return fCost > other.fCost;
        }
    };

    std::priority_queue<NodeEntry, std::vector<NodeEntry>, std::greater<NodeEntry>> openSet;
    std::vector<double> gCost(totalCells, 1e12);
    std::vector<int> parent(totalCells, -1);
    std::vector<bool> closed(totalCells, false);

    const int startIndex = startGz * W + startGx;
    const int goalIndex  = goalGz * W + goalGx;

    gCost[static_cast<size_t>(startIndex)] = 0.0;
    const double hStart = m_config.useDijkstra ? 0.0 : computeHeuristic(*startNode, *goalNode, m_config.heuristicWeight);
    openSet.push(NodeEntry{startIndex, hStart});

    int expanded = 0;
    bool found = false;

    while (!openSet.empty() && expanded < m_config.maxExpandedNodes) {
        const NodeEntry current = openSet.top();
        openSet.pop();

        const int curIdx = current.index;
        if (closed[static_cast<size_t>(curIdx)]) continue;
        closed[static_cast<size_t>(curIdx)] = true;
        ++expanded;

        if (curIdx == goalIndex) {
            found = true;
            break;
        }

        const int cx = curIdx % W;
        const int cz = curIdx / W;
        const NavNode* curNode = grid.getNode(cx, cz);
        if (!curNode) continue;

        const auto neighbors = grid.getNeighbors(cx, cz);
        for (const NavNode* neighbor : neighbors) {
            const int nIdx = neighbor->gridZ * W + neighbor->gridX;
            if (closed[static_cast<size_t>(nIdx)]) continue;

            const double transitionCost = grid.getMovementCost(*curNode, *neighbor);
            const double tentativeG = gCost[static_cast<size_t>(curIdx)] + transitionCost;

            if (tentativeG < gCost[static_cast<size_t>(nIdx)]) {
                gCost[static_cast<size_t>(nIdx)] = tentativeG;
                parent[static_cast<size_t>(nIdx)] = curIdx;
                const double hVal = m_config.useDijkstra ? 0.0 : computeHeuristic(*neighbor, *goalNode, m_config.heuristicWeight);
                openSet.push(NodeEntry{nIdx, tentativeG + hVal});
            }
        }
    }

    result.nodesExpanded = expanded;
    const auto endTime = std::chrono::high_resolution_clock::now();
    result.planningTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    if (!found) {
        result.failureReason = (expanded >= m_config.maxExpandedNodes)
            ? "Search budget exceeded without reaching destination"
            : "No safe, traversable route exists between start and goal";
        return result;
    }

    // Reconstruct raw path from goal to start
    std::vector<PathWaypoint> rawWaypoints;
    int curr = goalIndex;
    while (curr != -1) {
        const int cx = curr % W;
        const int cz = curr / W;
        const NavNode* node = grid.getNode(cx, cz);

        PathWaypoint wp;
        wp.positionWorld = node->worldPosition;
        wp.surfaceNormal = node->surfaceNormal;
        wp.slopeAngleRad = node->slopeAngleRad;
        wp.costFromStart = gCost[static_cast<size_t>(curr)];
        // Adaptive speed: slow down on steep slopes
        const double maxSlope = grid.getConfig().maxTraversableSlopeRad;
        const double slopeFactor = clamp(1.0 - 0.5 * (node->slopeAngleRad / maxSlope), 0.35, 1.0);
        wp.targetSpeedMPerSec = 1.02 * slopeFactor; // Max ~1.02 m/s (NASA Perseverance top speed ~3.7 km/h)

        rawWaypoints.push_back(wp);
        curr = parent[static_cast<size_t>(curr)];
    }

    std::reverse(rawWaypoints.begin(), rawWaypoints.end());

    result.totalPathCost = gCost[static_cast<size_t>(goalIndex)];
    result.rawGridPath.reserve(rawWaypoints.size());
    for (const auto& wp : rawWaypoints) {
        result.rawGridPath.push_back(wp.positionWorld);
    }

    // Path smoothing (safety-preserving line-of-sight shortcutting)
    if (m_config.enableSmoothing && rawWaypoints.size() > 2) {
        result.waypoints = smoothPath(grid, rawWaypoints);
    } else {
        result.waypoints = rawWaypoints;
    }

    // Calculate total geometric distance
    double totalDist = 0.0;
    for (size_t i = 1; i < result.waypoints.size(); ++i) {
        const Vec3 delta = vec3Sub(result.waypoints[i].positionWorld, result.waypoints[i - 1].positionWorld);
        totalDist += vec3Length(delta);
    }
    result.totalDistanceM = totalDist;
    result.success = true;
    return result;
}

std::vector<PathWaypoint> AStarPlanner::smoothPath(
    const NavGrid& grid,
    const std::vector<PathWaypoint>& rawPath
) const {
    if (rawPath.size() <= 2) return rawPath;

    std::vector<PathWaypoint> smoothed;
    smoothed.reserve(rawPath.size());
    smoothed.push_back(rawPath.front());

    size_t i = 0;
    while (i < rawPath.size() - 1) {
        // Look as far ahead as possible
        size_t bestNext = i + 1;
        for (size_t j = rawPath.size() - 1; j > i + 1; --j) {
            if (isLineOfSightClear(grid, rawPath[i].positionWorld, rawPath[j].positionWorld)) {
                bestNext = j;
                break;
            }
        }
        smoothed.push_back(rawPath[bestNext]);
        i = bestNext;
    }

    return smoothed;
}

bool AStarPlanner::isLineOfSightClear(const NavGrid& grid, const Vec3& p0, const Vec3& p1) const {
    const double dx = p1.x - p0.x;
    const double dz = p1.z - p0.z;
    const double len = std::sqrt(dx * dx + dz * dz);
    if (len < 0.5) return true;

    // Step-march along ray at half-cell increments
    const double stepDist = grid.getCellSize() * 0.5;
    const int numSteps = static_cast<int>(std::ceil(len / stepDist));

    const double footprintR = grid.getConfig().roverFootprintRadiusM;

    for (int s = 1; s < numSteps; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(numSteps);
        const double x = p0.x + t * dx;
        const double z = p0.z + t * dz;

        const NavNode* node = grid.getNodeAtWorld(x, z);
        if (!node || node->traversability != CellTraversability::Traversable) {
            return false;
        }
        // Strict obstacle clearance: smoothed line must maintain full rover footprint buffer
        if (node->obstacleDistanceM < footprintR) {
            return false;
        }
    }
    return true;
}

} // namespace torquelab
