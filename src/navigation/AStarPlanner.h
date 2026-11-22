#pragma once
/**
 * PROJECT ARES — A* Autonomous Planetary Path Planner.
 *
 * Implements optimal grid pathfinding with:
 * - Admissible Euclidean distance heuristic.
 * - Dijkstra exhaustive search validation mode.
 * - Multi-criteria terrain cost optimization (slope, roughness, rock clearance).
 * - Safety-preserving line-of-sight path smoothing.
 */

#include <cmath>
#include <vector>
#include "navigation/NavGrid.h"
#include "physics/Math3D.h"

namespace torquelab {

struct PlannerConfig {
    bool useDijkstra{false};         // If true, heuristic h(n) = 0 for validation against A*
    double heuristicWeight{1.0};     // 1.0 = standard admissible A* (optimal)
    bool enableSmoothing{true};      // Safety-preserving line-of-sight shortcutting
    int maxExpandedNodes{75000};     // Search budget to prevent application stalls
};

struct PathWaypoint {
    Vec3 positionWorld{0.0, 0.0, 0.0};
    Vec3 surfaceNormal{0.0, 1.0, 0.0};
    double targetSpeedMPerSec{1.0};
    double slopeAngleRad{0.0};
    double costFromStart{0.0};
};

struct PlanResult {
    bool success{false};
    std::string failureReason;
    std::vector<PathWaypoint> waypoints;
    std::vector<Vec3> rawGridPath;    // Unsmoothed node positions for visualization
    double totalPathCost{0.0};
    double totalDistanceM{0.0};
    int nodesExpanded{0};
    double planningTimeMs{0.0};
};

class AStarPlanner {
public:
    explicit AStarPlanner(const PlannerConfig& config = PlannerConfig{});

    void setConfig(const PlannerConfig& config) { m_config = config; }
    const PlannerConfig& getConfig() const { return m_config; }

    /**
     * Plans a collision-free, slope-safe route from startWorld to goalWorld across the NavGrid.
     */
    PlanResult plan(
        const NavGrid& grid,
        const Vec3& startWorld,
        const Vec3& goalWorld
    ) const;

private:
    static double computeHeuristic(const NavNode& current, const NavNode& goal, double weight);
    std::vector<PathWaypoint> smoothPath(const NavGrid& grid, const std::vector<PathWaypoint>& rawPath) const;
    bool isLineOfSightClear(const NavGrid& grid, const Vec3& p0, const Vec3& p1) const;

    PlannerConfig m_config;
};

} // namespace torquelab
