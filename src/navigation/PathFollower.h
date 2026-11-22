#pragma once
/**
 * PROJECT ARES — Autonomous Rover Path Following Controller.
 *
 * Implements a closed-loop Pure Pursuit waypoint-tracking controller that generates
 * authentic actuator commands (steering angle and drive torque / throttle) for the
 * NASA Perseverance 6-wheel rocker-bogie mobility system.
 *
 * Features:
 * - Dynamic terrain-adaptive speed regulation (slope, roughness, curvature).
 * - Multi-axle Ackermann steering angle calculation.
 * - Deviation monitoring, stuck/slip detection, and automatic replanning triggers.
 * - Seamless human-in-the-loop manual override and instant emergency stopping.
 */

#include <vector>
#include "navigation/AStarPlanner.h"
#include "physics/Math3D.h"
#include "vehicle/RoverRLInterface.h"

namespace torquelab {

class VehicleDynamics;

enum class NavState {
    Idle,
    Planning,
    Navigating,
    Paused,
    Arrived,
    Stuck,
    EmergencyStopped
};

struct PathFollowerConfig {
    double lookaheadDistanceM{2.2};      // Pure pursuit lookahead distance (m)
    double arrivalThresholdM{0.85};      // Distance to final waypoint to declare arrival (m)
    double waypointAdvanceRadiusM{1.5};  // Distance to advance to next intermediate waypoint (m)
    double cruiseSpeedMPerSec{1.02};     // Maximum autonomous exploration speed (~3.7 km/h)
    double steerKp{1.8};                 // Steering proportional gain
    double steerKd{0.25};                // Steering derivative damping gain
    double speedKp{2.0};                 // Speed control proportional gain
    double maxDeviationM{2.8};           // Cross-track error threshold triggering dynamic replan
    double stuckTimeoutSec{2.5};         // Persistence of zero velocity under throttle before declaring stuck
};

struct PathFollowerTelemetry {
    NavState state{NavState::Idle};
    size_t currentWaypointIndex{0};
    size_t totalWaypoints{0};
    Vec3 destinationWorld{0.0, 0.0, 0.0};
    double distanceToTargetM{0.0};
    double crossTrackErrorM{0.0};
    double headingErrorRad{0.0};
    double currentSpeedMPerSec{0.0};
    double targetSpeedMPerSec{0.0};
    double commandedThrottle{0.0};
    double commandedSteering{0.0};
    double commandedBrake{0.0};
    double estimatedTimeRemainingSec{0.0};
    bool isReplanRequested{false};
};

class PathFollower {
public:
    explicit PathFollower(const PathFollowerConfig& config = PathFollowerConfig{});

    void setConfig(const PathFollowerConfig& config) { m_config = config; }
    const PathFollowerConfig& getConfig() const { return m_config; }

    /** Sets a new planned route for the rover to follow */
    void setPath(const std::vector<PathWaypoint>& waypoints);

    /** Clears active route and returns to Idle state */
    void clearPath();

    /** User control actions */
    void startNavigation();
    void pauseNavigation(bool userExplicit = false);
    void resumeNavigation();
    void emergencyStop();

    /**
     * Executes one fixed-step control cycle (e.g. at 120 Hz or frame rate).
     * Computes steer, throttle, and brake commands and applies them to vehicle.
     * Returns true if autonomous navigation is currently active.
     */
    bool update(
        double dt,
        VehicleDynamics& vehicle,
        bool humanInterfered = false
    );

    const PathFollowerTelemetry& getTelemetry() const { return m_telemetry; }
    NavState getState() const { return m_telemetry.state; }
    bool isNavigating() const { return m_telemetry.state == NavState::Navigating; }
    bool hasActiveMission() const {
        return !m_waypoints.empty() &&
               m_telemetry.state != NavState::Arrived &&
               m_telemetry.state != NavState::EmergencyStopped;
    }
    bool isReplanRequested() const { return m_telemetry.isReplanRequested; }
    void clearReplanRequest() { m_telemetry.isReplanRequested = false; }
    const std::vector<PathWaypoint>& getWaypoints() const { return m_waypoints; }

private:
    Vec3 findLookaheadTarget(const Vec3& roverPos) const;
    void computeCrossTrackAndHeading(
        const Vec3& roverPos,
        double roverYaw,
        double& outCrossTrack,
        double& outHeadingError
    ) const;

    PathFollowerConfig m_config;
    std::vector<PathWaypoint> m_waypoints;
    PathFollowerTelemetry m_telemetry;

    double m_stuckTimer{0.0};
    double m_prevHeadingError{0.0};
    double m_manualOverrideTimer{0.0};
    bool m_explicitPause{false};
};

} // namespace torquelab
