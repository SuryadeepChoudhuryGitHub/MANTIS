#include "navigation/PathFollower.h"
#include <algorithm>
#include <cmath>
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double rad) {
    while (rad > kPi) rad -= 2.0 * kPi;
    while (rad < -kPi) rad += 2.0 * kPi;
    return rad;
}

} // namespace

PathFollower::PathFollower(const PathFollowerConfig& config)
    : m_config(config) {}

void PathFollower::setPath(const std::vector<PathWaypoint>& waypoints) {
    const bool wasNavigating = (m_telemetry.state == NavState::Navigating);
    m_waypoints = waypoints;
    m_telemetry.currentWaypointIndex = 0;
    m_telemetry.totalWaypoints = waypoints.size();
    m_telemetry.isReplanRequested = false;
    m_stuckTimer = 0.0;
    m_prevHeadingError = 0.0;
    m_manualOverrideTimer = 0.0;
    m_explicitPause = false;

    if (!waypoints.empty()) {
        m_telemetry.destinationWorld = waypoints.back().positionWorld;
        m_telemetry.state = wasNavigating ? NavState::Navigating : NavState::Idle;
    } else {
        m_telemetry.state = NavState::Idle;
    }
}

void PathFollower::clearPath() {
    m_waypoints.clear();
    m_telemetry = PathFollowerTelemetry{};
    m_telemetry.state = NavState::Idle;
    m_stuckTimer = 0.0;
    m_manualOverrideTimer = 0.0;
    m_explicitPause = false;
}

void PathFollower::startNavigation() {
    if (!m_waypoints.empty()) {
        m_telemetry.state = NavState::Navigating;
        m_stuckTimer = 0.0;
        m_manualOverrideTimer = 0.0;
        m_explicitPause = false;
    }
}

void PathFollower::pauseNavigation(bool userExplicit) {
    if (m_telemetry.state == NavState::Navigating) {
        m_telemetry.state = NavState::Paused;
    }
    if (userExplicit) {
        m_explicitPause = true;
    }
}

void PathFollower::resumeNavigation() {
    if (m_telemetry.state == NavState::Paused && !m_waypoints.empty()) {
        m_telemetry.state = NavState::Navigating;
        m_explicitPause = false;
        m_manualOverrideTimer = 0.0;
    }
}

void PathFollower::emergencyStop() {
    m_telemetry.state = NavState::EmergencyStopped;
    m_explicitPause = true;
    m_manualOverrideTimer = 0.0;
    m_telemetry.commandedThrottle = 0.0;
    m_telemetry.commandedSteering = 0.0;
    m_telemetry.commandedBrake = 1.0;
}

Vec3 PathFollower::findLookaheadTarget(const Vec3& roverPos) const {
    if (m_waypoints.empty()) return roverPos;
    if (m_waypoints.size() == 1) return m_waypoints[0].positionWorld;

    const size_t startIdx = m_telemetry.currentWaypointIndex;
    const double Ld = m_config.lookaheadDistanceM;

    for (size_t i = startIdx; i < m_waypoints.size(); ++i) {
        const double d = std::sqrt(
            (m_waypoints[i].positionWorld.x - roverPos.x) * (m_waypoints[i].positionWorld.x - roverPos.x) +
            (m_waypoints[i].positionWorld.z - roverPos.z) * (m_waypoints[i].positionWorld.z - roverPos.z)
        );
        if (d >= Ld) {
            return m_waypoints[i].positionWorld;
        }
    }
    return m_waypoints.back().positionWorld;
}

void PathFollower::computeCrossTrackAndHeading(
    const Vec3& roverPos,
    double roverYaw,
    double& outCrossTrack,
    double& outHeadingError
) const {
    if (m_waypoints.empty()) {
        outCrossTrack = 0.0;
        outHeadingError = 0.0;
        return;
    }

    const Vec3 lookTarget = findLookaheadTarget(roverPos);
    const double dx = lookTarget.x - roverPos.x;
    const double dz = lookTarget.z - roverPos.z;

    // Desired heading to lookahead point (+Z is Forward, +X is Left in right-handed convention)
    const double desiredHeading = std::atan2(dx, dz);
    outHeadingError = wrapAngle(desiredHeading - roverYaw);

    // Cross-track distance to active path segment
    if (m_telemetry.currentWaypointIndex < m_waypoints.size()) {
        const Vec3 wp = m_waypoints[m_telemetry.currentWaypointIndex].positionWorld;
        outCrossTrack = std::sqrt((wp.x - roverPos.x) * (wp.x - roverPos.x) + (wp.z - roverPos.z) * (wp.z - roverPos.z));
    } else {
        outCrossTrack = 0.0;
    }
}

bool PathFollower::update(
    double dt,
    VehicleDynamics& vehicle,
    bool humanInterfered
) {
    // 1. Human-in-the-loop manual override handling
    if (humanInterfered) {
        m_manualOverrideTimer = 0.8; // Hold override for 0.8s after manual inputs cease
        if (m_telemetry.state == NavState::Navigating) {
            m_telemetry.state = NavState::Paused;
        }
        return false;
    }

    if (m_manualOverrideTimer > 0.0) {
        m_manualOverrideTimer -= dt;
        if (m_manualOverrideTimer <= 0.0 && !m_explicitPause && m_telemetry.state == NavState::Paused && !m_waypoints.empty()) {
            m_telemetry.state = NavState::Navigating;
            m_telemetry.isReplanRequested = true;
        }
    }

    // 2. Emergency stop or explicit paused state enforcement
    if (m_telemetry.state == NavState::EmergencyStopped || (m_telemetry.state == NavState::Paused && m_explicitPause)) {
        vehicle.currentInputs.throttle = 0.0;
        vehicle.currentInputs.brake = 1.0;
        vehicle.currentInputs.handbrake = true;
        m_telemetry.commandedThrottle = 0.0;
        m_telemetry.commandedBrake = 1.0;
        return false;
    }

    if (m_telemetry.state != NavState::Navigating || m_waypoints.empty()) {
        return false;
    }

    // 3. Extract rover inertial state
    const Vec3 roverPos = vehicle.body.getPosition();
    const Quat roverRot = vehicle.body.getRotation();
    const VehicleEulerAngles euler = getVehicleEulerAngles(roverRot);
    const double roverYaw = euler.yaw;
    const double roverSpeed = vec3Length(vehicle.body.getLinearVelocity());
    m_telemetry.currentSpeedMPerSec = roverSpeed;

    // 4. Distance to final destination & Arrival detection
    const Vec3 finalDest = m_waypoints.back().positionWorld;
    const double distToDest = std::sqrt(
        (finalDest.x - roverPos.x) * (finalDest.x - roverPos.x) +
        (finalDest.z - roverPos.z) * (finalDest.z - roverPos.z)
    );
    m_telemetry.distanceToTargetM = distToDest;

    if (distToDest <= m_config.arrivalThresholdM) {
        m_telemetry.state = NavState::Arrived;
        vehicle.currentInputs.throttle = 0.0;
        vehicle.currentInputs.brake = 1.0;
        vehicle.currentInputs.handbrake = true;
        vehicle.currentInputs.steering = 0.0;
        m_telemetry.commandedThrottle = 0.0;
        m_telemetry.commandedBrake = 1.0;
        m_telemetry.commandedSteering = 0.0;
        return false;
    }

    // 5. Waypoint sequential advancement
    while (m_telemetry.currentWaypointIndex < m_waypoints.size() - 1) {
        const Vec3 curWp = m_waypoints[m_telemetry.currentWaypointIndex].positionWorld;
        const Vec3 nextWp = m_waypoints[m_telemetry.currentWaypointIndex + 1].positionWorld;
        const double dToWp = std::sqrt(
            (curWp.x - roverPos.x) * (curWp.x - roverPos.x) +
            (curWp.z - roverPos.z) * (curWp.z - roverPos.z)
        );
        const double segDx = nextWp.x - curWp.x;
        const double segDz = nextWp.z - curWp.z;
        const double segLenSq = segDx * segDx + segDz * segDz;
        const double proj = (roverPos.x - curWp.x) * segDx + (roverPos.z - curWp.z) * segDz;

        if (dToWp <= m_config.waypointAdvanceRadiusM || (segLenSq > 1e-4 && proj >= segLenSq)) {
            m_telemetry.currentWaypointIndex++;
        } else {
            break;
        }
    }

    // 6. Cross-track error and excessive deviation replan trigger
    double crossTrack = 0.0;
    double headingErr = 0.0;
    computeCrossTrackAndHeading(roverPos, roverYaw, crossTrack, headingErr);
    m_telemetry.crossTrackErrorM = crossTrack;
    m_telemetry.headingErrorRad = headingErr;

    if (crossTrack > m_config.maxDeviationM) {
        m_telemetry.isReplanRequested = true;
    }

    // 7. Steering Controller (Proportional-Derivative)
    const double dHeadingErr = (headingErr - m_prevHeadingError) / std::max(1e-4, dt);
    m_prevHeadingError = headingErr;
    const double rawSteer = m_config.steerKp * headingErr + m_config.steerKd * dHeadingErr;
    const double commandedSteer = clamp(rawSteer, -1.0, 1.0);
    m_telemetry.commandedSteering = commandedSteer;

    // 8. Adaptive Speed Regulation
    const auto& curWp = m_waypoints[m_telemetry.currentWaypointIndex];
    double targetSpeed = clamp(curWp.targetSpeedMPerSec, 0.25, m_config.cruiseSpeedMPerSec);

    // Speed reduction on sharp steering turns
    const double turnFactor = clamp(1.0 - 0.45 * std::abs(commandedSteer), 0.35, 1.0);
    targetSpeed *= turnFactor;

    if (std::abs(headingErr) > 0.5) {
        // High heading error: reduce speed to negotiate turn safely
        const double turnScale = clamp(1.0 - (std::abs(headingErr) - 0.5) / 1.0, 0.30, 1.0);
        targetSpeed *= turnScale;
    }

    // Smooth ramp-down approaching final destination
    if (distToDest < 4.0) {
        targetSpeed *= clamp(distToDest / 4.0, 0.20, 1.0);
    }
    m_telemetry.targetSpeedMPerSec = targetSpeed;
    m_telemetry.estimatedTimeRemainingSec = distToDest / std::max(0.1, roverSpeed);

    // 9. Throttle and Service Brake Control with Incline Feed-Forward
    double commandedThrottle = 0.0;
    double commandedBrake = 0.0;

    const double inclineFeedForward = clamp(std::sin(euler.pitch) * 0.40, -0.20, 0.45);
    const double speedError = targetSpeed - roverSpeed;
    if (speedError > 0.05) {
        commandedThrottle = clamp(m_config.speedKp * speedError + 0.32 + inclineFeedForward, 0.0, 1.0);
        commandedBrake = 0.0;
    } else if (speedError < -0.15) {
        commandedThrottle = 0.0;
        commandedBrake = clamp(-m_config.speedKp * speedError, 0.0, 0.85);
    } else {
        commandedThrottle = clamp(0.35 + inclineFeedForward, 0.15, 0.75); // Cruising sustain throttle
        commandedBrake = 0.0;
    }

    m_telemetry.commandedThrottle = commandedThrottle;
    m_telemetry.commandedBrake = commandedBrake;

    // 10. Stuck / Wheel Slip Detection
    if (commandedThrottle > 0.50 && roverSpeed < 0.05) {
        m_stuckTimer += dt;
        if (m_stuckTimer >= m_config.stuckTimeoutSec) {
            m_telemetry.state = NavState::Stuck;
            m_telemetry.isReplanRequested = true;
            commandedThrottle = 0.0;
            commandedBrake = 0.8;
        }
    } else {
        m_stuckTimer = std::max(0.0, m_stuckTimer - dt * 2.0);
    }

    // 11. Feed commands into vehicle input state
    vehicle.currentInputs.steering = commandedSteer;
    vehicle.currentInputs.throttle = commandedThrottle;
    vehicle.currentInputs.brake = commandedBrake;
    vehicle.currentInputs.handbrake = false;

    return true;
}

} // namespace torquelab
