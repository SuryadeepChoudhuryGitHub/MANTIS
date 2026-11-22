#pragma once
/**
 * Project ARES — Autonomous Rover Exploration Simulator.
 *
 * Reinforcement Learning (RL) & Autonomous Navigation Telemetry / Actuator Interface.
 * Defines structured, normalized observation states and actuation commands for
 * NASA Mars Perseverance / Rocker-Bogie mobility control.
 *
 * Designed for direct integration with Python Gym / Gymnasium environments,
 * TorchScript C++ inference, or autonomous path planning controllers.
 */

#include <array>
#include <cmath>
#include "physics/Math3D.h"

namespace torquelab {

// Forward declaration
class VehicleDynamics;

/**
 * High-dimensional observation space capturing 6-DOF chassis dynamics,
 * rocker-bogie kinematic angles, 6-wheel terramechanics, and local terrain geometry.
 */
struct RoverObservationState {
    // 1. Chassis 6-DOF Inertial State
    Vec3 positionWorld{0.0, 0.0, 0.0};          // Chassis CoM position in world coordinates (m)
    Vec3 linearVelocityWorld{0.0, 0.0, 0.0};    // Linear velocity vector in world space (m/s)
    Quat orientationWorld{0.0, 0.0, 0.0, 1.0};  // Chassis orientation quaternion
    Vec3 angularVelocityWorld{0.0, 0.0, 0.0};   // Angular velocity in world space (rad/s)
    Vec3 linearVelocityChassis{0.0, 0.0, 0.0};  // Body-frame velocity (X=Lateral, Y=Vertical, Z=Forward) (m/s)
    Vec3 angularVelocityChassis{0.0, 0.0, 0.0}; // Body-frame angular velocity (X=PitchRate, Y=YawRate, Z=RollRate) (rad/s)
    Vec3 linearAccelerationChassis{0.0, 0.0, 0.0}; // Linear acceleration in body frame (m/s^2)

    // Euler angles (SI radians)
    double yawRad{0.0};                         // Heading angle around world Up
    double pitchRad{0.0};                       // Pitch angle (+ = nose up, - = nose down)
    double rollRad{0.0};                        // Roll angle (+ = roll right, - = roll left)

    // 2. Rocker-Bogie Suspension Kinematics
    double leftRockerPitchRad{0.0};             // Left rocker angle relative to chassis (rad)
    double rightRockerPitchRad{0.0};            // Right rocker angle relative to chassis (rad)
    double leftBogiePitchRad{0.0};              // Left bogie angle relative to left rocker (rad)
    double rightBogiePitchRad{0.0};             // Right bogie angle relative to right rocker (rad)
    double differentialBarAngleRad{0.0};        // Top differential pivot angle (rad)
    double leftRockerPitchRateRadPerSec{0.0};   // Left rocker pitch angular velocity (rad/s)
    double rightRockerPitchRateRadPerSec{0.0};  // Right rocker pitch angular velocity (rad/s)

    // 3. Per-Wheel Terramechanics & Dynamics (6 Wheels: 0=FL, 1=FR, 2=RL, 3=RR, 4=ML, 5=MR)
    std::array<double, 6> wheelAngularVelocitiesRadPerSec{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> wheelSteeringAnglesRad{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // ML & MR are 0.0
    std::array<double, 6> wheelSlipRatios{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};        // Longitudinal slip kappa
    std::array<double, 6> wheelSlipAnglesRad{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};     // Lateral slip angle alpha
    std::array<double, 6> wheelNormalLoadsN{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      // Contact normal load Fz
    std::array<double, 6> wheelTractionForcesN{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // Longitudinal force Fx
    std::array<double, 6> wheelLateralForcesN{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    // Lateral force Fy
    std::array<bool, 6>   wheelInContact{true, true, true, true, true, true};    // Terrain contact flag

    // 4. Local Terrain Footprint Geometry
    Vec3 terrainNormalWorld{0.0, 1.0, 0.0};     // Best-fit terrain surface normal under rover
    double terrainPitchSlopeRad{0.0};           // Terrain inclination along forward axis (rad)
    double terrainRollSlopeRad{0.0};            // Terrain cross-slope along lateral axis (rad)
    double bellyGroundClearanceM{0.35};         // Lowest distance between chassis belly and ground (m)
    int totalWheelsInContact{6};                // Number of wheels actively supporting weight [0..6]

    // 5. Numerical Health
    bool isHealthy{true};                       // False if NaNs, infinite velocities, or rollover detected
};

/**
 * Control action interface supporting both low-level torque/steering commands
 * and coordinated velocity/path follower inputs.
 */
struct RoverActuatorCommand {
    enum class ControlMode {
        DirectTorque,   // Independent wheel hub motor torques (N*m)
        TargetVelocity, // Wheel speed closed-loop control (rad/s)
        TeleopNormalized// Normalized [-1..+1] throttle, steer, brake commands
    };

    ControlMode mode{ControlMode::TeleopNormalized};

    // Teleop normalized inputs [-1.0 .. +1.0]
    double normalizedThrottle{0.0}; // Forward (+), Reverse (-)
    double normalizedSteering{0.0}; // Left (+), Right (-)
    double normalizedBrake{0.0};    // [0.0 .. 1.0]

    // Low-level individual wheel torques (N*m) [0=FL, 1=FR, 2=RL, 3=RR, 4=ML, 5=MR]
    std::array<double, 6> wheelDriveTorquesNm{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> wheelBrakeTorquesNm{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Low-level target wheel speeds (rad/s)
    std::array<double, 6> wheelTargetOmegasRadPerSec{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 4-Corner independent steering angles (rad) [0=FL, 1=FR, 2=RL, 3=RR]
    std::array<double, 4> cornerSteeringAnglesRad{0.0, 0.0, 0.0, 0.0};
};

/**
 * Extracts a complete, normalized RoverObservationState snapshot from active vehicle dynamics.
 */
RoverObservationState extractRoverObservation(const VehicleDynamics& vehicle);

/**
 * Dispatches an actuator command to the vehicle dynamics pipeline.
 */
void applyRoverActuatorCommand(VehicleDynamics& vehicle, const RoverActuatorCommand& cmd);

} // namespace torquelab
