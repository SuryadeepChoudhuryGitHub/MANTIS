/**
 * Project MANTIS — Autonomous Rover Exploration Simulator.
 * Reinforcement Learning (RL) Observation & Actuator Implementation.
 */

#include "vehicle/RoverRLInterface.h"
#include <algorithm>
#include <cmath>
#include "terrain/TerrainManager.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

RoverObservationState extractRoverObservation(const VehicleDynamics& vehicle) {
    RoverObservationState obs{};

    // 1. Chassis 6-DOF Kinematics
    obs.positionWorld = vehicle.body.getPosition();
    obs.linearVelocityWorld = vehicle.body.getLinearVelocity();
    obs.orientationWorld = vehicle.body.getRotation();
    obs.angularVelocityWorld = vehicle.body.getAngularVelocity();

    const VehicleBasis basis = getVehicleBasis(obs.orientationWorld);
    const VehicleEulerAngles euler = getVehicleEulerAngles(obs.orientationWorld);

    obs.linearVelocityChassis = Vec3{
        vec3Dot(obs.linearVelocityWorld, basis.left),
        vec3Dot(obs.linearVelocityWorld, basis.up),
        vec3Dot(obs.linearVelocityWorld, basis.forward)
    };

    obs.angularVelocityChassis = Vec3{
        vec3Dot(obs.angularVelocityWorld, basis.left),
        vec3Dot(obs.angularVelocityWorld, basis.up),
        vec3Dot(obs.angularVelocityWorld, basis.forward)
    };

    obs.linearAccelerationChassis = Vec3{
        vec3Dot(vehicle.body.accelerationWorld, basis.left),
        vec3Dot(vehicle.body.accelerationWorld, basis.up),
        vec3Dot(vehicle.body.accelerationWorld, basis.forward)
    };

    obs.yawRad = euler.yaw;
    obs.pitchRad = euler.pitch;
    obs.rollRad = euler.roll;

    // 2. Rocker-Bogie Suspension Kinematics
    const auto& rb = vehicle.rockerBogie.getState();
    obs.leftRockerPitchRad = rb.leftRockerPitch;
    obs.rightRockerPitchRad = rb.rightRockerPitch;
    obs.leftBogiePitchRad = rb.leftBogiePitch;
    obs.rightBogiePitchRad = rb.rightBogiePitch;
    obs.differentialBarAngleRad = rb.differentialAngle;
    obs.leftRockerPitchRateRadPerSec = rb.leftRockerAngVel;
    obs.rightRockerPitchRateRadPerSec = rb.rightRockerAngVel;
    obs.totalWheelsInContact = rb.wheelsInContactCount;

    // 3. Per-Wheel Terramechanics (6 Wheels)
    const int numWheels = std::clamp(vehicle.config.numWheels, 4, 6);
    for (int i = 0; i < numWheels; ++i) {
        const auto& w = vehicle.wheels[static_cast<size_t>(i)];
        obs.wheelAngularVelocitiesRadPerSec[static_cast<size_t>(i)] = w.angularVelocity;
        obs.wheelSteeringAnglesRad[static_cast<size_t>(i)] = w.steeringAngle;
        obs.wheelSlipRatios[static_cast<size_t>(i)] = w.slipRatio;
        obs.wheelSlipAnglesRad[static_cast<size_t>(i)] = w.slipAngle;
        obs.wheelNormalLoadsN[static_cast<size_t>(i)] = w.normalLoad;
        obs.wheelTractionForcesN[static_cast<size_t>(i)] = w.longitudinalForce;
        obs.wheelLateralForcesN[static_cast<size_t>(i)] = w.lateralForce;
        obs.wheelInContact[static_cast<size_t>(i)] = w.inContact;
    }

    // 4. Local Terrain Footprint
    obs.terrainNormalWorld = basis.up; // Default to chassis up if no terrain
    if (vehicle.getTerrainManager()) {
        double elev = 0.0;
        Vec3 norm{0.0, 1.0, 0.0};
        if (vehicle.getTerrainManager()->getElevationAt(obs.positionWorld.x, obs.positionWorld.z, elev, norm)) {
            obs.terrainNormalWorld = norm;
            obs.bellyGroundClearanceM = std::max(0.0, obs.positionWorld.y - elev);
            obs.terrainPitchSlopeRad = std::asin(clamp(vec3Dot(norm, basis.forward), -1.0, 1.0));
            obs.terrainRollSlopeRad = std::asin(clamp(vec3Dot(norm, basis.left), -1.0, 1.0));
        }
    }

    // 5. Health Check
    obs.isHealthy = isVec3Finite(obs.positionWorld) &&
                    isVec3Finite(obs.linearVelocityWorld) &&
                    isVec3Finite(obs.angularVelocityWorld) &&
                    std::abs(obs.pitchRad) < 1.30 && // < 75 degrees pitch
                    std::abs(obs.rollRad) < 1.30;    // < 75 degrees roll

    return obs;
}

void applyRoverActuatorCommand(VehicleDynamics& vehicle, const RoverActuatorCommand& cmd) {
    if (cmd.mode == RoverActuatorCommand::ControlMode::TeleopNormalized) {
        VehicleControlInputs inputs{};
        inputs.throttle = clamp(cmd.normalizedThrottle, -1.0, 1.0);
        inputs.steering = clamp(cmd.normalizedSteering, -1.0, 1.0);
        inputs.brake = clamp(cmd.normalizedBrake, 0.0, 1.0);
        inputs.handbrake = false;
        vehicle.currentInputs = inputs;
    } else if (cmd.mode == RoverActuatorCommand::ControlMode::DirectTorque) {
        // Direct individual wheel hub torques
        const int numWheels = std::clamp(vehicle.config.numWheels, 4, 6);
        for (int i = 0; i < numWheels; ++i) {
            vehicle.wheels[static_cast<size_t>(i)].driveTorque = cmd.wheelDriveTorquesNm[static_cast<size_t>(i)];
            vehicle.wheels[static_cast<size_t>(i)].brakeTorque = cmd.wheelBrakeTorquesNm[static_cast<size_t>(i)];
        }
        // Direct steering angles on 4 corners
        vehicle.wheels[0].steeringAngle = cmd.cornerSteeringAnglesRad[0]; // FL
        vehicle.wheels[1].steeringAngle = cmd.cornerSteeringAnglesRad[1]; // FR
        vehicle.wheels[2].steeringAngle = cmd.cornerSteeringAnglesRad[2]; // RL
        vehicle.wheels[3].steeringAngle = cmd.cornerSteeringAnglesRad[3]; // RR
    } else if (cmd.mode == RoverActuatorCommand::ControlMode::TargetVelocity) {
        // Closed-loop wheel velocity control (proportional hub motor torque)
        const int numWheels = std::clamp(vehicle.config.numWheels, 4, 6);
        const double kP = 45.0; // N*m per (rad/s) speed error
        for (int i = 0; i < numWheels; ++i) {
            auto& w = vehicle.wheels[static_cast<size_t>(i)];
            const double targetOmega = cmd.wheelTargetOmegasRadPerSec[static_cast<size_t>(i)];
            const double errorOmega = targetOmega - w.angularVelocity;
            w.driveTorque = clamp(errorOmega * kP, -480.0, 480.0);
        }
        vehicle.wheels[0].steeringAngle = cmd.cornerSteeringAnglesRad[0];
        vehicle.wheels[1].steeringAngle = cmd.cornerSteeringAnglesRad[1];
        vehicle.wheels[2].steeringAngle = cmd.cornerSteeringAnglesRad[2];
        vehicle.wheels[3].steeringAngle = cmd.cornerSteeringAnglesRad[3];
    }
}

} // namespace torquelab
