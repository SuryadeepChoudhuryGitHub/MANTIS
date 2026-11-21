#include "vehicle/RockerBogieSuspension.h"
#include "vehicle/VehicleBody.h"
#include <algorithm>
#include <cmath>
#include "terrain/TerrainManager.h"

namespace torquelab {

namespace {

Quat makePitchQuat(double angleRad) {
    return quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, angleRad);
}

struct ExcludeRockerBogieRayCallback : public btCollisionWorld::ClosestRayResultCallback {
    ExcludeRockerBogieRayCallback(
        const btVector3& from,
        const btVector3& to,
        const btCollisionObject* chassisBody,
        const std::array<const btCollisionObject*, 6>& wheelObjs
    ) : btCollisionWorld::ClosestRayResultCallback(from, to),
        m_chassisBody(chassisBody),
        m_wheelObjs(wheelObjs) {}

    btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) override {
        if (rayResult.m_collisionObject == m_chassisBody) {
            return 1.0f;
        }
        for (size_t i = 0; i < 6; ++i) {
            if (m_wheelObjs[i] && rayResult.m_collisionObject == m_wheelObjs[i]) {
                return 1.0f;
            }
        }
        return btCollisionWorld::ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
    }

private:
    const btCollisionObject* m_chassisBody{nullptr};
    std::array<const btCollisionObject*, 6> m_wheelObjs{};
};

} // namespace

void RockerBogieSuspension::init(const VehicleConfig& config) {
    if (config.wheelbase > 1.8) {
        // Authentic NASA Mars 2020 Perseverance / MSL Curiosity dimensions from CAD geometry
        m_diffPivotLocal        = Vec3{ 0.000,  0.457, -0.114};
        m_leftRockerPivotLocal  = Vec3{ 0.806,  0.190,  0.249};
        m_rightRockerPivotLocal = Vec3{-0.806,  0.190,  0.249};
        m_rockerToFrontLocal    = Vec3{ 0.256, -0.400,  0.846};
        m_rockerToBogieLocal    = Vec3{ 0.100, -0.237, -0.754};
        m_leftBogiePivotLocal   = Vec3{ 0.906, -0.047, -0.505};
        m_rightBogiePivotLocal  = Vec3{-0.906, -0.047, -0.505};
        m_bogieToMidLocal       = Vec3{ 0.278, -0.163,  0.415};
        m_bogieToRearLocal      = Vec3{ 0.156, -0.163, -0.660};
    } else {
        // Scaled geometry for smaller rovers (e.g. Opportunity MER)
        const double s = config.wheelbase / 2.26;
        m_diffPivotLocal        = Vec3{ 0.000,  0.457 * s, -0.114 * s};
        m_leftRockerPivotLocal  = Vec3{ 0.806 * s,  0.190 * s,  0.249 * s};
        m_rightRockerPivotLocal = Vec3{-0.806 * s,  0.190 * s,  0.249 * s};
        m_rockerToFrontLocal    = Vec3{ 0.256 * s, -0.400 * s,  0.846 * s};
        m_rockerToBogieLocal    = Vec3{ 0.100 * s, -0.237 * s, -0.754 * s};
        m_leftBogiePivotLocal   = Vec3{ 0.906 * s, -0.047 * s, -0.505 * s};
        m_rightBogiePivotLocal  = Vec3{-0.906 * s, -0.047 * s, -0.505 * s};
        m_bogieToMidLocal       = Vec3{ 0.278 * s, -0.163 * s,  0.415 * s};
        m_bogieToRearLocal      = Vec3{ 0.156 * s, -0.163 * s, -0.660 * s};
    }
}

void RockerBogieSuspension::update(
    double dt,
    VehicleBody& body,
    const VehicleConfig& config,
    btDiscreteDynamicsWorld* world,
    SurfaceSystem* surfaceSystem,
    std::array<WheelState, 6>& wheels,
    double centerSteeringInput,
    const std::array<const btCollisionObject*, 6>& wheelObjsToIgnore,
    const TerrainManager* terrainManager
) {
    const Vec3 chassisPos = body.getPosition();
    const Quat chassisRot = body.getRotation();
    const VehicleBasis basis = getVehicleBasis(chassisRot);
    const Mat4 bodyTrans = Mat4::fromTRS(chassisPos, chassisRot);

    // 1. World positions of Left and Right Rocker Pivots (Trunnions)
    m_state.leftRockerPivotWorld = mat4TransformPoint(bodyTrans, m_leftRockerPivotLocal);
    m_state.rightRockerPivotWorld = mat4TransformPoint(bodyTrans, m_rightRockerPivotLocal);

    // 2. Multi-axle Ackermann steering for 4 corner wheels; mid wheels are non-steering (rigidly straight)
    // Front wheels steer in direction of turn (+ = Left); Rear wheels steer in opposite phase for minimum turn radius
    const double maxSteer = config.maxSteeringAngle;
    const double steerIn = clamp(centerSteeringInput, -1.0, 1.0) * maxSteer;

    // Corner Ackermann geometry:
    // Outer front wheel turns slightly less than inner front wheel; rear mirrors front
    const double turnRadius = (std::abs(steerIn) > 1e-4) ? (config.wheelbase / std::tan(std::abs(steerIn))) : 1e6;
    const double halfTrack = config.trackWidth * 0.5;

    double steerInnerFront = steerIn;
    double steerOuterFront = steerIn;
    if (std::abs(steerIn) > 1e-4) {
        steerInnerFront = std::copysign(std::atan(config.wheelbase / (turnRadius - halfTrack)), steerIn);
        steerOuterFront = std::copysign(std::atan(config.wheelbase / (turnRadius + halfTrack)), steerIn);
    }

    if (steerIn >= 0.0) {
        // Left turn: Left wheels are inner, Right wheels are outer
        wheels[0].steeringAngle = steerInnerFront;  // FL
        wheels[1].steeringAngle = steerOuterFront;  // FR
        wheels[2].steeringAngle = -steerInnerFront; // RL (counter-steered)
        wheels[3].steeringAngle = -steerOuterFront; // RR (counter-steered)
    } else {
        // Right turn: Right wheels are inner, Left wheels are outer
        wheels[0].steeringAngle = steerOuterFront;  // FL
        wheels[1].steeringAngle = steerInnerFront;  // FR
        wheels[2].steeringAngle = -steerOuterFront; // RL (counter-steered)
        wheels[3].steeringAngle = -steerInnerFront; // RR (counter-steered)
    }
    wheels[4].steeringAngle = 0.0; // ML non-steering
    wheels[5].steeringAngle = 0.0; // MR non-steering

    // 3. Ground raycast probing for all 6 wheels with authoritative terrain elevation integration
    std::array<Vec3, 6> groundPoints;
    std::array<Vec3, 6> groundNormals;
    std::array<bool, 6> groundHits;
    std::array<double, 6> groundY;

    // Compute unarticulated nominal hub coordinates to anchor raycasts
    const Vec3 nomFL = mat4TransformPoint(bodyTrans, vec3Add(m_leftRockerPivotLocal, m_rockerToFrontLocal));
    const Vec3 nomFR = mat4TransformPoint(bodyTrans, vec3Add(m_rightRockerPivotLocal, Vec3{-m_rockerToFrontLocal.x, m_rockerToFrontLocal.y, m_rockerToFrontLocal.z}));
    const Vec3 nomBogieL = mat4TransformPoint(bodyTrans, vec3Add(m_leftRockerPivotLocal, m_rockerToBogieLocal));
    const Vec3 nomBogieR = mat4TransformPoint(bodyTrans, vec3Add(m_rightRockerPivotLocal, Vec3{-m_rockerToBogieLocal.x, m_rockerToBogieLocal.y, m_rockerToBogieLocal.z}));
    const Vec3 nomML = mat4TransformPoint(bodyTrans, vec3Add(m_leftBogiePivotLocal, m_bogieToMidLocal));
    const Vec3 nomMR = mat4TransformPoint(bodyTrans, vec3Add(m_rightBogiePivotLocal, Vec3{-m_bogieToMidLocal.x, m_bogieToMidLocal.y, m_bogieToMidLocal.z}));
    const Vec3 nomRL = mat4TransformPoint(bodyTrans, vec3Add(m_leftBogiePivotLocal, m_bogieToRearLocal));
    const Vec3 nomRR = mat4TransformPoint(bodyTrans, vec3Add(m_rightBogiePivotLocal, Vec3{-m_bogieToRearLocal.x, m_bogieToRearLocal.y, m_bogieToRearLocal.z}));

    std::array<Vec3, 6> nominalHubs = { nomFL, nomFR, nomRL, nomRR, nomML, nomMR };

    for (size_t i = 0; i < 6; ++i) {
        const Vec3& hubNom = nominalHubs[i];

        // Authoritative terrain model elevation query
        double tElev = 0.0;
        Vec3 tNorm{0.0, 1.0, 0.0};
        bool hasTerrain = false;
        if (terrainManager) {
            hasTerrain = terrainManager->getElevationAt(hubNom.x, hubNom.z, tElev, tNorm);
            if (hasTerrain) {
                wheels[i].surface = getSurfaceProperties(terrainManager->getSurfaceMaterialAt(hubNom.x, hubNom.z));
            }
        }

        const Vec3 rayStart = vec3Add(hubNom, vec3Scale(basis.up, 0.80));
        const Vec3 rayEnd   = vec3Add(hubNom, vec3Scale(basis.down, 1.80));

        const btVector3 rFrom(static_cast<btScalar>(rayStart.x), static_cast<btScalar>(rayStart.y), static_cast<btScalar>(rayStart.z));
        const btVector3 rTo(static_cast<btScalar>(rayEnd.x), static_cast<btScalar>(rayEnd.y), static_cast<btScalar>(rayEnd.z));

        ExcludeRockerBogieRayCallback cb(rFrom, rTo, body.getRigidBody(), wheelObjsToIgnore);
        if (world) {
            world->rayTest(rFrom, rTo, cb);
        }

        if (cb.hasHit()) {
            const double hitY = static_cast<double>(cb.m_hitPointWorld.y());
            if (!hasTerrain || hitY >= tElev - 0.05) {
                groundHits[i] = true;
                groundPoints[i] = Vec3{
                    static_cast<double>(cb.m_hitPointWorld.x()),
                    hitY,
                    static_cast<double>(cb.m_hitPointWorld.z())
                };
                groundNormals[i] = vec3Normalize(Vec3{
                    static_cast<double>(cb.m_hitNormalWorld.x()),
                    static_cast<double>(cb.m_hitNormalWorld.y()),
                    static_cast<double>(cb.m_hitNormalWorld.z())
                });
                groundY[i] = hitY;
                if (surfaceSystem && cb.m_collisionObject) {
                    wheels[i].surface = surfaceSystem->getSurfaceForCollider(cb.m_collisionObject->getUserIndex());
                }
            } else {
                groundHits[i] = true;
                groundPoints[i] = Vec3{hubNom.x, tElev, hubNom.z};
                groundNormals[i] = tNorm;
                groundY[i] = tElev;
            }
        } else if (hasTerrain) {
            groundHits[i] = true;
            groundPoints[i] = Vec3{hubNom.x, tElev, hubNom.z};
            groundNormals[i] = tNorm;
            groundY[i] = tElev;
        } else {
            groundHits[i] = false;
            groundPoints[i] = vec3Add(hubNom, vec3Scale(basis.down, wheels[i].wheelRadius));
            groundNormals[i] = basis.up;
            groundY[i] = groundPoints[i].y;
        }
    }

    // 4. Calculate passive Rocker-Bogie kinematic pitch angles
    // Left Side:
    // Bogie pitch connects ML (idx 4) and RL (idx 2)
    const double bogieDeltaYL = (groundY[4] - groundY[2]);
    const double bogieDeltaZL = (nominalHubs[4].z - nominalHubs[2].z);
    const double targetLeftBogiePitch = clamp(std::atan2(bogieDeltaYL, std::max(0.2, bogieDeltaZL)), -0.60, 0.60);

    const double bogieCenterYL = 0.5 * (groundY[4] + groundY[2]) + wheels[4].wheelRadius;
    const double rockerDeltaYL = (groundY[0] + wheels[0].wheelRadius) - bogieCenterYL;
    const double rockerDeltaZL = (nominalHubs[0].z - nomBogieL.z);
    const double targetLeftRockerPitch = clamp(std::atan2(rockerDeltaYL, std::max(0.4, rockerDeltaZL)), -0.55, 0.55);

    // Right Side:
    // Bogie pitch connects MR (idx 5) and RR (idx 3)
    const double bogieDeltaYR = (groundY[5] - groundY[3]);
    const double bogieDeltaZR = (nominalHubs[5].z - nominalHubs[3].z);
    const double targetRightBogiePitch = clamp(std::atan2(bogieDeltaYR, std::max(0.2, bogieDeltaZR)), -0.60, 0.60);

    const double bogieCenterYR = 0.5 * (groundY[5] + groundY[3]) + wheels[5].wheelRadius;
    const double rockerDeltaYR = (groundY[1] + wheels[1].wheelRadius) - bogieCenterYR;
    const double rockerDeltaZR = (nominalHubs[1].z - nomBogieR.z);
    const double targetRightRockerPitch = clamp(std::atan2(rockerDeltaYR, std::max(0.4, rockerDeltaZR)), -0.55, 0.55);

    // Central Differential Coupling Bar constraint:
    // As left rocker tilts up, the central differential bar forces the right rocker down:
    // theta_rocker_left ≈ -theta_rocker_right relative to chassis pitch
    const double avgRockerPitch = 0.5 * (targetLeftRockerPitch + targetRightRockerPitch);
    const double coupledLeftRockerTarget = targetLeftRockerPitch - 0.5 * avgRockerPitch;
    const double coupledRightRockerTarget = targetRightRockerPitch - 0.5 * avgRockerPitch;

    // Smooth second-order kinematic tracking
    const double alpha = 1.0 - std::exp(-25.0 * dt);
    m_state.leftRockerPitch  = lerp(m_state.leftRockerPitch, coupledLeftRockerTarget, alpha);
    m_state.rightRockerPitch = lerp(m_state.rightRockerPitch, coupledRightRockerTarget, alpha);
    m_state.leftBogiePitch   = lerp(m_state.leftBogiePitch, targetLeftBogiePitch, alpha);
    m_state.rightBogiePitch  = lerp(m_state.rightBogiePitch, targetRightBogiePitch, alpha);
    m_state.differentialAngle = 0.5 * (m_state.leftRockerPitch - m_state.rightRockerPitch);

    // 5. Compute Articulated World Hub Positions and Linkage Orientations
    m_state.differentialPivotWorld = mat4TransformPoint(bodyTrans, m_diffPivotLocal);
    m_state.differentialOrientationWorld = quatMultiply(chassisRot, quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, m_state.differentialAngle));

    const Quat qLeftRocker = quatMultiply(chassisRot, makePitchQuat(m_state.leftRockerPitch));
    const Quat qRightRocker = quatMultiply(chassisRot, makePitchQuat(m_state.rightRockerPitch));
    m_state.leftRockerOrientationWorld = qLeftRocker;
    m_state.rightRockerOrientationWorld = qRightRocker;

    // Front hubs (FL idx 0, FR idx 1)
    m_state.wheelHubsWorld[0] = vec3Add(m_state.leftRockerPivotWorld, quatRotateVec3(qLeftRocker, m_rockerToFrontLocal));
    m_state.wheelHubsWorld[1] = vec3Add(m_state.rightRockerPivotWorld, quatRotateVec3(qRightRocker, Vec3{-m_rockerToFrontLocal.x, m_rockerToFrontLocal.y, m_rockerToFrontLocal.z}));

    // Bogie pivots
    m_state.leftBogiePivotWorld = vec3Add(m_state.leftRockerPivotWorld, quatRotateVec3(qLeftRocker, m_rockerToBogieLocal));
    m_state.rightBogiePivotWorld = vec3Add(m_state.rightRockerPivotWorld, quatRotateVec3(qRightRocker, Vec3{-m_rockerToBogieLocal.x, m_rockerToBogieLocal.y, m_rockerToBogieLocal.z}));

    // Bogie orientations
    const Quat qLeftBogie = quatMultiply(qLeftRocker, makePitchQuat(m_state.leftBogiePitch));
    const Quat qRightBogie = quatMultiply(qRightRocker, makePitchQuat(m_state.rightBogiePitch));
    m_state.leftBogieOrientationWorld = qLeftBogie;
    m_state.rightBogieOrientationWorld = qRightBogie;

    // Mid hubs (ML idx 4, MR idx 5)
    m_state.wheelHubsWorld[4] = vec3Add(m_state.leftBogiePivotWorld, quatRotateVec3(qLeftBogie, m_bogieToMidLocal));
    m_state.wheelHubsWorld[5] = vec3Add(m_state.rightBogiePivotWorld, quatRotateVec3(qRightBogie, Vec3{-m_bogieToMidLocal.x, m_bogieToMidLocal.y, m_bogieToMidLocal.z}));

    // Rear hubs (RL idx 2, RR idx 3)
    m_state.wheelHubsWorld[2] = vec3Add(m_state.leftBogiePivotWorld, quatRotateVec3(qLeftBogie, m_bogieToRearLocal));
    m_state.wheelHubsWorld[3] = vec3Add(m_state.rightBogiePivotWorld, quatRotateVec3(qRightBogie, Vec3{-m_bogieToRearLocal.x, m_bogieToRearLocal.y, m_bogieToRearLocal.z}));

    // Corner steering knuckles (FL, FR on Rockers; RL, RR on Bogies)
    m_state.knuckleOrientationsWorld[0] = quatMultiply(qLeftRocker, quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, wheels[0].steeringAngle));
    m_state.knuckleOrientationsWorld[1] = quatMultiply(qRightRocker, quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, wheels[1].steeringAngle));
    m_state.knuckleOrientationsWorld[2] = quatMultiply(qLeftBogie, quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, wheels[2].steeringAngle));
    m_state.knuckleOrientationsWorld[3] = quatMultiply(qRightBogie, quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, wheels[3].steeringAngle));

    // 6. Dynamic Wheel Telemetry & Equal Load Distribution
    const double planetGravity = world ? static_cast<double>(world->getGravity().length()) : 3.71;
    const double effectiveGravity = (planetGravity > 0.1) ? planetGravity : 3.71;
    const double totalRoverWeight = config.mass * effectiveGravity;
    const double nominalPerWheelLoad = totalRoverWeight / 6.0;

    m_state.totalNormalLoad = 0.0;
    m_state.wheelsInContactCount = 0;

    // Pass 1: Wheel orientation, surface seating, contact queries & contact counting
    for (size_t i = 0; i < 6; ++i) {
        WheelState& w = wheels[i];
        const double r = w.wheelRadius;
        const double minY = groundY[i] + r;

        // Kinematic surface seating constraint: prevent wheel from penetrating into the physical terrain
        if (groundHits[i] && m_state.wheelHubsWorld[i].y < minY) {
            m_state.wheelHubsWorld[i].y = minY;
        }

        const Vec3& hubPos = m_state.wheelHubsWorld[i];
        w.wheelCenterWorld = hubPos;
        w.mountPointWorld = hubPos;

        // Decoupled steering pivot vs independent wheel spin axis
        const Quat spinQ = quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, w.rotationAngle);
        if (i == 0) {
            w.worldOrientation = quatMultiply(m_state.knuckleOrientationsWorld[0], spinQ);
        } else if (i == 1) {
            w.worldOrientation = quatMultiply(m_state.knuckleOrientationsWorld[1], spinQ);
        } else if (i == 2) {
            w.worldOrientation = quatMultiply(m_state.knuckleOrientationsWorld[2], spinQ);
        } else if (i == 3) {
            w.worldOrientation = quatMultiply(m_state.knuckleOrientationsWorld[3], spinQ);
        } else if (i == 4) {
            w.worldOrientation = quatMultiply(qLeftBogie, spinQ);
        } else if (i == 5) {
            w.worldOrientation = quatMultiply(qRightBogie, spinQ);
        }
        m_state.wheelOrientationsWorld[i] = w.worldOrientation;

        // Ground contact detection
        const double distToGround = hubPos.y - groundY[i];
        const double delta = std::max(0.0, (groundY[i] + r) - hubPos.y);
        w.compression = std::max(0.0, r - distToGround) + delta;
        w.inContact = groundHits[i] && (distToGround <= r + 0.12);

        if (w.inContact) {
            w.contactPointWorld = Vec3{hubPos.x, groundY[i], hubPos.z};
            w.contactNormalWorld = groundNormals[i];
            m_state.wheelsInContactCount++;
        } else {
            w.contactPointWorld = vec3Add(hubPos, vec3Scale(basis.down, r));
            w.contactNormalWorld = basis.up;
            w.normalLoad = 0.0;
        }
    }

    // Pass 2: Passive Rocker-Bogie statically determinate normal load distribution with contact restoring penalty
    const double perWheelActiveLoad = (m_state.wheelsInContactCount > 0)
        ? (totalRoverWeight / static_cast<double>(m_state.wheelsInContactCount))
        : nominalPerWheelLoad;

    constexpr double kContactStiffness = 60000.0; // N/m penetration restoring stiffness
    constexpr double kContactDamping   = 4000.0;  // N*s/m velocity damping

    for (size_t i = 0; i < 6; ++i) {
        WheelState& w = wheels[i];
        if (w.inContact) {
            const Vec3& hubPos = m_state.wheelHubsWorld[i];
            const double hubVelY = body.getVelocityAtWorldPoint(hubPos).y;
            const double delta = std::max(0.0, (groundY[i] + w.wheelRadius) - hubPos.y);

            const double springForce = kContactStiffness * delta;
            const double dampingForce = -hubVelY * kContactDamping;
            const double rawNormal = perWheelActiveLoad + springForce + dampingForce;

            w.normalLoad = std::max(0.0, rawNormal);
            m_state.totalNormalLoad += w.normalLoad;
        }
    }
}

} // namespace torquelab
