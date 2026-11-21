#pragma once
/**
 * PROJECT ARES — 6-Wheel Passive Rocker-Bogie Mobility System.
 *
 * Implements the authentic NASA Mars 2020 (Perseverance) & MSL (Curiosity)
 * rocker-bogie kinematic suspension system:
 * - Two main rocker arms (Left & Right) pivoting on the chassis trunnions.
 * - Two bogie arms (Left & Right) pivoting on the rear of each rocker arm.
 * - Central transverse differential bar coupling left and right rockers:
 *     theta_rocker_left ≈ -theta_rocker_right (chassis pitch isolation).
 * - 6 independent drive wheels (FL, FR, RL, RR, ML, MR):
 *     FL (Front-Left), FR (Front-Right), RL (Rear-Left), RR (Rear-Right),
 *     ML (Mid-Left), MR (Mid-Right).
 * - 4-corner independent steering actuators (FL, FR, RL, RR) with non-steered
 *   middle wheels (ML, MR).
 * - Equalized static normal load distribution: F_z,i ≈ (1/6) * M * g_planet.
 * - Multi-body terrain obstacle climbing up to 1.5–2 wheel diameters (40 cm).
 */

#include <array>
#include <string>
#include <btBulletDynamicsCommon.h>
#include "physics/Math3D.h"
#include "physics/SurfaceSystem.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/WheelState.h"

namespace torquelab {

class VehicleBody;
class TerrainManager;

struct RockerBogieState {
    // Linkage pitch angles relative to chassis local frame (radians)
    double leftRockerPitch{0.0};    // Pitch of left rocker arm (+ = front wheel up)
    double rightRockerPitch{0.0};   // Pitch of right rocker arm
    double leftBogiePitch{0.0};     // Pitch of left bogie arm relative to left rocker
    double rightBogiePitch{0.0};    // Pitch of right bogie arm relative to right rocker
    double differentialAngle{0.0};  // Differential bar tilt angle

    // Angular velocities (rad/s)
    double leftRockerAngVel{0.0};
    double rightRockerAngVel{0.0};
    double leftBogieAngVel{0.0};
    double rightBogieAngVel{0.0};

    // World-space pivot positions (m)
    Vec3 leftRockerPivotWorld{0.0, 0.0, 0.0};
    Vec3 rightRockerPivotWorld{0.0, 0.0, 0.0};
    Vec3 leftBogiePivotWorld{0.0, 0.0, 0.0};
    Vec3 rightBogiePivotWorld{0.0, 0.0, 0.0};
    Vec3 differentialPivotWorld{0.0, 0.0, 0.0};

    // World-space linkage orientations
    Quat leftRockerOrientationWorld{0.0, 0.0, 0.0, 1.0};
    Quat rightRockerOrientationWorld{0.0, 0.0, 0.0, 1.0};
    Quat leftBogieOrientationWorld{0.0, 0.0, 0.0, 1.0};
    Quat rightBogieOrientationWorld{0.0, 0.0, 0.0, 1.0};
    Quat differentialOrientationWorld{0.0, 0.0, 0.0, 1.0};
    std::array<Quat, 4> knuckleOrientationsWorld{}; // 0=FL, 1=FR, 2=RL, 3=RR

    // 6 Wheel hub positions in world space (m)
    // Indexing: 0=FL, 1=FR, 2=RL, 3=RR, 4=ML, 5=MR
    std::array<Vec3, 6> wheelHubsWorld{};
    std::array<Quat, 6> wheelOrientationsWorld{};

    // Contact telemetry
    int wheelsInContactCount{6};
    double totalNormalLoad{0.0};
};

class RockerBogieSuspension {
public:
    RockerBogieSuspension() = default;

    /**
     * Initializes mounting geometry from vehicle config.
     */
    void init(const VehicleConfig& config);

    /**
     * Solves the 6-wheel passive rocker-bogie kinematic and dynamic equilibrium.
     * Enforces:
     * 1. 6-point downward terrain probing across the 3 axles (Front, Mid, Rear).
     * 2. Passive bogie tilt balancing between middle and rear wheels.
     * 3. Passive rocker tilt balancing between front wheel and bogie pivot.
     * 4. Central differential bar coupling between left and right rockers.
     * 5. Multi-axle Ackermann steering for 4 corner wheels (mid wheels straight).
     * 6. Normal load equalization across all 6 wheels (~1/6 total weight).
     */
    void update(
        double dt,
        VehicleBody& body,
        const VehicleConfig& config,
        btDiscreteDynamicsWorld* world,
        SurfaceSystem* surfaceSystem,
        std::array<WheelState, 6>& wheels,
        double centerSteeringInput,
        const std::array<const btCollisionObject*, 6>& wheelObjsToIgnore,
        const TerrainManager* terrainManager = nullptr
    );

    const RockerBogieState& getState() const { return m_state; }

    // Hard-point geometry accessors (chassis local, resting pose)
    const Vec3& getDifferentialPivotLocal() const { return m_diffPivotLocal; }
    const Vec3& getLeftRockerPivotLocal() const { return m_leftRockerPivotLocal; }
    const Vec3& getRightRockerPivotLocal() const { return m_rightRockerPivotLocal; }
    const Vec3& getLeftBogiePivotLocal() const { return m_leftBogiePivotLocal; }
    const Vec3& getRightBogiePivotLocal() const { return m_rightBogiePivotLocal; }
    const Vec3& getRockerToFrontLocal() const { return m_rockerToFrontLocal; }
    const Vec3& getRockerToBogieLocal() const { return m_rockerToBogieLocal; }
    const Vec3& getBogieToMidLocal() const { return m_bogieToMidLocal; }
    const Vec3& getBogieToRearLocal() const { return m_bogieToRearLocal; }

private:
    RockerBogieState m_state{};

    // Local hardpoint offsets in chassis coordinate frame (m)
    // Chassis Frame: +X = Left, +Y = Up, +Z = Forward
    // Matched to millimeter precision from NASA CAD geometry
    Vec3 m_diffPivotLocal{ 0.000, 0.457, -0.114};
    Vec3 m_leftRockerPivotLocal{  0.806, 0.190,  0.249};
    Vec3 m_rightRockerPivotLocal{-0.806, 0.190,  0.249};

    // Rocker arm vector from rocker pivot to front wheel mount (chassis local)
    Vec3 m_rockerToFrontLocal{ 0.256, -0.400,  0.846};
    // Rocker arm vector from rocker pivot to bogie pivot (chassis local)
    Vec3 m_rockerToBogieLocal{ 0.100, -0.237, -0.754};

    // Bogie pivot local in chassis frame at rest
    Vec3 m_leftBogiePivotLocal{  0.906, -0.047, -0.505};
    Vec3 m_rightBogiePivotLocal{-0.906, -0.047, -0.505};

    // Bogie arm vector from bogie pivot to mid wheel mount (bogie local at rest)
    Vec3 m_bogieToMidLocal{  0.278, -0.163,  0.415};
    // Bogie arm vector from bogie pivot to rear wheel mount (bogie local at rest)
    Vec3 m_bogieToRearLocal{ 0.156, -0.163, -0.660};
};

} // namespace torquelab
