# GEMINI.md — MANTIS (AI Context & Project Memory)

> **Purpose of this file:** This document is the single-source-of-truth project memory for **Gemini** and other AI coding agents. Read this file **first** when starting a new session or switching models—it provides the complete architecture, file map, coordinate/unit conventions, physics pipeline, hardware/driver quirks on this machine, and verified benchmark numbers so you do not need to scan the entire codebase.

---

## 1. Project Identity & Non-Negotiable Engineering Rules

- **Project Name:** **MANTIS** (*Mars Autonomous Navigation & Terrain Intelligent Simulator*)
- **Tech Stack:** Native **C++20**, **OpenGL 3.3 Core Profile**, **GLSL 330 core**, **GLFW 3.4**, **GLAD**, **Bullet Physics (`btDiscreteDynamicsWorld`)**, **Dear ImGui**, and **CMake** (Visual Studio 2026 / MSVC and MinGW-w64 UCRT64 GCC 16 Release builds).
- **Project Domain:** High-fidelity autonomous planetary rover mobility and multi-resolution extraterrestrial terrain simulation. All legacy road cars (BMW, Dodge, Hummer) have been completely eliminated. The active fleet comprises **NASA Perseverance Rover (Mars 2020)**, **NASA Curiosity Rover (MSL)**, **NASA Opportunity Rover (MER)**, and the **MANTIS Prototype Rover**.
- **Core Physical Rule (Non-Negotiable):**
  - **Every rover behavior must emerge from coherent physical laws** (forces, torques, mass, rotational inertia, tire-ground slip, friction-circle saturation, suspension compression/rebound, terramechanics, and collision impulse transfer).
  - **NEVER** introduce fake animations, arbitrary speed multipliers, unphysical engine power multipliers, or unjustified gravity hacks.

---

## 2. Quick Commands (Build, Run, Test, Diagnostics)

All commands run from the workspace root: `d:\# My Folder\My Work\Visual Studio script\C++\ProjectAres`

### MinGW-w64 (GCC 16 / UCRT64) Workflow:
```powershell
# 1. Configure in build/ (instant offline configuration via FETCHCONTENT_FULLY_DISCONNECTED)
cmake --preset windows-mingw
# or manually:
cmake -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_FULLY_DISCONNECTED=ON

# 2. Build both the main application and the automated test runner (Release)
cmake --build build --target MANTIS MANTISTests -j 6

# 3. Run interactive desktop application (1440x900 OpenGL 3.3 window)
.\build\MANTIS.exe

# 4. Run the 58-suite automated unit, validation & regression test harness
.\build\MANTIS.exe --tests
# or via standalone test binary:
.\build\MANTISTests.exe

# 5. Run headless OpenGL + physics smoke test
.\build\MANTIS.exe --smoke-test

# 6. Run full-pipeline acceleration telemetry table + 600-frame live OpenGL timing diagnostics
.\build\MANTIS.exe --diag
```

### Visual Studio 2026 / MSVC Workflow:
```powershell
# 1. Configure in build/
cmake --preset windows-msvc

# 2. Build Release targets
cmake --build build --config Release --target MANTIS MANTISTests -j 6

# 3. Run from build/Release/
.\build\Release\MANTIS.exe
.\build\Release\MANTIS.exe --tests
.\build\Release\MANTISTests.exe
```

---

## 3. Coordinate System, Units & Sign Conventions

All physics and rendering code operates strictly in **SI metric units (`m`, `kg`, `s`, `N`, `N·m`, `rad`, `rad/s`)** in a **Right-Handed World & Chassis Coordinate Frame**:

| Quantity / Axis | Convention | Sign / Notes |
| :--- | :--- | :--- |
| **Chassis Local $+X$** | **Left** (`basis.left`) | Front-Left (`FL`) & Rear-Left (`RL`) mounts have $x = +\text{trackWidth}/2 = +0.79\text{ m}$ |
| **Chassis Local $+Y$** | **Up** (`basis.up`) | Gravity acts along World $-Y$ ($g = 9.81\text{ m/s}^2$) |
| **Chassis Local $+Z$** | **Forward** (`basis.forward`) | Front axle is at $z > 0$, rear axle is at $z < 0$ |
| **Wheel Indexing `[0..3]`** | `0 = FL`, `1 = FR`, `2 = RL`, `3 = RR` | Fixed across `VehicleDynamics`, `Drivetrain`, `BrakeSystem`, `VehicleRendererGL`, and `ImGuiDashboard` |
| **Pitch Angle (`pitchRad`)** | Extracted via `getVehicleEulerAngles()` | **Positive ($>0$) = Rear Squat / Nose-Up** (under acceleration); **Negative ($<0$) = Front Dive / Nose-Down** (under braking) |
| **Roll Angle (`rollRad`)** | Extracted via `getVehicleEulerAngles()` | **Positive ($>0$) = Right Body Roll** (during a left turn with $+X$ lateral acceleration) |
| **Steering Angle** | `centerSteeringAngle`, `w.steeringAngle` | **Positive ($>0$) = Left Turn** (`A` / `Left Arrow`); **Negative ($<0$) = Right Turn** (`D` / `Right Arrow`) |

---

## 4. Complete Workspace File Map

### `src/physics/` — Math Primitives, Bullet World & Surface Registry
- [`src/physics/Math3D.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/Math3D.h): Double-precision `Vec3`, `Quat`, column-major float `Mat4` (for OpenGL), `VehicleBasis` (`forward`, `up`, `left`, `right`, `down`), `EulerAngles` (`yaw`, `pitch`, `roll`), and scalar helpers (`clamp`, `lerp`, `degToRad`, `radToDeg`).
- [`src/physics/CollisionMeshGeometry.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/CollisionMeshGeometry.h): CPU-retained raw mesh storage (`RawMeshGeometry`), convex decomposition pieces (`ConvexDecompositionPiece`), collision pipeline configuration (`CollisionPipelineSettings`), strategy enum (`CollisionAlgorithmStrategy`), and generated collision container (`GeneratedVehicleCollision`).
- [`src/physics/VehicleCollisionGenerator.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/VehicleCollisionGenerator.h) & [`VehicleCollisionGenerator.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/VehicleCollisionGenerator.cpp): Generic model-derived collision generation engine. Performs spatial voxel decimation, V-HACD volumetric decomposition, spatial topological clustering via `btConvexHullComputer`, automatic vehicle geometry derivation (wheelbase, track width, wheel radius, ground clearance), disk caching (`model/cache/<id>_collision.json`), and Bullet `btCompoundShape` synthesis.
- [`src/physics/SurfaceSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/SurfaceSystem.h) & [`SurfaceSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/SurfaceSystem.cpp): Defines the 6 road surfaces (`DryAsphalt` $\mu=1.00$, `WetAsphalt` $\mu=0.68$, `Gravel` $\mu=0.54$, `Ice` $\mu=0.18$, `Grass` $\mu=0.42$, `RumbleStrip` $\mu=0.88$) with friction, rolling-resistance, cornering-stiffness, and procedural roughness parameters. Maps Bullet collider `userIndex` handles to surface properties.
- [`src/physics/PhysicsWorld.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/PhysicsWorld.h) & [`PhysicsWorld.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/PhysicsWorld.cpp): Owns `btDiscreteDynamicsWorld`, fixed `120 Hz` (`fixedDt = 1/120 s`, `maxSubSteps = 12`) accumulator loop (`updateFrame`, `stepFixed`), the Proving Ground track layout (`buildProvingGround`), and the 7 teleport spawn presets (`getProvingGroundPresets`). Supports switching vehicles with model-derived collision shapes.

### `src/vehicle/` — 6-DOF Vehicle Dynamics Subsystems
- [`src/vehicle/VehicleConfig.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleConfig.h) & [`VehicleConfig.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleConfig.cpp): `VehicleConfig` presets:
  - `createBmwM4Config()` (Factory M4 Coupe: S55/S58 3.0L twin-turbo I6, $550\text{ N}\cdot\text{m}$, 6-speed RWD, mass $1600\text{ kg}$, wheelbase $2.76\text{ m}$, track $1.58\text{ m}$, 220 kPa front / 240 kPa rear pneumatic pressures, 0.24m rim radius).
  - `createDodgeChallengerConfig()` (Authentic 1970 Dodge Challenger R/T: 426 cu in Street Hemi V8, $664\text{ N}\cdot\text{m}$ peak torque, 4-speed manual gearbox, Dana 60 Sure-Grip LSD 3.54:1, mass $1680\text{ kg}$, 54% front distribution, wheelbase $2.794\text{ m}$ [$110.0\text{ in}$], track $1.56\text{ m}$, vintage muscle bias-ply tires and softer rear ARB for authentic launch squat, 195 kPa / 210 kPa pressures, 0.19m rim radius).
  - `createHummerConfig()` (Authentic Hummer H2: GM Vortec 6.2L V8 $565\text{ N}\cdot\text{m}$, 6-speed automatic 6L80, Full-Time AWD with 4.10:1 axle ratio, mass $2900\text{ kg}$, 53% front distribution, wheelbase $3.128\text{ m}$, track $1.731\text{ m}$, high-clearance long-travel off-road suspension, 34-inch all-terrain tires [315/70R17, $R=0.430\text{ m}$, $W=0.320\text{ m}$], 250 kPa front / 260 kPa rear pressures, 0.22m rim radius).
  - `createProceduralCoupeConfig()` (`mass = 1500 kg`, `wheelbase = 2.70 m`, `trackWidth = 1.60 m`, `frontWeightDistribution = 0.52`, 220 kPa / 230 kPa pressures, 0.23m rim radius).
- [`src/vehicle/PneumaticTire.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/PneumaticTire.h) & [`PneumaticTire.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/PneumaticTire.cpp): Full structural pneumatic tyre simulation engine. Implements Kelvin-Voigt viscoelastic carcass compliance, pressure-dependent radial/lateral/longitudinal stiffness ($k_r(p) = k_{r,0} + C_p (p - p_0)$), multi-point contact patch discretization across circumferential and lateral grids with Hertzian-like parabolic weighting, series suspension/tyre compliance integration, lateral carcass cornering shear $\delta_y$, longitudinal shear $\delta_x$, Poisson sidewall bulge ($\Delta w = \nu \cdot \delta_r$), pneumatic trail self-aligning torque ($M_z = t_p \cdot F_y$), thermal energy balance (frictional dissipation + hysteresis heating vs convective/conductive cooling), tread wear accumulation, and puncture air leakage dynamics.
- [`src/vehicle/WheelState.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/WheelState.h): Per-wheel state (`WheelState`) storing raycast contact, suspension compression/velocity/forces, slip ratio $\kappa$, slip angle $\alpha$, longitudinal/lateral tire forces, angular velocity $\omega$, rotation angle, drive/brake torques, and the physical `PneumaticTireModel` instance.
- [`src/vehicle/VehicleBody.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleBody.h) & [`VehicleBody.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleBody.cpp): Wraps the chassis `btRigidBody` + `btCompoundShape` (lower body box + upper greenhouse box offset relative to the configurable Center of Mass). Supports live CoM and mass updates (`syncMassProperties`), world-point force application (`applyForceAtWorldPoint`), and exact analytical + Bullet raycasting (`castRayExactWorld`).
- [`src/vehicle/SuspensionSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SuspensionSystem.h) & [`SuspensionSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SuspensionSystem.cpp): Hooke spring ($F_s = -k \Delta x$), asymmetric bound/rebound damper ($c_{\text{bound}} \neq c_{\text{rebound}}$), progressive quadratic bump stop, and front/rear axle anti-roll bars (`computeAxleAntiRollForce`).
- [`src/vehicle/TireModel.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/TireModel.h) & [`TireModel.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/TireModel.cpp): Longitudinal slip ratio $\kappa$, lateral slip angle $\alpha$, load-sensitive friction coefficient $\mu(F_z)$, Pacejka Magic Formula $y(x) = D \sin(C \arctan(Bx - E(Bx - \arctan(Bx))))$, and combined-slip friction-circle saturation ($\sqrt{F_x^2 + F_y^2} \le \mu F_z$).
- [`src/vehicle/Drivetrain.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Drivetrain.h) & [`Drivetrain.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Drivetrain.cpp): Piecewise-linear engine torque curve (`sampleEngineTorqueCurve`), closed-throttle engine braking (`computeEngineBrakingTorque`), automatic/manual shift state machine, launch clutch slip engagement, soft fuel-cut rev limiter at `maxRpm`, RWD/FWD/AWD layout distribution, and speed-sensing Limited-Slip Differential (`distributeAxleTorqueWithLSD`).
- [`src/vehicle/SteeringSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SteeringSystem.h) & [`SteeringSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SteeringSystem.cpp): Speed-sensitive steering limit and Ackermann inner/outer front wheel geometry (`computeAckermannWheelAngles`).
- [`src/vehicle/BrakeSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/BrakeSystem.h) & [`BrakeSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/BrakeSystem.cpp): Front/rear biased service brake torques (`brakeBiasFront = 0.64`) and rear-only mechanical handbrake (`handbrakeTorque = 3400 N·m`).
- [`src/vehicle/Aerodynamics.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Aerodynamics.h) & [`Aerodynamics.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Aerodynamics.cpp): Quadratic aerodynamic drag ($F_d = \frac{1}{2}\rho C_d A v^2$), aerodynamic downforce ($F_L = \frac{1}{2}\rho C_L A v^2$), and per-wheel rolling resistance.
- [`src/vehicle/VehicleDynamics.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleDynamics.h) & [`VehicleDynamics.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleDynamics.cpp): Master 6-DOF vehicle orchestrator (`step`, `postWorldStep`, `getInterpolatedTransforms`, `getTelemetrySnapshot`, `resetVehicle`).

### `src/renderer/`, `shaders/`, `src/ui/`, `src/input/`, `src/app/`
- [`src/renderer/Shader.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Shader.h) & [`Shader.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Shader.cpp): GLSL shader compiler/linker with uniform location caching.
- [`src/renderer/Mesh.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Mesh.h) & [`Mesh.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Mesh.cpp): VAO/VBO/EBO procedural mesh generators (boxes, cylinders, rings, planes).
- [`src/renderer/ModelLoaderGLB.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ModelLoaderGLB.h) & [`ModelLoaderGLB.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ModelLoaderGLB.cpp): Single-header `cgltf`-based binary glTF 2.0 / GLB loader. Parses multi-node hierarchies into OpenGL VAOs, maps nodes to semantic vehicle roles (Chassis, 4 independent wheel assemblies, calipers, steering wheel), parses PBR metallic/roughness & transparent glass materials, reads optional JSON manifests (`model/BMW_M4_manifest.json`, `model/Dodge_Challenger_manifest.json`, `model/HUMMER_manifest.json`), handles negative scale / reflection matrices by automatic index winding reversal ($i_1 \leftrightarrow i_2$), prevents bodywork transparency leakage with explicit material filtering, supports embedded binary glTF textures, root scale & yaw rotation, and automatic multi-wheel quadrant partitioning (`ModelPartRole::Wheel_All`) for models whose wheels were authored as a single merged mesh.
- [`src/renderer/VehicleAssetValidator.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleAssetValidator.h) & [`VehicleAssetValidator.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleAssetValidator.cpp): Automated diagnostic validation engine for imported 3D vehicle assets. Checks geometric integrity (degenerate faces, normal/tangent validity, negative scale detection, automotive bounding box proportions), material & transparency consistency (zero tolerance for body panel transparency leakage, glass isolation, double-sided culling), and kinematic hierarchy sanity (4-quadrant wheel identification, wheelbase/track boundaries). Generates formal multi-section diagnostic reports.
- [`docs/BLENDER_VEHICLE_EXPORT_GUIDELINES.md`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/docs/BLENDER_VEHICLE_EXPORT_GUIDELINES.md): Comprehensive production technical guide for 3D vehicle artists covering object naming conventions, applying transforms (`Ctrl+A`), normal orientation (`Shift+N`), PBR metallic/roughness materials, transparent glass isolation, double-sided thin sheet metal handling, and glTF 2.0 export settings.
- [`src/renderer/ShadowMapFBO.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ShadowMapFBO.h) & [`ShadowMapFBO.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ShadowMapFBO.cpp): `2048x2048` depth framebuffer for directional sun shadows.
- [`src/renderer/TrackRendererGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/TrackRendererGL.h) & [`TrackRendererGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/TrackRendererGL.cpp): Procedural grid/surface textures, Proving Ground meshes, 25m distance marker posts, and handling ring.
- [`src/renderer/VehicleRendererGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleRendererGL.h) & [`VehicleRendererGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleRendererGL.cpp): Master vehicle renderer with hot-swapping between multiple loaded 3D models (`BMW M4 Coupe`, `1970 Dodge Challenger R/T`, `Hummer H2`, and the procedural sports coupe). Supports 2-pass rendering (opaque + transparent glass), dynamic back-to-front camera distance sorting for transparent glass and light lenses, dynamic backface culling toggling for double-sided thin sheet metal panels, reactive brake/reverse lights, suspension struts, and a 3D visual alignment debug overlay (`renderVisualDebug`).
- [`src/renderer/DebugOverlayGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/DebugOverlayGL.h) & [`DebugOverlayGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/DebugOverlayGL.cpp): 3D force vector gizmos (normal load $F_z$, longitudinal $F_x$, lateral $F_y$, velocity vector, and CoM marker) + rear-wheel dynamic skidmarks.
- [`src/renderer/CameraControllerGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/CameraControllerGL.h) & [`CameraControllerGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/CameraControllerGL.cpp): 4 camera modes (`Follow`, `Orbit`, `Suspension`, `Hood`).
- [`src/ui/ImGuiDashboard.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/ui/ImGuiDashboard.h) & [`ImGuiDashboard.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/ui/ImGuiDashboard.cpp): Top bar (zone presets `1..7`, interactive car selection dropdown combo, camera/gizmo/telemetry toggles, physics verifier button), left live tuning panel (vehicle selection dropdown + radio buttons, alignment debug checkbox, 12 real-time sliders + drivetrain layout selector), right telemetry panel (force balance summary, 4-wheel table, 4-band real-time oscilloscope), and bottom instrument cluster (digital speedometer, RPM bar, pedal bars, G-G diagram).
- [`src/input/ControllerConfig.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/ControllerConfig.h) & [`ControllerConfig.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/ControllerConfig.cpp): Logical vehicle action enum, Forza Horizon default mappings, Xbox/PlayStation/Nintendo button glyph adaptivity, and persistent JSON configuration (`settings/controller_config.json`).
- [`src/input/VibrationManager.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/VibrationManager.h) & [`VibrationManager.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/VibrationManager.cpp): Dynamic Windows XInput force-feedback rumble pipeline. Purely event-driven haptics inspired by Forza Horizon: collision impacts with non-linear impulse scaling & lateral directionality, suspension events (jump landings, bump-stop bottom-out, sharp curb strikes), underside chassis scrape buzzing, smooth cosine envelope decay, and collision deduplication. Zero vibration from engine RPM, throttle, or smooth cruising.
- [`src/input/InputControllerGLFW.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/InputControllerGLFW.h) & [`InputControllerGLFW.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/InputControllerGLFW.cpp): Progressive keyboard + baseline-calibrated multi-gamepad input handler with continuous analog steering, trigger normalization, right-stick look-around, interactive rebinding, hotplugging, and car cycling (`RB` / `B`).
- [`src/app/Application.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/Application.h), [`Application.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/Application.cpp), [`main.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/main.cpp): Lifecycle manager and CLI entry point (`--smoke-test`, `--diag`).
- [`tests/native_validation_tests.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/tests/native_validation_tests.cpp): 43 automated test suites (`Unit 1–30`, `Validation Tests A–J`, `Controlled Tests 1–3`).

---

## 5. Critical Machine / Hardware Context & Resolved Gotchas (DO NOT REGRESS)

1. **Virtual HID Gamepad on Host PC (`Monect Hid Device`):**
   - On this machine, `glfwJoystickIsGamepad(GLFW_JOYSTICK_1)` returns `1` for `'Monect Hid Device'`, whose idle axes all report `0.000` (`LX=0, LY=0, RX=0, RY=0, LT=0, RT=0`).
   - **Why it matters:** Standard XInput triggers rest at `-1.0f`. Naive `(axis + 1.0) * 0.5` mapping turns `LT = 0.000` into a permanent **50% brake input (`brake = 0.50`)**, locking the vehicle at `0.03 m/s`!
   - **How it is solved in `InputControllerGLFW`:** We capture the initial resting baseline of all 6 gamepad axes on connection (`m_gamepadBaselineAxes`) and only mark an axis active (`m_gamepadAxisActivated[i] = true`) after it deviates by `> 0.12f` from its resting value. **Never remove this baseline activation guard.**

2. **Low-Speed Parking Hold Guard in `VehicleDynamics::step()`:**
   - The stationary parking-hold damper (`if (std::hypot(vx, vy) < 0.8 && w.brakeTorque > 50.0 && effectiveThrottle < 0.05)`) **must** keep the `effectiveThrottle < 0.05` condition so braking hold never opposes active throttle launch.

3. **Coupled Driven-Wheel Reflected Engine Inertia & Soft Redline Limit:**
   - In `VehicleDynamics::step()`, driven wheels include reflected engine inertia (`0.25 * clutchEngagement * engineInertia * totalRatio^2 / numDrivenWheels`) when accelerating and smoothly taper positive `driveTorque` as wheel angular speed approaches $\omega_{\text{redline}} = \frac{\text{maxRpm} \cdot \pi / 30}{|G_{\text{total}}|}$.
   - In `Drivetrain::updateDrivetrain()`, when `state.revLimiterActive` is true and `throttle > 0.05`, `state.engineTorqueNm` cuts cleanly to `0.0` instead of slamming full closed-throttle vacuum braking (`-95.8 N·m`).

4. **Multi-GPU / Discrete GPU Auto-Detection & High-Performance Routing (`NvOptimusEnablement` & `AmdPowerXpress`):**
   - On dual-GPU laptops and hybrid graphics systems (e.g., host machine with **`NVIDIA GeForce RTX 5050 Laptop GPU`** and integrated **`Intel(R) UHD Graphics`**), Windows and graphics drivers default to running OpenGL applications on the integrated GPU unless explicitly instructed otherwise.
   - **How it is solved:** `src/app/main.cpp` exports the driver hooks:
     ```cpp
     extern "C" {
         __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
         __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
     }
     ```
     This forces the NVIDIA Optimus and AMD PowerXpress/Enduro drivers to route the OpenGL context to the dedicated high-performance external/discrete GPU on startup.
   - **GPU Hardware Detection Subsystem (`src/renderer/GpuHardwareInfo.h` & `GpuHardwareInfo.cpp`):**
     On initialization, `GpuHardwareInfo::detect()` queries the active OpenGL device (`GL_RENDERER`, `GL_VENDOR`, `GL_VERSION`, `GL_SHADING_LANGUAGE_VERSION`), queries VRAM via `GL_NVX_gpu_memory_info` (NVIDIA) and `GL_ATI_meminfo` (AMD), enumerates all hardware display adapters via Windows DXGI (`dxgi.dll`), classifies the active GPU (Discrete/External vs Integrated), prints a clean diagnostic ASCII banner to console on launch, and surfaces live GPU metrics (Active Device, Driver, Dedicated VRAM, Multi-GPU Topology) in Dear ImGui across the Top Bar, the Left Tuning Panel, and the Right Telemetry Panel.
   - **Driver Performance Notes:** Keep `glfwWindowHint(GLFW_SAMPLES, 0)` and `glfwSwapInterval(0)` in `Application::init()`, and always unbind `GL_TEXTURE1` before `m_shadowFBO.beginPass()` in `Application::renderFrame()`. On the discrete NVIDIA RTX 5050, the application delivers **`1000–1140+ FPS` (`~0.8 ms` frame time)**.

5. **Chase Camera (`CameraMode::Follow`) Heading Damping:**
   - `CameraControllerGL::update()` damps the horizontal forward direction vector (`m_smoothedForward`) and vertical height (`m_smoothedHeight`) around the vehicle's interpolated position `pos` rather than lerping `m_position` in world space. This prevents steady-state camera distance lag at high speed (`CamDist` remains `6.61 m` at all speeds).

6. **Model-Derived Collision Geometry & Full 10-Hull Chassis/Wheel Alignment:**
   - Raw glTF mesh vertices extracted from Blender models have their origin at ground level $(0,0,0)$ and may include complex scene graph parent node hierarchies (armatures, collections, and scale factors).
   - In `ModelLoaderGLB.cpp`, all nodes are resolved via `cgltf_node_transform_world(node, worldMatrix)`, ensuring subparts with ancestor scales (such as bumper honeycombs and windshield wipers) are placed in exact sub-centimeter positions rather than floating off in space.
   - Wheel assemblies are extracted into dedicated tight wheel collision hulls (`WheelFL_Hull`, `WheelFR_Hull`, `WheelRL_Hull`, `WheelRR_Hull`), each filtering outlier hub links to form true cylindrical $0.30\text{ m}$ radius, $0.24\text{ m}$ width wheel colliders.
   - 100% of the imported car geometry is converted into 10 watertight convex collision hulls: 6 chassis body pieces (`FrontBumper_Hull`, `HoodFenders_Hull`, `Greenhouse_Hull`, `ChassisTub_Hull`, `RearDeck_Hull`, `RearBumper_Hull`) plus 4 dedicated wheel/tire hulls.
   - In `VehicleCollisionGenerator::generateFromModel()`, the raw chassis meshes are transformed into **Chassis Physics Space** via `Mat4::fromTranslation(model.chassisCenterOffset)` before volumetric convex decomposition.
   - This ensures child hulls in Bullet's `btCompoundShape` align 1:1 with the visual car body at $(T_{\text{chassis}} \times P_{\text{piece}})$, eliminating floating collision cages and bumper phasing.
   - Continuous Collision Detection (`setCcdMotionThreshold(0.08f)` and `setCcdSweptSphereRadius(0.20f)`) is active on `m_rigidBody`, preventing high-speed obstacle tunneling.
   - **Collision Bouncing Elimination:** Bullet solver is tuned with `m_splitImpulse = 1` (position-only contact push-out without velocity addition), `m_globalCfm = 0.0f` (eliminates spongy penetration), and chassis restitution $0.02f$. Suspension raycasts in `VehicleDynamics` skip non-driveable obstacle walls (`surfaceSystem.isColliderDriveable`), and bump-stop penetration is clamped to avoid upward load spikes on impact.

---

7. **Physics Event-Driven Controller Haptics (RPM Vibration Elimination):**
   - Controller vibration is strictly event-driven. Engine RPM, engine pitch/audio, throttle application, wheelspin slip, and smooth cruising produce **strictly 0.0% vibration**.
   - Haptics are triggered exclusively by:
     - **Barrier/Obstacle Collisions:** Derived from rigid-body velocity jump $|\Delta \vec{v}|$, contact normal impulse $J$, and contact lifetime. Resting contact against a wall settles to 0.0% rumble after initial impact. Collision deduplication suppresses contact chatter.
     - **Suspension Events:** Hard jump landings ($v_{\text{down}} > 0.6\text{ m/s}$, heavy motor), progressive bump-stop bottoming out ($w.\text{bumpStopForce} > 800\text{ N}$, heavy motor), sharp curb/bump strikes ($dL/dt < -2.2\text{ m/s}$, light motor), and underside chassis scrapes (light motor buzz).
     - **Lateral Directionality:** Left-side impacts bias toward the left vibration motor; right-side impacts bias toward the right motor.
     - **Decay & Controls:** Cosine envelope decay with master intensity, collision sensitivity, and suspension sensitivity sliders in ImGui and `settings/controller_config.json`.

8. **Atmospheric Environment Overhaul & Graphics Quality Presets:**
   - **Atmospheric Sky Dome & Color Calibration (`SkyRendererGL`):** Renders a procedural daytime sky dome at far clip depth ($z = w$, `GL_LEQUAL`, `glDepthMask(GL_FALSE)`). Calibrated Rayleigh scattering constants produce rich sapphire zenith (`rgb(0.04, 0.16, 0.65)`), clear azure mid-sky (`rgb(0.12, 0.38, 0.85)`), and luminous cerulean horizon blue (`rgb(0.38, 0.62, 0.94)`) with zero dull gray haze. Mie scattering models a sharp solar disc and multi-octave aureole corona.
   - **Double Gamma Elimination:** The Narkowicz ACES filmic tone reproduction curve (`acesToneMap`) natively maps linear HDR inputs to non-linear sRGB display output. Applying an extra `pow(x, 1/2.2)` on top resulted in double gamma compression (lifting dark shadows to 0.13 gray, flattening contrast, and washing out color saturation). Removing the redundant second gamma pass across both `phong_shadow.frag` and `sky.frag` restored deep, vibrant colors and authentic contrast.
   - **glTF Decal Alpha Compositing:** Vehicle liveries with transparent backgrounds (e.g. BMW M4 M-stripes and decals) blend over underlying body paint via `mix(uBaseColor, texCol.rgb, texCol.a)` rather than multiplicative blending (`albedo *= texCol.rgb; alpha *= texCol.a;`), preventing transparent decal boundaries from turning body paint black or invisible.
   - **Foreground Fog Exclusion (0–400m):** Distance fog starts strictly at $400\text{ m}$ (`fogDist = max(dist - 400.0, 0.0)`), leaving the vehicle, straightaway, skidpad, and track markers 100% fog-free and razor sharp. Beyond $400\text{ m}$, distant terrain skirts smoothly and seamlessly dissolve into the sky dome horizon using tonemapped horizon color (`acesToneMap(uFogColor * exposure)`).
   - **PBR Direct Light Energy Balance:** Direct diffuse lighting preserves unscaled sunlight irradiance ($L_o = (1 - k_S) \cdot \text{diffuse} \cdot \vec{L}_{\text{sun}} \cdot (\vec{N} \cdot \vec{L})$), avoiding rogue $1/\pi$ light reduction when sun color is supplied as illuminance, and automotive metallic paint basecoat diffuse is preserved under clearcoat (`diffuseColor = albedo * (1.0 - metalness * 0.75)`).
   - **Graphics Quality Presets (`GraphicsQuality`):**
     - **`Maximum`:** Full atmospheric Rayleigh/Mie sky dome, $2048 \times 2048$ shadow map with 8-tap Poisson disk PCF filtering, full HDR IBL reflections + diffuse irradiance, anti-banding dither, and expansive horizon skirt.
     - **`High`:** Baseline environment, $2048 \times 2048$ shadow map with 4-tap Poisson disk PCF, standard IBL, atmospheric sky dome.
     - **`Medium`:** Performance-balanced, $1024 \times 1024$ shadow map with fast 2-tap PCF, 3-stop sky gradient, basic IBL.
     - **`Low`:** Fast fallback, dynamic shadows disabled, fast 2-stop sky gradient, hemispherical ambient lighting without expensive IBL texture lookups.
   - **Runtime Hot-Swapping:** Live switching via Dear ImGui top-bar combo box with zero memory leaks, shader recompile hitches, or GPU stalls.

---

## 6. Verified Physical Benchmarks (`28 / 28` Suites Passing)

Any future code change must keep all 28 suites in `.\build\Release\TorqueLabNativeTests.exe` green:

- **Static Equilibrium (`Test A`):** $\sum F_z = 14715.0\text{ N}$ ($0.000\%$ error vs $mg = 1500 \times 9.81 = 14715.0\text{ N}$), ride height $= 0.4910\text{ m}$, suspension compression $= 0.1093\text{ m}$.
- **Full-Throttle Launch (`Test B` & `--diag`):**
  - At $t = 2.0\text{ s}$: $v = 10.60\text{ m/s}$ ($38.2\text{ km/h}$), $a_x = +5.34\text{ m/s}^2$, rear squat pitch $= +0.798^\circ$, rear axle load transfer $\Delta F_{z,\text{rear}} = +1312\text{ N}$.
  - $0\text{--}100\text{ km/h}$ ($27.78\text{ m/s}$) achieved in **$5.84\text{ s}$**; reaches **$143.6\text{ km/h}$ ($39.88\text{ m/s}$)** at $t = 10.0\text{ s}$ ($Z = 229.29\text{ m}$).
- **Service Braking (`Test C`):** From $20\text{ m/s}$, $v(0.5\text{ s}) = 16.94\text{ m/s}$, deceleration $a_x = -9.82\text{ m/s}^2$, nose-down dive pitch $= -1.609^\circ$, $F_{z,\text{front}} = 9970\text{ N}$ vs $F_{z,\text{rear}} = 4704\text{ N}$.
- **Cornering Roll (`Test D`):** Left turn roll $= +2.30^\circ$ ($a_y = +9.70\text{ m/s}^2$); Right turn roll $= -2.27^\circ$ ($a_y = -9.55\text{ m/s}^2$).
- **Surface Hierarchy (`Test E`):** Dry asphalt launch $v = 6.01\text{ m/s}$, stopping distance $= 11.36\text{ m}$ vs Ice launch $v = 1.24\text{ m/s}$, stopping distance $= 51.34\text{ m}$.
- **Timestep Independence (`Test I`):** $30\text{ FPS}$, $60\text{ FPS}$, and $120\text{ FPS}$ produce identical trajectories ($Z = 9.126\text{ m}, V = 9.790\text{ m/s}$ after $2.0\text{ s}$).
- **Controlled Force & Scale Sanity (`Controlled 1–3`):**
  - $F = 5800\text{ N}$ on $m = 1500\text{ kg}$ $\implies a = 3.868\text{ m/s}^2$ (expected $3.867\text{ m/s}^2$).
  - Constant $v = 25.00\text{ m/s}$ for $t = 2.000\text{ s}$ $\implies \Delta Z = 50.002\text{ m}$ (expected $50.000\text{ m}$).

---

## 8. Automated Compile-Time & Runtime Collision Baking Pipeline

1. **Automatic Build-Time Collision Generation (`CMakeLists.txt` Post-Build Custom Command):**
   - Whenever compiling (`cmake --build build --config Release`), CMake runs a post-build step: `TorqueLabNative --bake-collisions`.
   - The baker operates **100% headlessly on pure CPU** (`loadGpuResources = false` in `ModelLoaderGLB::loadFromFile`), requiring zero OpenGL or window initialization.
   - It scans `model/` for all `.glb` and `.gltf` files, checking their timestamps against `model/cache/<vehicle_id>_collision.json`.
   - If a model is new or was recently modified/exported from Blender (`last_write_time(glb) > last_write_time(cache)` or companion manifest was updated), it automatically executes volumetric convex decomposition (`btConvexHullComputer` / V-HACD), derives vehicle metrics (wheelbase, track width, wheel radius, ground clearance), writes the updated JSON cache, and syncs it to the build output directory.
   - If up-to-date, it skips re-computation in $<1\text{ ms}$.

2. **Runtime Stale Cache Invalidation:**
   - At runtime, `VehicleCollisionGenerator::generateFromModel()` executes the same timestamp validation. If a user modifies or drops in a `.glb` while running or between runs without recompiling, the game detects the stale cache and regenerates it on the fly.

3. **Dynamic Multi-Vehicle Auto-Discovery & Selection:**
   - `VehicleRendererGL::init()` dynamically scans `model/` for all `.glb` files and registers them into `m_availableVehicles` alongside `"procedural_coupe"`.
   - The Dear ImGui top-bar dropdown combo, left tuning panel combo, and radio buttons dynamically render all discovered vehicles.
   - Gamepad `RB` and Keyboard `B` dynamically cycle through all available vehicles.
   - Any newly added car model immediately receives physical collision hulls, derived wheel centers, wheelbase, and track width without requiring hardcoded vehicle configurations. Custom powertrain/mass parameters can be live-tuned and saved to `model/<id>_vehicle.json`.

1. **Decoupled Vehicle Configuration System (`VehicleConfig.h` / `VehicleConfig.cpp`):**
   - Pure physical parameters (`mass`, `wheelbase`, `trackWidth`, `inertiaTensor`, spring/damper rates, drivetrain, brakes) are decoupled from rendering models.
   - Per-vehicle JSON specifications stored in `model/bmw_m4_vehicle.json` and `model/procedural_coupe_vehicle.json`. Supports runtime serialization (`saveVehicleConfigToJson`) and deserialization (`loadVehicleConfigFromJson`).
   - Seamless live vehicle switching via `PhysicsWorld::switchVehicle(newConfig)` with zero memory leaks, full Bullet rigid body reconstruction, and suspension ray synchronization.

2. **Accurate Visual Appearance, Materials & Decal Composition:**
   - The Blender model is an **Alpine White BMW M4 with authentic BMW M tri-color racing stripes**, carbon fiber trim, chrome highlights, transparent tinted glass, and ruby red brake lights.
   - 4 packed textures extracted from `.blend` file into `model/textures/`:
     - `Front_Bumper_Text.png` (2048x2048 RGBA decal on front bumper)
     - `Front_Hodd_Text.png` (2048x2048 RGBA decal on hood)
     - `Stripes_Wings.png` (2048x2048 RGBA M-stripes on front fenders / doors)
     - `600px-BMW.svg.png` (600x600 RGBA BMW roundel emblem on wheels and hood)
   - Fragment shader (`phong_shadow.frag`) composites decal textures over Alpine White base paint (`#DCE0E6`) via `albedo = mix(albedo, texColor.rgb, texColor.a)`.

3. **Ground Clearance & Axis Alignment:**
   - Model origin $(0,0,0)$ in `BMW_M4.glb` sits at ground level under the tire contact patches.
   - Offset `visualOffset = Vec3{0.0, -0.477, -0.1475}` aligns the model's chassis floor with exact real-world ground clearance ($12.7\text{ cm}$), sitting flush on the road with wheel hubs centered in the wheel arches.

4. **Tapered Compound Collision Hull (`CollisionShapeType::TaperedCompound`):**
   - Replaced oversized rectangular collision box with an aerodynamic tapered compound shape in `VehicleBody.cpp`:
     - **Central Passenger Tub:** $Z \in [-0.40 \cdot hz, +0.40 \cdot hz]$, width $2 \cdot hx$ ($1.84\text{ m}$)
     - **Tapered Front Nose & Curved Bumper (`btConvexHullShape`):** Tapers from $1.75\text{ m}$ down to $1.44\text{ m}$ at bumper corners with chamfered apex tip, eliminating the phantom 25–30 cm corner collision gap!
     - **Tapered Rear Bumper (`btConvexHullShape`):** Tapers to $1.55\text{ m}$ at rear bumper tip.
     - **Upper Cabin Greenhouse:** Half-extents matching glass cabin envelope.
   - Real-time 3D wireframe visualization toggle in `DebugOverlayGL` (`showCollisionWireframe`) renders the exact Bullet collision boundary cage in emerald green.

5. **Generic Model-Derived Shape-Accurate Vehicle Collision Pipeline (`VehicleCollisionGenerator`):**
   - Replaces hardcoded box configurations with a generic automatic collision generation pipeline derived directly from loaded 3D vehicle models (GLTF/GLB or procedural meshes).
   - Dynamic moving rigid bodies in Bullet strictly require convex shapes or compound shapes of convex hulls (`btCompoundShape` with `btConvexHullShape` children). Concave triangle meshes (`btBvhTriangleMeshShape`) are static-only in Bullet and fail on dynamic bodies.
   - The generator performs spatial voxel decimation from high-density visual vertices (e.g. 743k triangles) down to uniform surface vertices in $<5\text{ ms}$, then executes volumetric approximate convex decomposition (V-HACD) or spatial topological clustering via `btConvexHullComputer`.
   - Automatically decomposes the car into 10 shape-accurate watertight convex hulls: 6 chassis body pieces (`FrontBumper_Hull` following the curved nose contour, `HoodFenders_Hull`, `Greenhouse_Hull` following the authentic A-pillars and roofline, `ChassisTub_Hull`, `RearDeck_Hull`, `RearBumper_Hull`) plus 4 dedicated tire/wheel hulls (`WheelFL_Hull`, `WheelFR_Hull`, `WheelRL_Hull`, `WheelRR_Hull`). 100% of the imported model geometry is captured into physical collision.
   - Separates wheel geometry to automatically derive wheel centers, wheel radius ($0.30\text{ m}$), wheel width ($0.24\text{ m}$), wheelbase ($2.76\text{ m}$), track width ($1.58\text{ m}$), and ground clearance ($10.0\text{ cm}$) directly from 3D mesh coordinates. Outlier vertices in wheel asset origins are filtered from the chassis collision mesh, eliminating pointed front/rear bumper protrusion artifacts ($Z$ strictly spans $[-2.265, +2.330]\text{ m}$, total length $4.595\text{ m}$).
   - Automatically serializes and caches generated convex hulls to `model/cache/<vehicle_id>_collision.json` (cache version 5) for instantaneous subsequent loads ($<1\text{ ms}$).
   - 3D Wireframe Overlay (`DebugOverlayGL`) renders each convex child hull with distinct vibrant colors (emerald, cyan, amber, purple, pink, blue, lime, red) for clear visual inspection.
   - Real-time diagnostic telemetry and manual re-generation buttons are integrated into `ImGuiDashboard`.


---

## 9. Complete glTF 2.0 PBR Material, Texture Pipeline & IBL Lighting Engine

1. **Full glTF 2.0 Metallic-Roughness Asset Importer (`ModelLoaderGLB.cpp`):**
   - **Preservation of Authentic PBR Properties:** Reads `base_color_factor`, `metallic_factor`, `roughness_factor`, `emissive_factor`, and alpha modes (`Opaque`, `Mask`, `Blend`) directly from source glTF meshes. Eliminated destructive hardcoded overrides that previously forced all materials to Alpine White.
   - **Complete Texture Channel Resolution:** Resolves and caches embedded binary glTF textures (`bufferView`) and external textures across all standard channels:
     - Unit 0: **Base Color / Albedo** (`GL_SRGB8_ALPHA8`, sRGB color space)
     - Unit 1: **Directional Shadow Map** (`GL_DEPTH_COMPONENT24`, depth)
     - Unit 2: **HDR Environment Panoramic Map** (`GL_RGBA8` / sRGB, IBL)
     - Unit 3: **Tangent-Space Normal Map** (`GL_RGBA8`, Linear space) with `normalScale`
     - Unit 4: **Metallic-Roughness Map** (`GL_RGBA8`, Linear space; Green = Roughness, Blue = Metalness)
     - Unit 5: **Ambient Occlusion Map** (`GL_RGBA8`, Linear space; Red = AO) with `occlusionStrength`
     - Unit 6: **Emissive Map** (`GL_SRGB8_ALPHA8`, sRGB color space)
   - **Orthonormal Tangent Generation (`Mesh::computeTangents`):** Automatically computes smooth orthogonal tangents ($\vec{T}$) and bitangent handedness signs ($tw \in \{-1.0, 1.0\}$) across indexed triangle meshes using Gram-Schmidt orthogonalization when assets lack pre-authored tangents.

2. **Cook-Torrance Microfacet GGX PBR Shader Pipeline (`phong_shadow.vert` / `phong_shadow.frag`):**
   - Replaced legacy empirical Blinn-Phong lighting with industry-standard metallic-roughness BRDF:
     - **Normal Distribution Function (NDF):** Trowbridge-Reitz GGX ($D = \frac{\alpha^2}{\pi ((N \cdot H)^2 (\alpha^2 - 1) + 1)^2}$ where $\alpha = \text{roughness}^2$).
     - **Geometric Shadowing:** Smith-Schlick GGX ($G = G_1(N, V) \cdot G_1(N, L)$).
     - **Fresnel Reflectance:** Fresnel-Schlick ($F_0 = \text{mix}(0.04, \text{albedo}, \text{metalness})$) with roughness attenuation.
     - **Energy Conservation:** $k_D = (1 - F) \cdot (1 - \text{metalness})$ balances diffuse and specular lobes.
     - **Tangent-Space Normal Mapping:** Evaluates $3 \times 3$ TBN matrix with two-sided normal inversion (`gl_FrontFacing`).

3. **High-Dynamic-Range Environment Map & IBL (`EnvironmentMapGL`):**
   - Procedural automotive HDR environment panoramic texture ($512 \times 256$) featuring atmospheric sky gradient, sun corona, and 3 overhead studio softbox light banks that highlight vehicle body curvature and shoulder creases.
   - Trilinear mipmap filtering (`glGenerateMipmap(GL_TEXTURE_2D)`) maps surface `roughness` directly to reflection blur ($LOD = \text{roughness} \times \text{maxLod}$).
   - Diffuse irradiance ambient integration combined with ACES filmic tone reproduction and gamma 2.2 linear workflow.

4. **Real-Time PBR Material Inspector & Diagnostics Panel (`ImGuiDashboard`):**
   - 9 real-time shading diagnostic modes: Full PBR Lit, Albedo, Metalness, Roughness, Shading Normals, Ambient Occlusion, Emissive, Tangents, and Environment Specular.
   - Real-time Exposure ($0.2\text{--}3.0$) and IBL Multiplier ($0.0\text{--}3.0$) sliders.
   - Interactive breakdown table displaying all distinct materials in the active model, complete with live color swatches, roughness/metalness parameters, and texture map badges (`[Alb]`, `[Nrm]`, `[Met-Rough]`, `[AO]`, `[Ems]`, `[Blend]`, `[Mask]`, `[2-Sided]`).


---

## 10. Volumetric Tyre Collision, Kinematic Wheel Cylinders & Dynamic Geometry Derivation

1. **Volumetric Circumferential & Lateral Tire Collision Pipeline (`VehicleDynamics.cpp`):**
   - Replaced legacy 1D single-point raycasts with multi-point volumetric envelope contact queries: 27 circumferential and lateral probe vectors spanning outer tyre radius $R$ and tread width $W$ across inner shoulder (-40%), center (0%), and outer shoulder (+40%) slices.
   - Forward approach arc ($\theta \in [0^\circ..40^\circ]$) detects curbs, raised platforms, stones, and steps before the axle center reaches the obstacle ($Z_{\text{contact}} > Z_{\text{axle}}$).
   - Rear departure arc ($\theta \in [-25^\circ..0^\circ]$) provides continuous tire support when descending from raised ledges and ramps without abrupt drops.
   - Calculates effective circumferential contact normal $\vec{n} = \cos(\theta)\vec{u}_{\text{up}} - \sin(\theta)\vec{u}_{\text{fwd}}$, applying horizontal obstacle resistance force ($F_{\text{obs}}$) opposing forward travel into the obstacle, and drive-wheel climbing traction ($F_{\text{climb}}$) pulling the wheel up and over steps under throttle.

2. **Dedicated Physical Wheel Collision Objects (`btCylinderShapeX`):**
   - Removed static/frozen wheel hulls from the chassis `btCompoundShape`, eliminating rigid collision interference and obstacle snagging.
   - Created 4 dedicated Bullet collision objects using `btCylinderShapeX(wWidth * 0.5, wRadius, wRadius)`. In Torque Lab local frame ($+X$ Left, $+Y$ Up, $+Z$ Forward), the cylinder central axis along $+X$ aligns with the wheel rotation axle with zero rotational offset.
   - Flagged with `btCollisionObject::CF_NO_CONTACT_RESPONSE` and broadphase mask `btBroadphaseProxy::StaticFilter` so Bullet's dynamic solver never applies unwanted contact impulses or constraints to the chassis rigid body.
   - Exact world transforms ($T_{\text{wheel}} = \text{pos} + \text{rot}$) synchronized at 120 Hz with suspension stroke and Ackermann steering rotation.
   - `castRayExactWorld` and `ExcludeVehicleRayCallback` explicitly ignore all 4 wheel collision objects to ensure raycasts find the terrain and obstacle geometry with zero self-occlusion.

3. **Generic Model-Derived Wheel Geometry & Per-Axle Fitment (`ModelLoaderGLB.cpp`, `VehicleCollisionGenerator.cpp`, `VehicleConfig.cpp`):**
   - Wheel radius ($R$), width ($W$), pivot center, and track width are derived directly from actual 3D mesh vertex bounding boxes across model parts tagged with `ModelPartRole::Wheel_*`.
   - Populates per-axle dimensions: `frontTrackWidth`, `rearTrackWidth`, `frontWheelRadius`, `rearWheelRadius`, `frontWheelWidth`, and `rearWheelWidth`, supporting staggered front/rear fitments.
   - Automatically serialized and deserialized to/from JSON in vehicle configurations.

4. **3D Collision Wireframe & Telemetry Visualization (`DebugOverlayGL.cpp`, `ImGuiDashboard.cpp`):**
   - Wireframe cylinder rendering (`addCylinderXWireframe`) dynamically renders the exact physical `btCylinderShapeX` for each wheel at its true simulated position, compression, and steering angle in distinct Cyan (front) and Amber (rear) colors.
   - Visualizes circumferential obstacle contact points, resistance force arrows, and climbing traction vectors.
   - Interactive 4-wheel telemetry table highlights obstacle contact with live hit angle and force metrics.

---

## 11. Deformable Pneumatic Tyre Simulation, Multi-Point Contact Patch & Vertex Shader Deformation

1. **Pneumatic Tyre Carcass Mechanics & Series Compliance (`PneumaticTire.cpp`, `VehicleDynamics.cpp`):**
   - Structural Kelvin-Voigt viscoelastic carcass model with dynamic pressure dependence ($p \in [80, 400]\text{ kPa}$):
     $$k_r(p) = k_{r,0} + C_{p,r}(p - p_0), \quad k_y(p) = k_{y,0} + C_{p,y}(p - p_0), \quad k_x(p) = k_{x,0} + C_{p,x}(p - p_0)$$
   - Series spring equilibrium between suspension spring ($k_{\text{susp}}$) and tyre radial carcass ($k_{\text{tire}}$):
     $$\frac{1}{k_{\text{eff}}} = \frac{1}{k_{\text{susp}}} + \frac{1}{k_{\text{tire}}}, \quad \delta_r = \Delta x_{\text{total}} \cdot \frac{k_{\text{susp}}}{k_{\text{susp}} + k_{\text{tire}}}$$
   - Verified static equilibrium on flat ground with $0.000\%$ error ($F_z = 14715.0\text{ N}$).

2. **Distributed Multi-Point Contact Patch Discretization:**
   - Discretizes the rectangular tyre contact patch ($2 a \times W$) into runtime grids matching the collision fidelity modes:
     - **Performance Mode:** 3 samples ($3 \times 1$ along centerline).
     - **Balanced Mode:** 9 samples ($3 \times 3$ across inner shoulder, center, outer shoulder).
     - **Detailed Mode:** 15 samples ($5 \times 3$ grid).
   - Hertzian-like parabolic pressure profile: $w(u) = 0.20 + 0.80(1 - u^2)$, strictly normalized so $\sum w_j = 1.0$, guaranteeing total contact load identically equals axle normal load ($\sum F_{z, j} = F_z$).
   - Longitudinal contact moment arm incorporates pneumatic trail ($t_p = 0.22 \cdot a$), generating physical self-aligning torque ($M_z = t_p \cdot F_y$) applied directly to the chassis body.

3. **Strict Rigid Rim Protection in GLSL Shaders (`phong_shadow.vert`, `shadow_depth.vert`):**
   - Evaluates vertex radial distance in wheel local cross-section: $r = \sqrt{P_{\text{wheelLocal}, y}^2 + P_{\text{wheelLocal}, z}^2}$.
   - Rim threshold guard: If $r \le R_{\text{rim}}$, radial factor $f(r) = 0.0$ identically. Only outer rubber ($r \in [R_{\text{rim}}, R_0]$) undergoes deformation:
     $$P_{\text{local}}' = P_{\text{local}} + \text{radialSquash} \cdot \vec{u}_r + \text{sidewallBulge} \cdot \vec{u}_{\text{axle}} + \text{lateralShear} \cdot \vec{u}_y + \text{longitudinalShear} \cdot \vec{u}_z$$
   - Wheel hub, spokes, calipers, and rim beads remain completely rigid and pristine at all pressures and loads.

4. **Thermodynamics, Wear Accumulation & Puncture Leakage:**
   - First-law thermal energy balance: frictional sliding dissipation ($Q_{\text{slip}} = |\vec{F}_{\text{friction}} \cdot \vec{v}_{\text{slip}}|$) + cyclic carcass hysteresis ($Q_{\text{hyst}} = C_h \cdot \omega \cdot \delta_r \cdot F_z$) vs forced convective air cooling ($h_{\text{conv}} \propto 1 + 0.04 v$) and conductive ground transfer.
   - Archard-inspired abrasive wear accumulation: $\Delta \text{wear} = K_w \frac{F_{\text{friction}} \cdot v_{\text{slip}}}{H} \Delta t$.
   - Puncture orifice air leakage: $\dot{p} = -C_{\text{leak}} \sqrt{p - p_{\text{amb}}}$, causing observable carcass deflation, sidewall squashing, and increased rolling resistance.

5. **Dear ImGui Pneumatic Tyre Dashboard & 3D Debug Overlays:**
   - Real-time front and rear tyre pressure sliders ($100\text{--}350\text{ kPa}$ / $15\text{--}50\text{ PSI}$) with factory presets (`Factory M4`, `Dodge Track`, `High Speed`, `Comfort`).
   - Puncture / Repair simulation buttons with live deflation rates.
   - 4-wheel telemetry table showing deflection $\delta_r$, sidewall bulge $\Delta w$, lateral shear $\delta_y$, temperatures, wear, contact length, and self-aligning moments.
   - 3D overlay rendering flattened contact patch rectangular footprint box, longitudinal centerline, bulging sidewall profiles, and multi-point sample contact force vectors.
   - Live ground constraint status indicators: physical penetration $0.0\text{ mm}$, visual tread penetration $0.0\text{ mm}$, rigid rim protected.

6. **Ground Penetration Elimination & World-Space Deformation (`phong_shadow.vert`, `shadow_depth.vert`, `Mesh.cpp`):**
   - **Root Cause Diagnosis (Case C):** Series compliance lowers the physical wheel hub by $\delta_r$ ($Y_{\text{hub}} = R_0 - \delta_r$) so normal forces balance vehicle mass at $Y = 0.0$ ($0.000\%$ error). However, undeformed wheel radius $R_0$ placed bottom vertices at $Y = -\delta_r$ (e.g., $-2.2\text{ cm}$). When vertex deformation was computed in local `aPos` space using an empirical angular decay curve, the displacement dropped faster than the circular arc depth, causing tread vertices in the contact zone to sag up to $7\text{ mm}$ below the asphalt and appear clipped or phasing through the ground. Furthermore, imported glTF meshes had node hierarchies and scale offsets that distorted local radius evaluations.
   - **World-Space Invariant Constraint:** Vertices are transformed to world coordinates first ($P_{\text{world}} = uModel \cdot aPos$). The exact signed perpendicular distance to the road plane is calculated:
     $$d_{\text{ground}} = (P_{\text{world}} - P_{\text{ground}}) \cdot \hat{n}_{\text{ground}}$$
     When penetrating ($d_{\text{ground}} < 0$), vertices are lifted along $\hat{n}_{\text{ground}}$ by exactly $(-d_{\text{ground}} + \text{edgeRounding}) \cdot \eta_{\text{smooth}}$, guaranteeing that deformed contact tread vertices meet the road surface flush at $d_{\text{ground}} = 0.000\text{ m}$ without knife-edges or center craters.
   - **Dedicated Procedural Pneumatic Tyre Mesh (`Mesh::createPneumaticTireMesh`):** Built with 64 radial segments, 6 concentric sidewall rings, crowned tread band, and distinct bead seats, providing smooth vertex density across the deformation zone.
   - **Dielectric Carbon Black PBR Material:** `roughness = 0.82`, `metalness = 0.00`, `baseColor = #1c1f24`, applied across procedural wheels and manifests (`BMW_M4_manifest.json`, `Dodge_Challenger_manifest.json`).
   - **Directional Shadow Depth Parity (`shadow_depth.vert`):** World-space ground flattening and sidewall bulging are mirrored identically in the shadow pass, casting accurate deformed tyre silhouettes onto the track.
   - **Automated Verification (`Unit 28`):** Rigorously verifies zero penetration ($d_{\text{ground}} \ge -0.0001\text{ m}$), flush contact patch ($Y = 0.000\text{ m}$), strict rigid rim protection ($r \le R_{\text{rim}} \implies |\Delta P| = 0$), and monotonic curvature continuity.

---

## 12. Hummer H2 Integration & Heavy-Duty Off-Road Dynamics

1. **3D Asset & Manifest Architecture (`HUMMER.glb`, `HUMMER_manifest.json`):**
   - Single-mesh glTF multi-node structure with 65 nodes (76 parts, 780,003 vertices, 1,250,404 triangles).
   - Raw model yaw authoring alignment: authored with an intrinsic $-5.1382^\circ$ yaw offset in Blender. Compensated via manifest:
     `"scale": 0.370`, `"rotation_y_deg": 5.1382`, yielding micrometer-accurate square axle alignment ($|\Delta Z_{\text{front}}| = 24\ \mu\text{m}$, $|\Delta Z_{\text{rear}}| = 57\ \mu\text{m}$).
   - Roles mapped explicitly in `model/HUMMER_manifest.json`:
     - FL: `Cylinder.009` (Tyre), `Cylinder.010` (Rim), `Cylinder.008` (Cap), `Cube.017`, `Plane.025` (Caliper).
     - FR: `Cylinder.012` (Tyre), `Cylinder.020` (Rim), `Cylinder.013` (Cap), `Cube.018`, `Cube.023`, `Plane.026` (Caliper).
     - RL: `Cylinder.018` (Tyre), `Cylinder.016` (Rim), `Cylinder.019` (Cap), `Cube.021`, `Plane.028` (Caliper).
     - RR: `Cylinder.015` (Tyre), `Cylinder.011` (Rim), `Cylinder.014` (Cap), `Cube.019`, `Cube.022`, `Plane.027` (Caliper).
     - Chassis: tailgate spare tire carrier (`Plane.018`), underbody frame rails, exhaust pipes, headlights, cabin glass.
   - PBR material overrides: authentic `DarkBlueMetallicPaint` (#102a45), `DarkGrayMetallicPaint` (#2d333b), `Tire Rubber` (#1c1f24), `Metal1` chrome grille (#f1f5f9), `Metal2` aluminum steps (#94a3b8), `AnodizedMetal` rims (#1e242c), `whiteGlass` headlights with emissive glow, `redGLASS` reactive brake lights.

2. **Off-Road Physics Parameters (`createHummerConfig()`, `model/hummer_vehicle.json`):**
   - **Mass & Inertia:** $m = 2900\text{ kg}$ curb weight, 53% front distribution, $I = (5200, 5800, 1850)\text{ kg}\cdot\text{m}^2$.
   - **Derived Geometry:** Wheelbase $L = 3.128\text{ m}$ ($123.1\text{ in}$), Track width $W = 1.731\text{ m}$ ($68.1\text{ in}$), Wheel radius $R = 0.430\text{ m}$ (34-inch all-terrain tire 315/70R17), Wheel width $W_0 = 0.320\text{ m}$, Rim radius $R_{\text{rim}} = 0.220\text{ m}$.
   - **Long-Travel Suspension:** Rest length $0.46\text{ m}$, Travel $0.42\text{ m}$ ($0.16\text{--}0.58\text{ m}$), $k_{\text{front}} = 54\,000\text{ N/m}$, $k_{\text{rear}} = 48\,000\text{ N/m}$, $c_{\text{bound}} = 4800\text{ N}\cdot\text{s/m}$, $c_{\text{rebound}} = 7200\text{ N}\cdot\text{s/m}$.
   - **Full-Time AWD Drivetrain:** GM Vortec 6200 6.2L V8 ($565\text{ N}\cdot\text{m}$ peak torque @ 4300 RPM, $393\text{ hp}$ @ 5700 RPM, low-end crawler torque $> 400\text{ N}\cdot\text{m}$ at 1000 RPM), 6L80 6-speed automatic transmission, heavy-duty 4.10:1 final drive axle ratio, and 4-wheel torque transfer.
   - **Aerodynamics & Rolling Resistance:** Frontal area $A = 3.65\text{ m}^2$, $C_d = 0.54$, $C_{rr} = 0.018$.

3. **Collision Hulls & Automated Verification (`Unit 29`):**
   - Auto-baker generates 6 watertight convex collision pieces (`FrontBumper_Hull`, `Cabin_Hull`, `RearBed_Hull`, etc.) cached in `model/cache/Hummer_H2_collision.json` (93 ms).
   - Validated in `native_validation_tests.cpp` (Unit 29, 42 suites total): static equilibrium $\sum F_z = 28\,449.0\text{ N}$ (0.0% error), AWD 4-wheel drive torque, and seamless 4-way vehicle switching (BMW M4 $\leftrightarrow$ Dodge Challenger $\leftrightarrow$ Hummer H2 $\leftrightarrow$ Procedural Coupe).

---

## 13. PROJECT ARES — HACKATHON VISION AND DEVELOPMENT ROADMAP

### 13.1 Project Identity & Executive Vision
- **Official Name:** **PROJECT ARES — Autonomous Rover Exploration Simulator**
- **Foundation:** Built directly upon the high-performance C++20/OpenGL 3.3/Bullet Physics/Dear ImGui architecture of **Torque Lab Native**.
- **Mission:** Transform a 6-DOF terrestrial vehicle simulation into a scientifically grounded, physics-driven planetary rover exploration and autonomous navigation platform.
- **Core Pillars:**
  1. **Multi-Resolution Planetary Terrain:** Regional orbital geography (1 m/px HiRISE DTMs), local high-detail surface reconstruction (photogrammetric stereo meshes), and wheel-contact terramechanics.
  2. **Realistic Rover Mechanics:** High-articulation rocker-bogie suspension, 6 independent drive wheels with hub motors, active corner steering, and low-speed obstacle climbing.
  3. **Planetary Environmental Simulation:** Centralized planetary parameters (Martian gravity $g = 3.71\text{ m/s}^2$, thin atmospheric density $\rho \approx 0.020\text{ kg/m}^3$, surface temperature, and Rayleigh/dust aerosol sky scattering).
  4. **Multi-Map Support & Runtime Toggling:** Dynamic switching between multiple surface environments (e.g., Earth Proving Ground, Mars Site Alpha, Mars Site Beta, Mars Regional HiRISE DTM) with synchronized physics, rendering, and spawn coordinates.
  5. **Physics-Aware Autonomous Navigation:** Real-time traversability evaluation, cost map generation, obstacle detection, rollover risk estimation, and dynamic replanning (A* / D* Lite).
  6. **Scientific & Engineering Telemetry:** 3D force vectors, wheel sinkage/slip telemetry, terrain grade/slope visualization, and autonomous path tracking.

---

### 13.2 Original TorqueLab Foundation (Preservation & Reusability)
The existing TorqueLab codebase provides an industrial-grade foundation that will be preserved without breaking existing features:
- **Directly Reusable Systems:**
  - Modern OpenGL 3.3 Core rendering pipeline (`Shader`, `Mesh`, `Texture2D`, `ShadowMapFBO`, `EnvironmentMapGL`).
  - Single-header glTF 2.0 loader (`ModelLoaderGLB`) supporting multi-node hierarchies, embedded textures, and PBR materials.
  - Bullet 3D Physics fixed 120 Hz accumulator loop and constraint solver (`PhysicsWorld`).
  - Procedural atmospheric sky dome renderer (`SkyRendererGL`, `sky.frag`, `sky.vert`).
  - 3D force gizmos, wireframes, and contact overlays (`DebugOverlayGL`).
  - 4-mode camera controller (`CameraControllerGL`: Orbit, Follow, Suspension, Hood).
  - Progressive input pipeline with baseline trigger calibration and gamepad adaptivity (`InputControllerGLFW`, `ControllerConfig`, `VibrationManager`).
  - Automated 43-suite validation test runner (`native_validation_tests.cpp`).
  - Model-derived collision baking pipeline (`VehicleCollisionGenerator`).
- **Preservation Rule:** Existing road vehicles (`BMW M4`, `Dodge Challenger`, `Hummer H2`, `Procedural Coupe`), the Earth Proving Ground track, and the 43 verification test suites will remain operational. Planetary rovers will be added as first-class vehicles alongside existing vehicles.

---

### 13.3 Architectural Separation: Road Vehicles vs. Planetary Rovers
- **Shared Low-Level Physics Utilities:**
  - Both vehicle classes share: `btDiscreteDynamicsWorld`, `btRigidBody`, Bullet raycasting / sweep queries, `SurfaceSystem` registry, `Math3D` primitives, and `DebugOverlayGL`.
- **Decoupled Vehicle Simulation Pipeline:**
  - `VehicleDynamics`: Retained strictly for conventional 4-wheel terrestrial vehicles. Contains road-specific assumptions: 4-strut spring/damper suspension, internal combustion engine torque curve, multi-gear transmission with clutch, and Pacejka 94/96 Magic Formula rubber tyre mechanics.
  - `RoverDynamics` (Future Phase 2): A dedicated 6-wheel rover simulation engine accommodating:
    - Rocker-bogie articulated mechanism (main rocker + rear bogie articulated through a top chassis differential bar or pivot).
    - Passive geometric load equalization without springs or dampers.
    - 6 independently motorized wheels with high gear-reduction electric hub motors.
    - Active corner steering actuators (Ackermann, point turns, crab steering).
    - Terramechanics soil contact interface.

---

### 13.4 Three-Level Multi-Resolution Terrain Architecture
The terrain architecture is structured into three distinct physical and visual levels to avoid treating orbital data as sufficient for rover-scale obstacles:

1. **Level 1 — Regional Terrain (Macro Scale, 1–10 m/pixel):**
   - **Data Source:** Measured orbital elevation datasets (e.g. NASA HiRISE DTM `mars_terrain2.tif`, MOLA altimetry).
   - **Role:** Regional geography, macroscopic slope gradients, crater rim morphology, distant visual horizon, and global traversability routing.
   - **Physics & Graphics:** Managed via Bullet's `btHeightfieldTerrainShape` for $O(1)$ raycasting and chunked LOD grid meshes in OpenGL.
   - **Limitation:** At 1.0 m/pixel, sub-meter rocks (10–50 cm) that could high-center or damage a rover are unrepresented.
2. **Level 2 — Local Terrain (Meso Scale, 0.02–0.20 m/pixel, centimeter resolution):**
   - **Data Source:** High-resolution 3D surface reconstructions from rover stereo Navcams/Mastcams and photogrammetric site meshes (e.g. `mars_terrain0.glb`, `mars_terrain1.glb`).
   - **Role:** Discrete rocks, boulders (10 cm to 2 m), bedrock fractures, local sand ripples, immediate rover navigation corridor ($\sim 50\text{ m} \times 50\text{ m}$).
   - **Physics & Graphics:** Static triangle meshes in Bullet (`btBvhTriangleMeshShape`) and textured PBR meshes in OpenGL.
3. **Level 3 — Wheel-Contact Terrain (Micro Scale, millimeter resolution):**
   - **Role:** Local soil deformation, wheel grouser interaction, sinkage, and track imprint formation.
   - **Terramechanics Model:** Bekker-Wong pressure-sinkage and Janosi-Hanamoto shear stress displacement relationships.
4. **Strict Separation of Scientific Data vs. Procedural Augmentation:**
   - Measured elevation $z_{\text{measured}}(x, y)$ from HiRISE or photogrammetry is strictly preserved in memory and tagged as `ElevationSource::ScientificMeasurement`.
   - Procedural micro-roughness or soil displacement $\Delta z_{\text{proc}}$ may supplement rendering or contact compliance, but is tracked as an explicit secondary layer:
     $$z_{\text{render}}(x, y) = z_{\text{measured}}(x, y) + \Delta z_{\text{procedural}}(x, y)$$
   - Cost maps and scientific queries have direct access to the unadulterated elevation data.

---

### 13.5 Verified Inspection of Existing Mars Assets

| Asset Name | File Size | Geometry / Resolution | Spatial Coverage | Coordinate System & Orientation | Textures | Elevation Geometry | Suitability |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`mars_terrain0.glb`** | 24.19 MB (25,362,804 B) | 7 meshes, 123,606 vertices, 232,571 triangles | $\approx 33.9\text{ m} \times 29.2\text{ m}$ (footprint), $2.97\text{ m}$ relief | Root matrix has $-90^\circ$ pitch ($Z \leftrightarrow Y$ flip) | 3 JPEG textures ($4096 \times 4096$) | Actual 3D photogrammetric triangle mesh; includes 10m scale bar mesh | **Visual & Collision** (Level 2 Local Terrain) |
| **`mars_terrain1.glb`** | 56.92 MB (59,683,904 B) | 3 meshes, 137,285 vertices, 200,463 triangles | Raw bounds $36.6\text{ m} \times 65.3\text{ m}$; with root scale $0.357$ $\implies 13.1\text{ m} \times 23.3\text{ m}$ ($2.04\text{ m}$ relief) | Root matrix has $-90^\circ$ pitch and $0.357$ uniform scale | 1 PNG texture ($8192 \times 8192$, 67.1 MP) | Actual 3D photogrammetric surface mesh with boulders and regolith | **Visual & Collision** (Level 2 Local Terrain) |
| **`mars_terrain2.tif`** | 1.84 GB (1,839,545,409 B) | 21,400 cols $\times$ 21,488 rows ($459,843,200$ pixels), 32-bit float IEEE | $21.4\text{ km} \times 21.488\text{ km}$ ($459.8\text{ km}^2$), $1.0\text{ m/px}$ resolution | Equirectangular Mars 2000 Sphere IAU ($R=3,396,190\text{ m}$), tie pt $(4577366, 1106618)$ | Elevation-only (no RGB orthophoto) | Actual measured HiRISE DTM; center sector: $-2463\text{ m}$ to $-2422\text{ m}$ ($41.0\text{ m}$ relief) | **Visual & Collision** (Level 1 Regional via tile preprocessor) |

---

### 13.6 Geospatial Preprocessing Pipeline & Coordinate Mapping
- **Tooling:** Offline Python preprocessing tool (`scripts/preprocess_mars_dtm.py`) utilizing NumPy.
- **Preserved Metadata:**
  - Coordinate Reference System: Equirectangular Mars 2000 Sphere IAU (Semi-major $a = 3,396,190.0\text{ m}$, Semi-minor $b = 3,396,190.0\text{ m}$).
  - Map projection: Equirectangular (`CT_Equirectangular`), linear units in meters.
  - Horizontal scale: exactly $1.0\text{ m/pixel}$.
  - Elevation datum: MOLA reference aeroid (elevation values stored in unscaled meters).
  - NoData handling: Tag 42113 value ($-3.40282 \times 10^{38}$) explicitly filtered during tile extraction.
- **Coordinate Transformation to Project ARES Frame ($+X$ Left, $+Y$ Up, $+Z$ Forward):**
  - Tile center projected coordinates: $(X_c, Y_c)$.
  - For pixel $(col, row)$ within tile:
    - Easting offset: $\Delta E = (col - col_c) \times 1.0\text{ m}$.
    - Northing offset: $\Delta N = (row_c - row) \times 1.0\text{ m}$ (row 0 is North).
    - Elevation offset: $\Delta Y = h(col, row) - h_{\text{ref}}$.
    - TorqueLab mapping: $X = -\Delta E$ (Left / West), $Z = \Delta N$ (Forward / North), $Y = \Delta Y$ (Up).
- **Export Artifacts:**
  - Binary height array (`model/terrain/<name>.bin`) for $O(1)$ C++ runtime loading.
  - Companion metadata JSON (`model/terrain/<name>.json`) preserving full georeferencing, bounds, and spawn points.
  - Surface normal map (`model/terrain/<name>_normal.png`) computed from central differences for PBR shading.

---

### 13.7 Surface Physics & Provisional Regolith Parameters
- **Provisional Nature:** Friction and resistance parameters are working engineering approximations derived from published Mars exploration mobility literature, not absolute constants:
  - **Fine Martian Dust:** $\mu \approx 0.35 - 0.40$ (Provisional approximation based on Viking/MER wheel trenching; Moore et al., *NASA SP-4212*; Sullivan et al., *J. Geophys. Res.* 2008).
  - **Compacted Regolith:** $\mu \approx 0.60 - 0.70$ (Provisional approximation from Spirit/Opportunity hard soil flats; Arvidson et al., *J. Field Robotics* 2010).
  - **Basalt Bedrock / Outcrop:** $\mu \approx 0.80 - 0.90$ (Provisional approximation from basaltic rock friction tests).
  - **Loose Sand Dunes / Ripples:** $\mu \approx 0.25 - 0.35$ (Provisional approximation from Opportunity Purgatory Dune entrapment analysis).
- **Future Terramechanics Formulation (Phase 2):**
  - Bekker-Wong pressure-sinkage: $p = \left(\frac{k_c}{b} + k_\phi\right) z^n$.
  - Janosi-Hanamoto shear stress displacement: $\tau = (c + p \tan \phi)(1 - e^{-j/K})$.
  - Compaction resistance: $R_c = \frac{b}{n+1} \left(\frac{k_c}{b} + k_\phi\right) z^{n+1}$.
  - Bulldozing resistance: $R_b$.

---

### 13.8 Immediate Development Priority (Phase 1 Focused)
Our immediate milestone is establishing the functional Mars surface environment:

1. **Asset & Geospatial Inspection:** [COMPLETED] Verified metadata, dimensions, and suitability for `mars_terrain0.glb`, `mars_terrain1.glb`, and `mars_terrain2.tif`.
2. **Geospatial Preprocessing Pipeline:** Implement `scripts/preprocess_mars_dtm.py` to extract high-resolution playable sectors from `mars_terrain2.tif` with metadata JSON.
3. **Multi-Map Management Architecture (`TerrainManager`):** Create `TerrainManager` and `ITerrainMap` interface supporting Earth Proving Ground, Mars Site Alpha, Mars Site Beta, and Mars Regional DTM.
4. **Photogrammetric 3D Terrain Collision & Mesh Loading:** Integrate `btBvhTriangleMeshShape` for GLB terrain assets.
5. **Raster Heightfield Elevation Loader:** Integrate `btHeightfieldTerrainShape` and OpenGL grid mesh generation for preprocessed DTM sectors.
6. **Martian Environmental Configuration:** Implement `PlanetaryEnvironment` ($g = 3.71\text{ m/s}^2$, $\rho = 0.020\text{ kg/m}^3$, surface regolith types, sky scattering).
7. **Mars Surface & Sky Rendering:** Configure procedural atmospheric scattering (`sky.frag`) and PBR regolith shading.
8. **Validation & Regression Testing:** Confirm all 43 existing test suites pass under Earth presets, and add automated tests for Martian gravity and terrain contact stability.

---

## 14. Phase 1 Implementation & Bug Fix Report (ARES Architectural Audit)

### 14.1 Root Cause & Permanent Resolution: Terrain Moving With Vehicle
- **The Bug:** In earlier builds, the entire Mars terrain appeared to move and rotate synchronously with vehicle movements instead of anchoring stationary in world space.
- **Root Cause Discovered in Code Audit:**
  1. `VehicleRendererGL::init()` recursively scanned the root `model/` directory for any file with a `.glb` extension and unconditionally loaded every found model into its `m_importedModels` registry as a drivable vehicle chassis.
  2. Because `mars_terrain0.glb` and `mars_terrain1.glb` were stored directly in `model/`, they were parsed by `ModelLoaderGLB` as candidate vehicle bodies.
  3. When an imported chassis was rendered in `VehicleRendererGL::renderMainPass()`, it was transformed by `m_importedChassisWorldMatrix`—the dynamic 6-DOF transform of the vehicle chassis computed each frame from Bullet rigid-body state!
- **Permanent Solution:**
  1. **Strict Asset Segregation:** Physically relocated `mars_terrain0.glb`, `mars_terrain1.glb`, and `mars_terrain2.tif` into a dedicated subdirectory `model/terrain/`.
  2. **Model Import Guard:** Added an explicit semantic name filter in `VehicleRendererGL.cpp` (`if (lowerStem.find("terrain") != std::string::npos) continue;`) to guarantee no terrain asset can ever be loaded as a vehicle chassis.
  3. **World Coordinate Anchoring:** In `TrackRendererGL`, terrain chunks and meshes are rendered strictly with fixed world transform matrices (`Mat4::identity()` or `Mat4::fromTranslation(chunkCenter)`), completely decoupled from any vehicle transform.

### 14.2 Multi-Resolution Chunked Terrain Architecture
- **Chunk Representation (`TerrainChunk`):**
  - Defined in `src/terrain/TerrainChunk.h`. Each chunk owns an immutable world transform matrix, center position, axis-aligned bounding box (`TerrainAABB`), procedural OpenGL `Mesh`, PBR material properties, and visibility state.
- **Regional DTM 8x8 Grid Partitioning:**
  - In `MarsRegionalDTMMap::buildGraphics()`, the $1024 \times 1024$ raster heightfield is divided into an $8 \times 8$ grid of 64 independent chunks ($128\text{ m} \times 128\text{ m}$ each, 2m sampling step, 4,225 vertices per chunk).
  - Vertex coordinates are local to the chunk center to maximize floating-point precision on GPU.
  - Chunk boundaries share border samples (`cEnd = (cx + 1) * samplesPerChunk`), and vertex normals are sampled from the global terrain grid, guaranteeing zero visible seams or cracks across chunk edges.
- **Matching Bullet Heightfield Collision:**
  - Physics collision is implemented via `btHeightfieldTerrainShape` with `PHY_FLOAT` elevation data and `upAxis = 1` ($Y$).
  - Correct Bullet vertical centering: `btHeightfieldTerrainShape` centers its bounding box at $y = \frac{\min H + \max H}{2}$. The Bullet rigid body is positioned at $(0, \frac{\min H + \max H}{2}, 0)$ with zero rotation, aligning raycasts within $0.05\text{ m}$ of visual heights.

### 14.3 Martian Environmental Physics & Soil Properties
- **Planetary Parameters (`PlanetaryEnvironment`):**
  - Surface Gravity: $g = 3.71\text{ m/s}^2$ ($0.378 g_\oplus$).
  - Atmospheric Density: $\rho = 0.020\text{ kg/m}^3$ (Earth standard: $1.225\text{ kg/m}^3$).
  - Mean Surface Pressure: $P = 636\text{ Pa}$ ($6.36\text{ mbar}$).
  - Mean Surface Temperature: $T = 210\text{ K}$ ($-63^\circ\text{C}$).
  - Solar Irradiance: $S_0 = 590\text{ W/m}^2$ (Earth standard: $1361\text{ W/m}^2$).
- **Surface Material Hierarchy (`SurfaceSystem`):**
  - `MarsDust`: $\mu = 0.38$, $C_{rr} = 0.045$, $C_\alpha = 0.70$.
  - `MarsRegolith`: $\mu = 0.65$, $C_{rr} = 0.022$, $C_\alpha = 0.90$.
  - `MarsBedrock`: $\mu = 0.85$, $C_{rr} = 0.012$, $C_\alpha = 1.05$.
  - `MarsDuneSand`: $\mu = 0.30$, $C_{rr} = 0.055$, $C_\alpha = 0.60$.

### 14.4 Automated Regression & Validation Suite Expansion (45 Suites Total)
- **Unit 31: Planetary Environment & Mars Surface Physics:**
  - Verifies Earth vs Mars gravity and atmospheric density ratios.
  - Validates Martian datum surface pressure ($636\text{ Pa}$) and solar irradiance ($590\text{ W/m}^2$).
  - Verifies friction hierarchy across Martian dust, regolith, bedrock, and sand dunes.
  - Simulates dynamic vehicle static equilibrium under $3.71\text{ m/s}^2$ gravity, confirming equilibrium normal load matches $m \cdot g_{\text{mars}}$ within $0.1\%$.
- **Unit 32: Terrain World Transform Stability & Physics Alignment (ARES Regression):**
  - Instantiates `TerrainChunk` grid and validates that chunk `worldMatrix` remains strictly invariant before and after vehicle movement, acceleration, and rotation.
  - Validates that physics raycasts onto `btHeightfieldTerrainShape` match analytical raster elevations within $0.05\text{ m}$.
  - Confirms that terrain meshes exist strictly in `model/terrain/` and are absent from root `model/`.
- **Integrated Test Execution:**
  - Added `--tests` CLI argument to `ProjectARES.exe` (and `TorqueLabNative.exe`) to allow running the full 47-suite test harness directly within the primary executable.

---

## 15. Project Cleanup, Crash Investigation & Stability Restoration (ARES Milestone)

### 15.1 Complete Elimination of Legacy Road Cars
- **Assets Removed:**
  - `model/BMW_M4.glb`, `model/BMW_M4_manifest.json`, `model/bmw_m4_vehicle.json`
  - `model/Dodge_Challenger.glb`, `model/Dodge_Challenger_manifest.json`
  - `model/HUMMER.glb`, `model/HUMMER_manifest.json`, `model/hummer_vehicle.json`
  - `model/procedural_coupe_vehicle.json`
  - Decals: `model/textures/Stripes_Wings.png`, `Front_Bumper_Text.png`, `Front_Hodd_Text.png`, `600px-BMW.svg.png`
  - Collision caches: `model/cache/1970_Dodge_Challenger_R_T_collision.json`, `Hummer_H2_collision.json`, `BMW_M4_Coupe_collision.json`
- **Active Mars Rover Fleet:**
  - **NASA Perseverance Rover (`model/PerseveranceRover.glb`):** Mass 1025 kg, 2.65m track width, 2.25m wheelbase, -0.15m CoM offset, AWD direct-drive electric motors with flat 480 N·m torque, 120.0:1 planetary final drive, 0.2625m rigid cleated aluminium wheels.
  - **NASA Curiosity Rover (`model/24584_Curiosity_static.glb`):** Mass 899 kg, MSL scientific exploration platform, 120.0:1 reduction.
  - **NASA Opportunity Rover (`model/24883_MER_static.glb`):** Mass 185 kg, MER platform, 80.0:1 reduction.
  - **ARES Prototype Rover (`procedural_rover`):** Procedural geometric chassis for rapid telemetry testing.

### 15.2 Root-Cause Bug Fixes & Stability Restoration
1. **Crash Cause 1 — Map Selector Dangling Pointers (`ImGuiDashboard.cpp`):**
   - *Bug:* In both Top Bar and Left Panel map combo selectors, `maps[i]->getDisplayName().c_str()` was passed to `items.push_back()`. Because `getDisplayName()` returns `std::string` by value, `.c_str()` pointed to destroyed temporary memory, causing crashes or corrupted labels.
   - *Fix:* Retained strings inside `std::vector<std::string> mapLabels` container across the combo lifetime.
2. **Crash Cause 2 — Off-By-One Buffer Overflow in DTM Chunks (`TerrainManager.cpp`):**
   - *Bug:* In `MarsRegionalDTMMap::buildGraphics()`, the chunk loops (`r` and `c`) ran with `r <= rEnd` up to 1024 on a 1024x1024 heightfield (valid indices `0..1023`), accessing `r * cols + c = 1,048,576` which caused a SIGSEGV heap read violation.
   - *Fix:* Clamped sampling indices: `rClamped = std::min(r, rows - 1)`, `cClamped = std::min(c, cols - 1)`.
3. **Crash Cause 3 — Underground Spawn Explosion (`PhysicsWorld.cpp`):**
   - *Bug:* `PhysicsWorld::teleportToPreset()` hardcoded vehicle spawn elevation to `spawnPos.y = eqHeight + 0.04`, ignoring preset elevations and placing the rover 6–20m underground inside elevated Martian heightfields, causing collision impulse explosions.
   - *Fix:* Dynamically raycasts downwards from the preset position to locate true terrain surface elevation (`groundY`) and places chassis safely at `groundY + eqHeight + 0.06m`.
4. **Leak Cause 4 — TrackRenderer Chunk Leakage (`TrackRendererGL.cpp`):**
   - *Bug:* `rebuildProvingGround()` cleared `m_items` but not `m_chunks`. Switching back to Earth Proving Ground from Mars Jezero Crater left 64 Martian terrain chunks rendered simultaneously.
   - *Fix:* Updated `rebuildProvingGround()` to invoke `clearItems()`, purging both items and chunks.
5. **Leak Cause 5 — GLB Texture Destruction / Use-After-Free (`TerrainManager.cpp`):**
   - *Bug:* `MarsSiteAlphaMap::buildGraphics()` loaded GLB terrain meshes into a temporary stack variable `LoadedVehicleModel model`. When the function returned, `model.loadedTextures` was destroyed, calling `glDeleteTextures` and leaving dangling texture handles.
   - *Fix:* Promoted `m_loadedModel` to a class member of `MarsSiteAlphaMap` and `MarsSiteBetaMap`, preserving textures throughout the map's active lifetime.
6. **Path Resolution — Deterministic Asset Resolver (`ResourcePath.h`):**
   - Created `ResourcePath::resolve()` to discover assets relative to CWD, parent folders (`../`, `../../`), and the Windows executable module path (`GetModuleFileNameW`), guaranteeing asset discovery regardless of launch location.

### 15.3 Build Targets & Smart App Control (SAC) Guidance
- **CMake Targets:**
  - `ProjectARES` (main application executable)
  - `ProjectARESTests` (automated 47-suite test runner)
  - Backwards-compatible aliases: `TorqueLabNative` -> `ProjectARES`, `TorqueLabNativeTests` -> `ProjectARESTests`
- **Windows 11 Smart App Control (SAC) Execution:**
  - Machine state: `VerifiedAndReputablePolicyState = 1` (strict enforcement). Unsigned newly compiled binaries run outside an IDE may be flagged by SAC.
  - Recommended developer workflows:
    - **Option 1 (Recommended):** Run from Visual Studio IDE via **F5 (Debug)** or **Ctrl+F5 (Start Without Debugging)**. VS debugger attachment bypasses SAC.
    - **Option 2:** Run automated test runner via CLI: `.\build\ProjectARESTests.exe` or `.\build\ProjectARES.exe --tests` (terminal runs cleanly).
    - **Option 3:** Enable Windows Developer Mode / turn off SAC in Windows Security for C++ development environments.

### 15.4 Verified Test Benchmarks
- **47 / 47 Test Suites Passing (0 Failed Assertions):**
  - **Units 1–30:** Mathematical suspension, tire friction, Pacejka magic formula, Ackermann steering, aerodynamics, V-HACD collision decomposition, controller mapping, pneumatic tire compliance, and 3D rover asset validation.
  - **Unit 31:** Planetary Environment & Mars surface physics ($g = 3.71\text{ m/s}^2$, $\rho = 0.020\text{ kg/m}^3$).
  - **Unit 32:** Terrain World Transform Stability & Physics Alignment (regression test guaranteeing terrain chunks remain strictly static in world space).
  - **Unit 33:** Mars Rover Drivetrain, Low CoM & Planetary Suspension (Perseverance mass 1025 kg, AWD flat 480 N·m torque, 120:1 reduction, 3802.75 N Mars weight equilibrium).
  - **Unit 34:** Map Switching Lifecycle & Multi-Terrain Integrity (cyclical transitions between Proving Ground, Site Alpha, Site Beta, and Jezero DTM with 0 leaks).
  - **Validation Tests A–J & Controlled Tests 1–3:** Static equilibrium, acceleration squat, braking dive, cornering roll, surface comparison, speed bumps, collision recovery, and timestep independence.
- **Live OpenGL Timing (NVIDIA GeForce RTX 5050 Laptop GPU):**
  - 600 live frames rendered on Mars Jezero Crater HiRISE DTM at **753.9 FPS** (`phys=0.00ms, rnd=0.7ms, ui=0.3ms, swp=0.4ms`).

---

## 17. Perseverance Rover Mobility & Mechanical Articulation Refinement

### 17.1 glTF Node Hierarchy & 11-Part Suspension Mesh Partitioning
- **Node Hierarchy Breakdown (`model/PerseveranceRover.glb`):**
  - **Node 40 (`Wheels_objs`):** 68,016 triangles comprising all 6 rover wheels ($R = 0.26267\text{ m}$, wheel width $0.3356\text{ m}$).
  - **Node 41 (`suspension`):** 23,324 triangles comprising the entire rocker-bogie mechanical linkage and steering knuckles.
  - **Node 42 (`Chassis_obj`):** 44,798 triangles comprising the rover central avionics warm electronics box (WEB), top deck, and science payloads.
- **Topological & Spatial Mesh Partitioning (`ModelLoaderGLB.cpp`):**
  - Previously, Node 41 was unclassified and grouped entirely into static chassis parts, freezing the suspension geometry.
  - Evaluated in local node coordinate space (prior to root node transformation $[0, 0.710975, -0.034999]$), partitioned all 23,324 triangles into 11 semantic kinematic roles with exact bilateral symmetry:
    1. `Rover_DiffBar` (3,414 triangles): Differential pivot bar on top chassis deck ($|x| < 0.65\text{ m}, y > 0.38\text{ m}$).
    2. `Rover_DiffLink_L` / `Rover_DiffLink_R` (252 triangles each): Drop tie-rod links connecting differential bar to main rockers.
    3. `Rover_Rocker_L` / `Rover_Rocker_R` (4,972 triangles each): Main longitudinal rocker arms spanning from front wheel hub mount to rear bogie pivot trunnion ($y \in [-0.25, 0.38]\text{ m}, z \in [-0.55, 1.25]\text{ m}$).
    4. `Rover_Bogie_L` / `Rover_Bogie_R` (2,050 triangles each): Secondary rear bogie arms spanning from bogie pivot trunnion to mid and rear wheel hub mounts ($z < -0.45\text{ m}, y < 0.05\text{ m}$).
    5. `Rover_Knuckle_FL` / `Rover_Knuckle_FR` / `Rover_Knuckle_RL` / `Rover_Knuckle_RR` (1,843 triangles each): Steerable kingpin spindle knuckles on 4 corner wheel stations.

### 17.2 Sub-Millimeter NASA CAD Kinematic Hardpoints & Wheel Alignment
- **Root-Cause Wheel Detachment Fix:** In `ModelLoaderGLB.cpp`, the wheel pivot derivation loop was bounded at `i < 4`. Wheels `ML` (index 4) and `MR` (index 5) were never subtracted by `truePivot`, causing them to inherit their local bind coordinates twice and float outward by over 1.3 metres. Expanded the loop to `i < 6`, restoring exact wheel centering.
- **False-Positive Elimination:** Filtered out the `rimfax` instrument substring from wheel rim matching, preventing non-wheel geometry from corrupting wheel radius or bounding extents.
- **Precision Kinematic Pivot trunnions (`RockerBogieSuspension.h/cpp`):**
  - Rocker trunnion pivots: $P_L = (+0.806, +0.190, +0.249)\text{ m}$, $P_R = (-0.806, +0.190, +0.249)\text{ m}$.
  - Bogie trunnion pivots: $B_L = (+0.906, -0.047, -0.505)\text{ m}$, $B_R = (-0.906, -0.047, -0.505)\text{ m}$.
  - Differential bar trunnion: $D_0 = (0.000, +0.457, -0.114)\text{ m}$.
  - Differential link joints: $T_{L,\text{bar}} = (+0.540, +0.457, -0.114)\text{ m}$, $T_{L,\text{rocker}} = (+0.690, +0.220, -0.114)\text{ m}$.
  - Rest pose wheel hubs match bind coordinates with zero offset.

### 17.3 Independent 6-Wheel Spin & Decoupled Knuckle Steering
- **Independent Sub-Stepped Rotation:** All 6 wheels spin independently around their spindle axis based on physical angular velocity $\omega_i = v_i / r_i$, accounting for slip, drive torque, and terrain contact. Forward and reverse drive directions rotate wheels in physically authentic directions.
- **Decoupled 4-Corner Steering:** Steering angle $\delta_i$ (Ackermann corner steering) rotates the knuckle assembly around the vertical kingpin axis, while wheel rotation $\theta_{\text{rot},i}$ spins the wheel around its spindle axis:
  $$M_{\text{wheel},i} = M_{\text{mount},i} \cdot R_y(\delta_i) \cdot R_x(\theta_{\text{rot},i})$$
- **Rigid Middle Axle:** Middle wheels `ML` and `MR` remain strictly non-steered ($\delta = 0.0\text{ rad}$) while spinning independently.

### 17.4 Full Articulated Rendering & Diagnostic Gizmo
- **Forward Kinematic Hierarchical Rendering (`VehicleRendererGL.cpp`):**
  - Main pass and shadow caster pass dynamically evaluate transformed local matrices for all 11 suspension components.
  - Rocker arms pitch around $P_{L/R}$; bogie arms pitch relative to rocker around $B_{L/R}$; differential bar pitches around $D_0$; knuckles steer around corner kingpins.
- **Real-Time 3D Articulation Skeleton Gizmo (`DebugOverlayGL.cpp`):**
  - Diagnostic visual overlay rendering rocker links, bogie triangles, differential bar, tie rods, and kingpin steering axes.
- **50 / 50 Test Suites Passing:**
  - Automated test suite `Unit 37` verifies mechanical articulation, 6-wheel rotation, and kinematic joint closure under Martian terrain contact. All 50 test suites pass with 0 failed assertions.


### 16.1 Perseverance Invisibility & Draco glTF Decompression
- **Draco Compression Decoding:** All 252 primitives across 24 meshes in `model/PerseveranceRover.glb` were encoded using the `KHR_draco_mesh_compression` extension. Without a Draco decoder, `cgltf` skipped buffer attribute extraction, yielding 0 vertices and making the rover invisible. Integrated `draco::Decoder` in `ModelLoaderGLB.cpp`, successfully decompressing all 138,783 vertices and 199,521 triangles in memory.
- **Tri-Band 6-Wheel Assembly Partitioning:** In `PerseveranceRover.glb`, all wheels were authored in a single mesh node (`Wheels_objs`). The legacy 4-wheel quadrant partitioning ($Z \ge 0$ vs $Z < 0$) merged middle wheels into the rear wheel assembly. Implemented a tri-band $Z$-clustering algorithm ($Z > +0.3\text{ m} \to \text{Front}$, $|Z| \le 0.3\text{ m} \to \text{Mid}$, $Z < -0.3\text{ m} \to \text{Rear}$) and partitioned them by lateral sign ($X > 0 \to \text{Left}$, $X < 0 \to \text{Right}$) into 6 distinct wheel assemblies: `FL=0, FR=1, RL=2, RR=3, ML=4, MR=5`.
- **Debug Wireframe Elimination:** Default debug visual overlays (cyan wireframes, cylinder bounds, suspension line gizmos) were muted by default (`showWireframe = false`, `showColliders = false`), delivering a clean cinematic view.

### 16.2 Authentic 6-Wheel Rocker-Bogie Suspension Mechanics (`RockerBogieSuspension`)
- **Passive Articulation Kinematics:** Replaced passenger car coilover springs and anti-roll bars with a true rocker-bogie kinematic linkage. Features left and right primary rockers connected to chassis trunnions at $y = +0.18\text{ m}$, pivoting secondary bogies connecting mid and rear wheels, and authentic pitch range constraints ($\pm 31.5^\circ$ rocker, $\pm 34.4^\circ$ bogie).
- **Central Differential Bar Coupling:** Constrains rocker pitch motions through a mechanical differential bar ($\theta_{\text{diff}} = \frac{1}{2}(\theta_L - \theta_R)$), ensuring that when one rocker pitches upward over an obstacle, the opposite rocker pitches downward to maintain continuous chassis leveling and ground contact.
- **4-Corner Ackermann Steering & Fixed Mid Wheels:** Front wheels (`FL`, `FR`) steer in the direction of travel, while rear wheels (`RL`, `RR`) counter-steer in opposite phase for minimum turn radius and zero-radius skid-steering. Middle wheels (`ML`, `MR`) remain rigidly aligned straight ($0.0\text{ rad}$ steering angle).
- **Statically Determinate Normal Load Equalization:** Implemented a 2-pass contact and load solver. When all 6 wheels are in ground contact, each wheel carries exactly $\frac{1}{6} M g$ ($\sim 633.8\text{ N}$ under Mars gravity for Perseverance). Dynamic vertical velocity damping cushions drop landings and completely eliminates floating or bouncing.
- **Chassis Collision Hull Refinement:** Refined rover chassis collision half-extents ($X=1.15\text{ m}, Y=0.28\text{ m}, Z=1.35\text{ m}$) so that the belly pan sits cleanly above ground clearance ($> 0.20\text{ m}$ clearance), preventing the chassis compound shape from resting on the ground while the wheels are in the air.

### 16.3 Multi-Resolution Terrain System & Jezero Crater Skirt Elimination
- **Bisecting Skirt Bug Fix:** Previously, a flat $3600\text{ m} \times 3600\text{ m}$ horizon ground skirt at $y = -0.05\text{ m}$ bisected the NASA HiRISE Jezero Crater DTM ($y \in [-21.43\text{ m}, +20.97\text{ m}]$), concealing valleys and crater depressions under flat brown slabs. Replaced the flat center plane with a perimeter collar cutout with an inner void ($512\text{ m} \times 512\text{ m}$) and 4 peripheral skirt quads dropped below the crater floor ($y = \text{minHeight} - 2.0\text{ m}$), leaving 100% of the authentic Martian elevation geometry fully exposed and unclipped.
- **Regolith Texture UV Rescaling:** Adjusted UV texture coordinates to $0.25\text{ m}^{-1}$ (4-metre repeating tiling), eliminating texture stretching across steep crater rims.
- **Multi-Resolution Terrain Support:** Maintained the 3-tier hierarchy: Regional orbital DTM (1024x1024, 1 m/pixel from HiRISE), Local site terrain reconstruction (`mars_terrain0.glb`, `mars_terrain1.glb`), and wheel-contact terramechanics.

### 16.4 Physical Scale Validation & Microsoft Smart App Control Resolution
- **Strict 1:1 SI Metric Verification:** Verified all dimensional scales across the pipeline: 1 world unit = 1 metre. Perseverance Rover model dimensions ($2.33\text{ m}$ body width / $2.65\text{ m}$ track width, $1.63\text{ m}$ body height / $2.22\text{ m}$ total height, $3.11\text{ m}$ length) and Jezero Crater DTM grid dimensions ($1024\text{ m} \times 1024\text{ m}$) conform precisely to NASA mission specifications.
- **Smart App Control (SAC) Mitigation:** Removed legacy `TorqueLabNative.exe` copies from CMake post-build rules. The application now compiles strictly to `ProjectARES.exe` and `ProjectARESTests.exe`. Clean execution is achieved directly from PowerShell, Visual Studio (F5 / Ctrl+F5), or terminal without Smart App Control publisher reputation blocks.

### 16.5 Comprehensive Test Benchmarks (51 / 51 Passed)
- **51 / 51 Test Suites Passing (0 Failed Assertions):**
  - **Unit 35:** 6-Wheel Rocker-Bogie Kinematic Articulation, Ackermann Steering, and Differential Coupling.
  - **Unit 36:** Rover Physical Scale & NASA Jezero Crater DTM Scale Verification.
  - **Unit 37:** Perseverance Rover Mechanical Articulation, 6-Wheel Rotation & Kinematic Joint Closure.
  - **Unit 38:** Wheel Collision Geometry, Terrain Elevation Mapping & Anti-Sinkage Verification.
  - **Units 1–34, Validation Tests A–J, Controlled Tests 1–3:** 100% passing.
- **Headless Smoke Test (`.\build\ProjectARES.exe --smoke-test`):** PASS | Eq Load Err: `0.00%` | Accel Squat Pitch: `+10.30 deg` @ 4 km/h | Brake Dive Pitch: `+7.06 deg`.
- **Live OpenGL Timing Diagnostics (`.\build\ProjectARES.exe --diag`):**
  - **~900–1040 FPS** on NVIDIA GeForce RTX 5050 Laptop GPU (`phys=0.06ms, rnd=0.3ms, ui=0.8ms`).
  - Smooth 6-wheel rover mobility reaching authentic planetary exploration speeds ($3.8\text{ km/h} \approx 1.04\text{ m/s}$).

---

## 18. Phase 6: Mechanical Stability, Multi-Point Reset & Visual Model Fidelity

### 18.1 Forward Pitch/Creep Balance & True Physical Load Transfer
- **Elimination of Centerline Load Collapse:** Previously, suspension reaction forces were partitioned as 31.4% at the rocker pivots and 68.6% at the centerline differential pivot ($X = 0.0\text{ m}$). Applying 68.6% of vertical load at the centerline eliminated 68.6% of the chassis roll stiffness, causing severe roll drift on Martian slopes.
- **Authentic Reaction Force Points:** Reaction forces now act at the effective rocker trunnion planes ($X = \pm 0.806\text{ m}$), preserving 100% of the natural roll restoring moment $(f_{\text{Left}} - f_{\text{Right}}) \times 0.806\text{ m}$, while maintaining zero pitch moment about CoM at $Z = 0.0\text{ m}$.
- **Decoupled Mid-Air Contact:** Tightened contact detection threshold to $1.2\text{ cm}$ below hub seating, preventing airborne wheels from artificially applying full $W/6$ normal load and driving the vehicle while suspended in mid-air.

### 18.2 Terrain-Aware Multi-Point Footprint Reset Engine
- **Multi-Point Terrain Probing (`computeTerrainAwareSpawnPose` in `PhysicsWorld.cpp`):**
  - Probes 6 wheel footprints and 5 chassis belly points against the active DTM / elevation map.
  - Computes the average terrain normal vector $\mathbf{n}_{\text{terrain}}$, constructing a stable orthonormal chassis basis $[\mathbf{u}_{\text{left}}, \mathbf{u}_{\text{up}}, \mathbf{u}_{\text{forward}}]$ aligned to the local surface incline via Shepperd's quaternion algorithm (`quatFromBasis`).
  - Evaluates minimum clearance across belly points ($> 0.25\text{ m}$) and calculates exact flush wheel seating height, eliminating underground chassis spawns and shock-induced flying wheels upon reset.

### 18.3 Visual Hierarchy Alignment & Sub-Part Matrix Centering
- **Blender 1:1 Matrix Centering (`VehicleRendererGL.cpp`):**
  - Removed redundant manual translation offsets ($-D_0, -P_L, -P_R$) which previously shifted geometry 71 cm away from physical joints.
  - Each suspension part is authored relative to its own joint origin in model space; multiplying $M_{\text{link}} \cdot M_{\text{subPartLocal}}$ directly places every rocker, bogie, strut, and knuckle into perfect mathematical and visual coincidence with the original Blender model hierarchy.

### 18.4 Bullet GJK Simplex Precision in Flat Test Pad
- **Resolution of Raycast Simplex Epsilon:**
  - Discovered that a single $1000\text{ m} \times 1000\text{ m} \times 1\text{ m}$ `btBoxShape` (`Vec3{500.0, 0.5, 500.0}`) caused Bullet's GJK subsimplex raycast to exhibit a $6.4\text{ cm}$ bounding-radius stopping error, artificially lifting right-side wheels in test suites.
  - Resized the baseline flat test pad to a $160\text{ m} \times 160\text{ m}$ box (`Vec3{80.0, 0.5, 80.0}`), reducing the GJK simplex raycast error to $< 0.5\text{ mm}$ and achieving an exact 6-wheel normal load balance with $0.00018\%$ error.

### 18.5 Autonomous Rover Reinforcement Learning (RL) Interface
- **State & Action Architecture (`RoverRLInterface.h` / `RoverRLInterface.cpp`):**
  - Defines `RoverObservationState` capturing full 6-DOF chassis linear/angular state, Euler angles, accelerations, rocker/bogie pitch angles and pitch rates, 6-wheel speeds, steering angles, slip ratios, normal loads, traction forces, terrain slope, and belly clearance.
  - Defines `RoverActuatorCommand` supporting 3 operational control regimes: `DirectTorque` (individual per-wheel torque control), `TargetVelocity` (planetary closed-loop cruise control), and `TeleopNormalized` (standard teleoperation).






