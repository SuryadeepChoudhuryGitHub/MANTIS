# MANTIS
### Mars Autonomous Navigation & Terrain Intelligent Simulator
**High-Fidelity Planetary Rover Mobility, Rocker-Bogie Kinematics, Autonomous Pathfinding & Real-Time Extraterrestrial Physics Simulation Engine**

*Built with Native C++20, OpenGL 3.3 Core Profile, Bullet Physics 3, and Dear ImGui.*

---

## 1. Project Overview & Physical Philosophy

**MANTIS** (*Mars Autonomous Navigation & Terrain Intelligent Simulator*) is an advanced, physics-first planetary rover mobility simulation and autonomous exploration laboratory. Designed specifically for extreme off-road extraterrestrial traversal, MANTIS simulates the complex dynamics of robotic explorers traversing rugged martian and lunar environments with uncompromising physical fidelity.

### Core Physical Axiom (Non-Negotiable)
**Every rover motion, articulation, wheel contact, and trajectory emerges strictly from coherent physical laws:**
- **Zero Fake Animations:** No predefined wheel roll cycles, artificial body tilt animations, or visual interpolation hacks.
- **Zero Speed Multipliers:** Vehicle velocity is strictly the mathematical integration of wheel-ground traction, motor torques, aerodynamic drag, rolling resistance, and slope forces.
- **Authentic Gravitation & Atmosphere:** Full support for Mars ($g = 3.71\text{ m/s}^2, \rho = 0.020\text{ kg/m}^3$) and Moon ($g = 1.62\text{ m/s}^2, \rho = 0.000\text{ kg/m}^3$).
- **Strict SI Metric Units & Right-Handed Frame:** All calculations operate in $\text{m}, \text{kg}, \text{s}, \text{N}, \text{N}\cdot\text{m}, \text{rad}, \text{rad/s}$ in a right-handed frame ($+X\text{ Left}, +Y\text{ Up}, +Z\text{ Forward}$).

---

## 2. System Architecture & Key Capabilities

```
                                  +-------------------------------------------------------+
                                  |                     MANTIS ENGINE                     |
                                  +-------------------------------------------------------+
                                                              |
                  +-------------------------+-----------------+-------------------------+
                  |                         |                                           |
                  v                         v                                           v
       +--------------------+    +--------------------+                      +--------------------+
       |  PHYSICS & ROVER   |    |    TERRAIN & MAP   |                      |    AUTONOMOUS NAV  |
       +--------------------+    +--------------------+                      +--------------------+
       | - Rocker-Bogie     |    | - HiRISE Jezero DTM|                      | - 2D NavGrid       |
       | - 6 Hub Motors     |    | - Procedural fBm   |                      | - A* Path Planner  |
       | - 4-Corner Steer   |    | - Craters & Rocks  |                      | - Path Follower    |
       | - Terramechanics   |    | - 2.0m Metric Grid |                      | - Topo Minimap HUD |
       +--------------------+    +--------------------+                      +--------------------+
                  |                         |                                           |
                  +-------------------------+-----------------+-------------------------+
                                                              |
                                                              v
                                             +----------------------------------+
                                             |     RENDERING & USER CONSOLE     |
                                             +----------------------------------+
                                             | - OpenGL 3.3 Core PBR Pipeline   |
                                             | - Cook-Torrance GGX & Shadows    |
                                             | - 3-Column Exploration Console   |
                                             | - 58-Suite Automated Test Runner |
                                             +----------------------------------+
```

### A. Rocker-Bogie Suspension & Mobility Kinematics
- **Passive Linkage Equilibrium:** Authentic implementation of the flight-proven Mars rocker-bogie mechanism. Larger "rocker" arms on each side pivot from the main chassis and connect to a 2-wheel "bogie" assembly via secondary revolute joints.
- **Differential Balancing Crossbar:** A top-mounted mechanical differential crossbar couples the left and right rockers across the chassis roof. When one side climbs a rock, the differential forces the opposing side downward, equalizing wheel ground contact and halving chassis pitch and roll.
- **Obstacle Clearance:** Enables traversing obstacles up to twice the wheel diameter (over 0.5 meters) without chassis high-centering or tipping.
- **6 Independent Electric Hub Motors:** Direct-drive brushless electric hub motors inside each of the 6 cleated aluminium wheels provide independent drive torque ($N\cdot m$), allowing individual wheel speed synchronization and zero-creep parking stiction.
- **4-Corner Ackermann Steering:** Independent steering actuators on the 4 corner wheels (Front-Left, Front-Right, Rear-Left, Rear-Right) with Ackermann inner/outer angle compensation and point-turn zero-radius steering capability.

### B. Planetary Terrain & Elevation Mapping System
- **Mars Jezero Crater Regional DTM:** High-resolution digital terrain model derived from NASA MRO HiRISE stereo photogrammetry (1024×1024 grid, 1.0-meter horizontal spacing, 42.4 m elevation relief). Features central-difference vertex normal reconstruction for seamless lighting.
- **Procedural Planetary Terrain Generator:** Multi-octave fractal Brownian motion (fBm) Perlin noise with analytical crater depression/ejecta-rim mathematical profiles, coordinate domain warping to break Cartesian grid artifacts, and procedural basalt rock boulders with Bullet convex hull collision bodies.
- **High-Contrast 2.0-Meter Metric Ground Grid:** Procedural surface shader with anti-aliased dark basalt seams (`0x220c05` on `0xb85d38`) forming a prominent 2.0 m × 2.0 m metric coordinate grid matching the rover's wheelbase (2.26 m) and track width (2.0 m) for instant visual perception of forward velocity.

### C. Autonomous Navigation & Pathfinding Pipeline
- **2D Traversability Navigation Grid (`NavGrid`):** Real-time spatial grid sampling terrain elevation, slope gradients, rollover thresholds ($>25^\circ$), and obstacle safety margins around the rover footprint.
- **Optimal A* Path Planner (`AStarPlanner`):** Heuristically admissible $A^*$ search with directional slope penalty costs, rock clearance weighting, and string-pulling line-of-sight path smoothing.
- **Adaptive Path Follower (`PathFollower`):** Closed-loop pursuit controller with heading-error speed adaptation, incline pitch feed-forward torque, stuck detection/auto-recovery, and seamless manual driving override with a 0.8-second auto-resume hold.
- **Interactive Topographical Minimap HUD (`MinimapRenderer`):** Dual-layer topographical minimap with color-coded slope safety contours (Green: nominal $<15^\circ$, Yellow: caution $15-25^\circ$, Red: rollover hazard $>25^\circ$), elevation shading, planned waypoint paths, rover heading arrow, and 1-click destination dispatch.

### D. Active Planetary Rover Fleet
1. **NASA Perseverance Rover (Mars 2020):** Primary active vehicle. High-fidelity 536-node visual glTF 2.0 model with articulated rockers, bogies, 6 cleated aluminium wheels, robotic science arm, SuperCam mast, MMRTG power source, and SHERLOC turret.
2. **NASA Curiosity Rover (MSL):** 9-part visual mesh (63,516 vertices, 51,305 triangles) with operational rocker-bogie kinematics and science deck.
3. **NASA Opportunity Rover (MER):** Solar-powered exploration rover (44,117 vertices, 31,450 triangles) with authentic lightweight chassis and panoramic camera mast.
4. **MANTIS Prototype Rover:** Procedural baseline rover platform for automated regression testing and rapid suspension tuning.

### E. Planetary Exploration Console & Telemetry (Dear ImGui)
- **Top Bar:** Quick planetary gravity indicator ($g$), map selection combo, camera view buttons, force vector gizmo toggles, and instant physics verifier.
- **Left Control Panel:** 6-wheel rocker-bogie angle monitor, 6-wheel ground contact interaction table (Normal Load $N$, Drive Torque $N\cdot m$, Traction $N$, Slip Ratio, Contact state), zero-creep stiction toggle, and real-time physical tuning sliders (Mass, CoM offset, spring rates, regolith $\mu$, steering limits).
- **Right Telemetry Panel:** 4-wheel physics balance summary, component hierarchy tree, and 4-band real-time oscilloscope tracking speed (0–2 m/s), acceleration ($\pm 4\text{ m/s}^2$), attitude pitch/roll ($\pm 30^\circ$), and suspension travel (0–250 mm).
- **Bottom Rover Exploration Console:**
  - *Column 0 (Navigation & Position):* World coordinates $(X, Z)$, Elevation $Y$ (MSL), 8-point compass heading, and forward velocity in m/s and km/h.
  - *Column 1 (Attitude & Slope Safety):* Pitch & Roll degrees, terrain slope hazard badge (`[NOMINAL]`, `[CAUTION]`, `[TILT HAZARD]`), linkage pitch angles (Left Rocker, Right Rocker, Differential Bar), and 6-wheel contact status pills (`[FL] [FR] [ML] [MR] [RL] [RR]`).
  - *Column 2 (Autonomous Mission & Motors):* Mission status badge (`NAVIGATING`, `ARRIVED`, `PAUSED`, `IDLE`, `STUCK`), destination coordinates, distance remaining, progress bar, 6-wheel hub motor power %, and holding brake state.

---

## 3. Project Dependencies & Prerequisites

MANTIS is designed to build out-of-the-box with minimal external setup. Most libraries are automatically fetched and compiled via CMake `FetchContent` or vendored in `third_party/`.

### System Requirements
| Component | Minimum Specification | Recommended |
| :--- | :--- | :--- |
| **Operating System** | Windows 10 / 11 (64-bit) | Windows 11 (64-bit) |
| **Compiler** | C++20 compliant compiler (GCC 13+, Clang 16+, MSVC 19.38+) | MinGW-w64 GCC 16 (UCRT64) or Visual Studio 2026 MSVC |
| **Build System** | CMake 3.20+ | CMake 3.28+ |
| **GPU & Drivers** | OpenGL 3.3 Core Profile capable GPU | Dedicated NVIDIA GeForce / AMD Radeon GPU with vendor OpenGL drivers |

### Core Software Dependencies
1. **OpenGL 3.3 Core Profile:** High-performance hardware-accelerated 3D rendering pipeline.
2. **GLAD:** OpenGL function pointer loader (`third_party/glad/`).
3. **GLFW 3.4:** Cross-platform window creation, OpenGL context management, and raw keyboard/mouse/gamepad polling (fetched via CMake `FetchContent`).
4. **Bullet Physics 3.25:** Real-time 3D rigid-body collision detection, contact constraint resolution, and continuous collision detection (CCD) (`btDiscreteDynamicsWorld`, fetched via CMake `FetchContent`).
5. **Dear ImGui v1.91+:** Immediate-mode graphical user interface for real-time telemetry, console displays, and interactive parameter tuning (`third_party/imgui/`).
6. **cgltf:** Lightweight, single-header C99 glTF 2.0 and binary GLB parser (`third_party/cgltf/`).
7. **stb_image:** Public-domain image decompression for PBR textures (`third_party/stb/`).
8. **V-HACD:** Volumetric Hierarchical Approximate Convex Decomposition for automatic model-derived collision hull generation (`bullet3/Extras/VHACD/`).
9. **Draco (Optional):** Google Draco 3D geometric compression library for decoded meshes (auto-detected via MSYS2 / system paths).

---

## 4. How to Build & Run

### Quick 1-Click Launchers (Windows)
- Double-click **`run.bat`** in the project root to launch the interactive simulation.
- Double-click **`run_tests.bat`** to execute the complete 58-suite automated validation test suite.

---

### Method A: MinGW-w64 GCC Workflow (UCRT64 / MSYS2)

Open PowerShell in the workspace root (`d:\# My Folder\My Work\Visual Studio script\C++\ProjectAres`):

```powershell
# 1. Configure the build system (using preset or manual command)
cmake --preset windows-mingw
# or manually:
cmake -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_FULLY_DISCONNECTED=ON

# 2. Compile the interactive simulator and the test suite in parallel
cmake --build build --target MANTIS MANTISTests -j 6

# 3. Launch the interactive simulation window (1440x900 OpenGL 3.3)
.\build\MANTIS.exe

# 4. Run the 58-suite automated physics & navigation validation tests
.\build\MANTISTests.exe
# or via the primary binary:
.\build\MANTIS.exe --tests

# 5. Run headless smoke test (quick headless sanity check)
.\build\MANTIS.exe --smoke-test

# 6. Run live timing diagnostics & telemetry table
.\build\MANTIS.exe --diag
```

---

### Method B: Visual Studio 2022 / 2026 (MSVC) Workflow

Open Developer PowerShell for Visual Studio:

```powershell
# 1. Configure using the MSVC preset
cmake --preset windows-msvc

# 2. Build Release configuration
cmake --build build --config Release --target MANTIS MANTISTests -j 6

# 3. Run the executable from build/Release/
.\build\Release\MANTIS.exe
.\build\Release\MANTISTests.exe
```

---

### Windows 11 Smart App Control (SAC) Note
On Windows 11 systems with **Smart App Control (SAC)** enabled:
- SAC automatically flags newly compiled unsigned binaries (`Publisher: Unknown`) when double-clicked through Windows Explorer.
- **Recommended Developer Fix:** Open **Windows Security** $\rightarrow$ **App & browser control** $\rightarrow$ **Smart App Control settings** $\rightarrow$ set to **Off**. *(Your system remains fully protected by Windows Defender Antivirus and Real-Time Protection).*
- Alternatively, launching directly via PowerShell (`.\build\MANTIS.exe`), Visual Studio (F5), or via `run.bat` runs cleanly without SAC intervention.

---

## 5. Controls & User Interface Reference

### Rover Driving Controls
| Action | Keyboard | Xbox Gamepad | PlayStation Controller |
| :--- | :--- | :--- | :--- |
| **Drive Forward** | `W` / `Up Arrow` | `Right Trigger (RT)` | `R2 Trigger` |
| **Reverse / Brake** | `S` / `Down Arrow` | `Left Trigger (LT)` | `L2 Trigger` |
| **Steer Left** | `A` / `Left Arrow` | `Left Stick Left` | `Left Stick Left` |
| **Steer Right** | `D` / `Right Arrow` | `Left Stick Right` | `Left Stick Right` |
| **Holding Brake / Zero-Creep** | `Spacebar` | `X Button` | `Square Button` |
| **Hold-to-Recover Vehicle Reset** | `R` (Hold 0.5s) | `Y Button` (Hold 0.5s) | `Triangle` (Hold 0.5s) |
| **Cycle Rover Model** | `V` | `B Button` / `RB` | `Circle` / `R1` |
| **Cycle Camera Mode** | `C` | `Right Stick Click (RS)` | `R3 Click` |
| **Free Look-Around Camera** | Mouse Drag | `Right Analog Stick` | `Right Analog Stick` |

### Autonomous Navigation & Map Interaction
| Key / Input | Action |
| :--- | :--- |
| **`M`** | Toggle Topographical Minimap HUD on/off. |
| **Left Click on Minimap** | Set new target coordinates $\rightarrow$ calculates optimal $A^*$ path and immediately starts autonomous driving. |
| **`Escape`** | Cancel active autonomous navigation mission and halt rover. |
| **Manual Override** | Tapping any drive input (`W/A/S/D`) immediately yields control to the pilot; releasing controls auto-resumes autonomous waypoint navigation after 0.8s. |

---

## 6. Automated Validation & Physics Verification

MANTIS incorporates a rigorous, automated 58-suite unit, validation, and comparative regression test harness (`MANTISTests.exe`). Every build continuously validates the mathematical and physical integrity of the simulation:

- **Units 1–8:** Suspension kinematics, Hooke springs, dampers, Pacejka combined slip friction circles, Ackermann steering angles, and aerodynamic drag.
- **Units 9–12:** V-HACD collision decomposition, topological convex hulls, CCD, and obstacle collision dynamics.
- **Units 13–15:** Controller input normalization, baseline calibration guards, and physics-driven haptics.
- **Units 16–20:** Volumetric wheel contact, curb climbing, fender clearance, and zero-creep parking stiction.
- **Units 21–30:** Multi-point contact patch integration, structural tyre compliance, and vehicle asset validation.
- **Units 31–38:** Mars environmental gravity, HiRISE DTM coordinate alignment, rocker-bogie differential equilibrium, and wheel anti-sinkage elevation mapping.
- **Units 39–45:** Multi-octave procedural terrain generation, crater profiles, rock collision, NavGrid traversability, and $A^*$ pathfinding admissibility.
- **Validation Tests A–J:** Static equilibrium, longitudinal load transfer, service braking dive, cornering roll, low-grip ice surfaces, time-step independence (30 vs 60 vs 120 FPS), and coasting energy conservation.
- **Controlled Tests 1–3:** Analytical force verification ($a = F/m$), zero-resistance comparisons, and scale sanity ($d = v \cdot t$).

```text
====================================================================
 MANTIS — Automated Physics & Autonomous Rover Validation Suite
====================================================================
 Summary: 58 / 58 test suites passed (0 failed assertions).
====================================================================
```

---

## 7. Directory & Workspace Structure

```
ProjectAres/
├── CMakeLists.txt                # Root CMake project file (targets: MANTIS, MANTISTests)
├── CMakePresets.json             # Build configuration presets for MSVC and MinGW-w64
├── run.bat                       # 1-click launcher for MANTIS interactive simulator
├── run_tests.bat                 # 1-click launcher for MANTIS automated test suite
├── Commands.txt                  # CLI reference cheat-sheet
├── GEMINI.md                     # AI context & single-source-of-truth project memory
├── README.md                     # Comprehensive project documentation
├── model/                        # 3D glTF 2.0 assets (Perseverance, Curiosity, Opportunity, DTM)
│   ├── PerseveranceRover.glb     # NASA Perseverance Rover primary 3D asset
│   ├── terrain/                  # High-resolution Mars digital elevation models
│   └── cache/                    # Pre-baked convex collision hull caches (.json)
├── shaders/                      # GLSL 330 core shaders
│   ├── phong_shadow.vert/.frag   # Cook-Torrance PBR, shadows & metric surface grid
│   ├── shadow_depth.vert/.frag   # Cascaded directional shadow map generation
│   └── debug_line.vert/.frag     # 3D force vector gizmos & path trajectory lines
├── src/
│   ├── app/                      # Application lifecycle & main CLI entry point
│   ├── input/                    # Controller mapping, baseline calibration & XInput haptics
│   ├── navigation/               # NavGrid, AStarPlanner, PathFollower, MinimapRenderer
│   ├── physics/                  # Bullet dynamics world, math primitives & surface system
│   ├── renderer/                 # ModelLoaderGLB, TrackRendererGL, VehicleRendererGL, PBR
│   ├── terrain/                  # TerrainManager, ProceduralPlanetMap, ProceduralTerrainGenerator
│   ├── ui/                       # Dear ImGui exploration dashboard & rover telemetry console
│   └── vehicle/                  # RockerBogieSuspension, WheelState, VehicleDynamics
└── tests/                        # 58-suite automated unit, validation & regression harness
```

---

## 8. License & Attribution
- Developed as part of the **AstroGenesis** exploration initiative.
- Rover 3D models and elevation datasets courtesy of **NASA / JPL-Caltech / University of Arizona (HiRISE)**.
