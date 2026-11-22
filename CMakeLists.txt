cmake_minimum_required(VERSION 3.20)

# Ensure CMake 4.x compatibility with third-party subprojects (e.g. Bullet3)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum CMake policy version for subprojects" FORCE)

project(MANTIS
    VERSION 1.0.0
    DESCRIPTION "MANTIS — Autonomous Rover Exploration Simulator (C++20 / OpenGL 3.3 / Bullet / Dear ImGui)"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_compile_options(/W3 /FS /utf-8 /EHsc)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -g)
endif()

include(FetchContent)

# -----------------------------------------------------------------------------
# 1. GLFW 3.4 (Windowing, OpenGL Context, Keyboard/Mouse/Gamepad Input)
# -----------------------------------------------------------------------------
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glfw)

# -----------------------------------------------------------------------------
# 2. Bullet Physics 3.25 (Rigid-Body Dynamics, Collision Detection, Raycasting)
# -----------------------------------------------------------------------------
set(USE_MSVC_RUNTIME_LIBRARY_DLL ON CACHE BOOL "Use dynamic MSVC CRT (/MD)" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_CLSOCKET OFF CACHE BOOL "" FORCE)
set(BUILD_ENET OFF CACHE BOOL "" FORCE)
set(BUILD_PYBULLET OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    bullet3
    GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
    GIT_TAG        3.25
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(bullet3)

# -----------------------------------------------------------------------------
# 3. Dear ImGui v1.91.8 (Immediate-Mode Telemetry, Oscilloscope & Tuning UI)
# -----------------------------------------------------------------------------
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw)

# -----------------------------------------------------------------------------
# 4. Bundled GLAD (OpenGL 3.3 Core Profile Loader)
# -----------------------------------------------------------------------------
add_library(glad STATIC
    third_party/glad/src/glad.c
)
target_include_directories(glad PUBLIC
    third_party/glad/include
)

# -----------------------------------------------------------------------------
# 4b. Volumetric Hierarchical Approximate Convex Decomposition (V-HACD)
# -----------------------------------------------------------------------------
add_library(vhacd_lib STATIC
    ${bullet3_SOURCE_DIR}/Extras/VHACD/src/VHACD.cpp
    ${bullet3_SOURCE_DIR}/Extras/VHACD/src/vhacdICHull.cpp
    ${bullet3_SOURCE_DIR}/Extras/VHACD/src/vhacdManifoldMesh.cpp
    ${bullet3_SOURCE_DIR}/Extras/VHACD/src/vhacdMesh.cpp
    ${bullet3_SOURCE_DIR}/Extras/VHACD/src/vhacdVolume.cpp
)
target_include_directories(vhacd_lib PUBLIC
    ${bullet3_SOURCE_DIR}/Extras/VHACD/public
    ${bullet3_SOURCE_DIR}/Extras/VHACD/inc
    ${bullet3_SOURCE_DIR}/src
)
target_link_libraries(vhacd_lib PUBLIC LinearMath)

find_package(OpenGL REQUIRED)

# -----------------------------------------------------------------------------
# 5. Core Physics & Vehicle Dynamics Static Library (Headless-Capable)
# -----------------------------------------------------------------------------
add_library(TorqueLabPhysics STATIC
    src/physics/PlanetaryEnvironment.cpp
    src/physics/SurfaceSystem.cpp
    src/physics/PhysicsWorld.cpp
    src/physics/VehicleCollisionGenerator.cpp
    src/renderer/Mesh.cpp
    src/renderer/Texture2D.cpp
    src/renderer/ModelLoaderGLB.cpp
    src/renderer/VehicleAssetValidator.cpp
    src/renderer/GpuHardwareInfo.cpp
    src/renderer/Shader.cpp
    src/renderer/EnvironmentMapGL.cpp
    src/renderer/SkyRendererGL.cpp
    src/renderer/ShadowMapFBO.cpp
    src/renderer/TrackRendererGL.cpp
    src/terrain/TerrainManager.cpp
    src/terrain/ProceduralTerrainGenerator.cpp
    src/terrain/ProceduralPlanetMap.cpp
    src/navigation/NavGrid.cpp
    src/navigation/AStarPlanner.cpp
    src/navigation/PathFollower.cpp
    src/navigation/MinimapRenderer.cpp
    src/vehicle/VehicleConfig.cpp
    src/vehicle/SuspensionSystem.cpp
    src/vehicle/TireModel.cpp
    src/vehicle/PneumaticTire.cpp
    src/vehicle/SteeringSystem.cpp
    src/vehicle/Drivetrain.cpp
    src/vehicle/BrakeSystem.cpp
    src/vehicle/Aerodynamics.cpp
    src/vehicle/VehicleBody.cpp
    src/vehicle/RockerBogieSuspension.cpp
    src/vehicle/RoverRLInterface.cpp
    src/vehicle/VehicleDynamics.cpp
)

target_include_directories(TorqueLabPhysics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include
    ${bullet3_SOURCE_DIR}/src
    ${bullet3_SOURCE_DIR}/Extras/VHACD/public
)

target_link_libraries(TorqueLabPhysics PUBLIC
    BulletDynamics
    BulletCollision
    LinearMath
    vhacd_lib
    glad
    OpenGL::GL
)

find_library(DRACO_LIB NAMES draco libdraco PATHS C:/msys64/ucrt64/lib /usr/lib /usr/local/lib)
find_path(DRACO_INCLUDE_DIR NAMES draco/compression/decode.h PATHS C:/msys64/ucrt64/include /usr/include /usr/local/include)
if(DRACO_LIB AND DRACO_INCLUDE_DIR)
    message(STATUS "Found Draco library: ${DRACO_LIB}")
    target_link_libraries(TorqueLabPhysics PUBLIC ${DRACO_LIB})
    target_include_directories(TorqueLabPhysics PUBLIC ${DRACO_INCLUDE_DIR})
    target_compile_definitions(TorqueLabPhysics PUBLIC MANTIS_HAS_DRACO=1 ARES_HAS_DRACO=1)
endif()

# -----------------------------------------------------------------------------
# 6. Native Desktop Application Target (MANTIS)
# -----------------------------------------------------------------------------
add_executable(MANTIS
    src/app/main.cpp
    src/app/Application.cpp
    src/renderer/VehicleRendererGL.cpp
    src/renderer/DebugOverlayGL.cpp
    src/renderer/CameraControllerGL.cpp
    src/input/ControllerConfig.cpp
    src/input/VibrationManager.cpp
    src/input/InputControllerGLFW.cpp
    src/ui/ImGuiDashboard.cpp
)

target_include_directories(MANTIS PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

# -----------------------------------------------------------------------------
# 6a. Validation Test Library
# -----------------------------------------------------------------------------
add_library(TorqueLabTestsLib STATIC
    tests/native_validation_tests.cpp
    src/input/ControllerConfig.cpp
    src/input/VibrationManager.cpp
)
target_compile_definitions(TorqueLabTestsLib PUBLIC NO_STANDALONE_TEST_MAIN)
target_include_directories(TorqueLabTestsLib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
target_link_libraries(TorqueLabTestsLib PUBLIC
    TorqueLabPhysics
    glfw
)

target_link_libraries(MANTIS PRIVATE
    TorqueLabPhysics
    TorqueLabTestsLib
    glad
    glfw
    imgui_lib
    OpenGL::GL
)

# Copy GLSL shaders and vehicle 3D models next to executable after build for portable execution
add_custom_command(TARGET MANTIS POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders"
        "$<TARGET_FILE_DIR:MANTIS>/shaders"
    COMMENT "Copying GLSL shaders to output directory"
)

add_custom_command(TARGET MANTIS POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/model"
        "$<TARGET_FILE_DIR:MANTIS>/model"
    COMMENT "Copying vehicle 3D models to output directory"
)

# Backwards-compatible alias targets
add_custom_target(ProjectARES DEPENDS MANTIS)

# -----------------------------------------------------------------------------
# 7. Automated Comparative Physics & Validation Test Runner (MANTISTests)
# -----------------------------------------------------------------------------
enable_testing()

add_executable(MANTISTests
    tests/test_main.cpp
)

target_include_directories(MANTISTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

target_link_libraries(MANTISTests PRIVATE
    TorqueLabTestsLib
)

add_custom_target(ProjectARESTests DEPENDS MANTISTests)

add_test(NAME MANTISValidationSuite COMMAND MANTISTests)
