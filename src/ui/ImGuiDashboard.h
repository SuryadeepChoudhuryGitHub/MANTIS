#pragma once
/**
 * Dear ImGui Vehicle Instrument Cluster, Telemetry Inspector, 4-Band Oscilloscope,
 * and Live Parameter Tuning Laboratory.
 *
 * All displayed values are read directly from the live VehicleTelemetrySnapshot.
 */

#include <array>
#include <deque>
#include <functional>
#include <string>
#include "physics/PhysicsWorld.h"
#include "renderer/TrackRendererGL.h"
#include "navigation/MinimapRenderer.h"
#include "navigation/NavGrid.h"
#include "navigation/AStarPlanner.h"
#include "navigation/PathFollower.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

class InputControllerGLFW;
struct LoadedVehicleModel;
struct GpuHardwareInfo;
class TerrainManager;

struct DashboardCallbacks {
    std::function<void()> onReset;
    std::function<void()> onTogglePause;
    std::function<void()> onCycleCamera;
    std::function<void()> onToggleTelemetry;
    std::function<void()> onToggleGizmos3D;
    std::function<void()> onToggleTransmission;
    std::function<void(const SpawnPreset&)> onSelectPreset;
    std::function<void(GraphicsQuality)> onChangeQuality;
    std::function<void()> onRunValidationCheck;
    std::function<void(bool)> onToggleImportedModel;
    std::function<void()> onToggleVisualDebug;
    std::function<void(const std::string&)> onSelectVehicle;
    std::function<void(const std::string&)> onSelectMap;
    std::function<void()> onSaveVehicleJson;
    std::function<void()> onReloadVehicleJson;
    std::function<void()> onResetVehicleDefaults;
    std::function<void(bool)> onToggleCollisionWireframe;
    std::function<void(CollisionAlgorithmStrategy)> onRegenerateCollision;
    std::function<void(WheelCollisionFidelity)> onSelectCollisionFidelity;
    std::function<void(bool)> onToggleStationaryHold;
    std::function<void(const Vec3&)> onSelectDestination;
    std::function<void()> onStartNav;
    std::function<void()> onPauseNav;
    std::function<void()> onResumeNav;
    std::function<void()> onStopNav;
    std::function<void()> onClearDestination;
    std::function<void(uint32_t)> onRegenerateProceduralTerrain;
    std::function<void(const Vec3&, double)> onAddWorldObstacle;
    std::function<void()> onClearWorldObstacles;
};

class ImGuiDashboard {
public:
    explicit ImGuiDashboard(DashboardCallbacks callbacks);

    void applyTheme();

    void render(
        VehicleDynamics& vehicle,
        const VehicleTelemetrySnapshot& snap,
        const VehicleControlInputs& inputs,
        bool paused,
        const std::string& cameraLabel,
        bool gizmosEnabled,
        GraphicsQuality currentQuality,
        double fps,
        double physicsStepTimeMs,
        int windowWidth,
        int windowHeight,
        bool useImportedModel = true,
        bool hasImportedModel = true,
        const std::string& modelName = "BMW M4 Coupe",
        bool visualDebugEnabled = false,
        bool collisionWireframeEnabled = true,
        InputControllerGLFW* inputController = nullptr,
        const LoadedVehicleModel* activeModel = nullptr,
        float* exposure = nullptr,
        float* iblIntensity = nullptr,
        int* diagnosticMode = nullptr,
        const GpuHardwareInfo* gpuInfo = nullptr,
        const TerrainManager* terrainManager = nullptr,
        MinimapRenderer* minimapRenderer = nullptr,
        PathFollower* pathFollower = nullptr,
        NavGrid* navGrid = nullptr,
        AStarPlanner* planner = nullptr,
        Vec3* selectedDestination = nullptr,
        bool* hasDestination = nullptr
    );

    void showValidationReport(const std::string& message, bool isSuccess = true);

    struct VehicleOption {
        std::string id;
        std::string name;
    };
    void setAvailableVehicles(std::vector<VehicleOption> vehicles) {
        m_availableVehicles = std::move(vehicles);
    }

    bool telemetryEnabled{true};
    bool minimapEnabled{true};
    bool navigationPanelEnabled{true};

    enum class MinimapPathColorMode {
        BrightRed,  // Mission High-Vis Neon Crimson / Bright Red (Default)
        DarkBlue    // Deep Royal Navy Blue
    };
    MinimapPathColorMode minimapPathColor{MinimapPathColorMode::BrightRed};

private:
    std::vector<VehicleOption> m_availableVehicles;
    struct TelemetrySample {
        float speedKmh{0.0f};
        float longAccel{0.0f};
        float latAccel{0.0f};
        float pitchDeg{0.0f};
        float rollDeg{0.0f};
        std::array<float, 4> compMM{0.0f, 0.0f, 0.0f, 0.0f};
    };

    void renderTopBar(
        const std::string& vehicleId,
        bool paused,
        const std::string& cameraLabel,
        bool gizmosEnabled,
        GraphicsQuality currentQuality,
        int windowWidth,
        bool useImportedModel,
        bool hasImportedModel,
        bool visualDebugEnabled,
        const GpuHardwareInfo* gpuInfo = nullptr,
        const TerrainManager* terrainManager = nullptr
    );
    void renderLeftPanels(
        VehicleDynamics& vehicle,
        int windowHeight,
        bool useImportedModel,
        bool hasImportedModel,
        const std::string& modelName,
        bool visualDebugEnabled,
        bool collisionWireframeEnabled,
        InputControllerGLFW* inputController,
        const LoadedVehicleModel* activeModel = nullptr,
        float* exposure = nullptr,
        float* iblIntensity = nullptr,
        int* diagnosticMode = nullptr,
        const GpuHardwareInfo* gpuInfo = nullptr,
        const TerrainManager* terrainManager = nullptr
    );
    void renderMinimapAndNavigationWindow(
        VehicleDynamics& vehicle,
        MinimapRenderer& minimap,
        PathFollower& pathFollower,
        NavGrid& navGrid,
        AStarPlanner& planner,
        Vec3& selectedDestination,
        bool& hasDestination,
        int windowWidth,
        int windowHeight
    );
    void renderControllerSettings(InputControllerGLFW* inputController);
    void renderRightTelemetryPanel(
        const VehicleTelemetrySnapshot& snap,
        double fps,
        double physicsStepTimeMs,
        int windowWidth,
        int windowHeight,
        const GpuHardwareInfo* gpuInfo = nullptr
    );
    void renderBottomInstrumentCluster(
        const VehicleDynamics& vehicle,
        const VehicleTelemetrySnapshot& snap,
        const VehicleControlInputs& inputs,
        const PathFollower* pathFollower,
        const Vec3* selectedDestination,
        bool hasDestination,
        int windowWidth,
        int windowHeight
    );
    void drawOscilloscopeCanvas(float width, float height);

    DashboardCallbacks m_callbacks;
    std::deque<TelemetrySample> m_history;
    static constexpr size_t MAX_SAMPLES = 180; // 3 seconds at 60 Hz

    std::string m_validationMessage;
    bool m_validationSuccess{true};
    double m_validationToastTimer{0.0};
};

} // namespace torquelab
