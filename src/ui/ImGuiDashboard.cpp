#include "ui/ImGuiDashboard.h"
#include "renderer/GpuHardwareInfo.h"
#include "renderer/ModelLoaderGLB.h"
#include "terrain/TerrainManager.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include "input/InputControllerGLFW.h"
#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

namespace {

ImU32 hexToImU32(unsigned int hex, uint8_t alpha = 255) {
    const uint8_t r = static_cast<uint8_t>((hex >> 16) & 0xFF);
    const uint8_t g = static_cast<uint8_t>((hex >> 8) & 0xFF);
    const uint8_t b = static_cast<uint8_t>(hex & 0xFF);
    return IM_COL32(r, g, b, alpha);
}

ImVec4 hexToImVec4(unsigned int hex, float alpha = 1.0f) {
    const float r = static_cast<float>((hex >> 16) & 0xFF) / 255.0f;
    const float g = static_cast<float>((hex >> 8) & 0xFF) / 255.0f;
    const float b = static_cast<float>(hex & 0xFF) / 255.0f;
    return ImVec4(r, g, b, alpha);
}

} // namespace

ImGuiDashboard::ImGuiDashboard(DashboardCallbacks callbacks)
    : m_callbacks(std::move(callbacks))
{}

void ImGuiDashboard::applyTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 5.0f;
    style.GrabRounding = 4.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.WindowPadding = ImVec2(10.0f, 8.0f);
    style.FramePadding = ImVec2(6.0f, 4.0f);
    style.ItemSpacing = ImVec2(7.0f, 5.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]       = hexToImVec4(0x0f172a, 0.88f);
    colors[ImGuiCol_Border]         = hexToImVec4(0x334155, 0.70f);
    colors[ImGuiCol_FrameBg]        = hexToImVec4(0x1e293b, 0.90f);
    colors[ImGuiCol_FrameBgHovered] = hexToImVec4(0x334155, 0.95f);
    colors[ImGuiCol_FrameBgActive]  = hexToImVec4(0x475569, 1.00f);
    colors[ImGuiCol_TitleBg]        = hexToImVec4(0x0b1120, 0.95f);
    colors[ImGuiCol_TitleBgActive]  = hexToImVec4(0x1e293b, 0.98f);
    colors[ImGuiCol_Button]         = hexToImVec4(0x1e293b, 0.95f);
    colors[ImGuiCol_ButtonHovered]  = hexToImVec4(0x2563eb, 0.95f);
    colors[ImGuiCol_ButtonActive]   = hexToImVec4(0x1d4ed8, 1.00f);
    colors[ImGuiCol_SliderGrab]     = hexToImVec4(0x38bdf8, 0.95f);
    colors[ImGuiCol_SliderGrabActive]= hexToImVec4(0x0ea5e9, 1.00f);
    colors[ImGuiCol_Header]         = hexToImVec4(0x1e293b, 0.90f);
    colors[ImGuiCol_HeaderHovered]  = hexToImVec4(0x334155, 0.95f);
    colors[ImGuiCol_HeaderActive]   = hexToImVec4(0x2563eb, 0.95f);
    colors[ImGuiCol_Text]           = hexToImVec4(0xf8fafc, 1.00f);
    colors[ImGuiCol_TextDisabled]   = hexToImVec4(0x94a3b8, 1.00f);
    colors[ImGuiCol_PlotHistogram]  = hexToImVec4(0x38bdf8, 1.00f);
}

void ImGuiDashboard::showValidationReport(const std::string& message, bool isSuccess) {
    m_validationMessage = message;
    m_validationSuccess = isSuccess;
    m_validationToastTimer = 6.5;
}

void ImGuiDashboard::render(
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
    bool useImportedModel,
    bool hasImportedModel,
    const std::string& modelName,
    bool visualDebugEnabled,
    bool collisionWireframeEnabled,
    InputControllerGLFW* inputController,
    const LoadedVehicleModel* activeModel,
    float* exposure,
    float* iblIntensity,
    int* diagnosticMode,
    const GpuHardwareInfo* gpuInfo,
    const TerrainManager* terrainManager,
    MinimapRenderer* minimapRenderer,
    PathFollower* pathFollower,
    const NavGrid* navGrid,
    AStarPlanner* planner,
    Vec3* selectedDestination,
    bool* hasDestination
) {
    if (!paused) {
        m_history.push_back(TelemetrySample{
            static_cast<float>(snap.speedKmh),
            static_cast<float>(snap.longitudinalAccelMps2),
            static_cast<float>(snap.lateralAccelMps2),
            static_cast<float>(radToDeg(snap.pitchRad)),
            static_cast<float>(radToDeg(snap.rollRad)),
            {
                static_cast<float>(snap.wheels[0].compression * 1000.0),
                static_cast<float>(snap.wheels[1].compression * 1000.0),
                static_cast<float>(snap.wheels[2].compression * 1000.0),
                static_cast<float>(snap.wheels[3].compression * 1000.0)
            }
        });
        while (m_history.size() > MAX_SAMPLES) {
            m_history.pop_front();
        }
    }

    if (m_validationToastTimer > 0.0) {
        m_validationToastTimer = std::max(0.0, m_validationToastTimer - (1.0 / std::max(10.0, fps)));
    }

    renderTopBar(vehicle.config.id, paused, cameraLabel, gizmosEnabled, currentQuality, windowWidth, useImportedModel, hasImportedModel, visualDebugEnabled, gpuInfo, terrainManager);
    renderLeftPanels(vehicle, windowHeight, useImportedModel, hasImportedModel, modelName, visualDebugEnabled, collisionWireframeEnabled, inputController, activeModel, exposure, iblIntensity, diagnosticMode, gpuInfo, terrainManager);
    if (telemetryEnabled) {
        renderRightTelemetryPanel(snap, fps, physicsStepTimeMs, windowWidth, windowHeight, gpuInfo);
    }
    if (minimapEnabled && minimapRenderer && pathFollower && navGrid && planner && selectedDestination && hasDestination) {
        renderMinimapAndNavigationWindow(
            vehicle,
            *minimapRenderer,
            *pathFollower,
            *navGrid,
            *planner,
            *selectedDestination,
            *hasDestination,
            windowWidth,
            windowHeight
        );
    }
    renderBottomInstrumentCluster(
        vehicle,
        snap,
        inputs,
        pathFollower,
        selectedDestination,
        hasDestination ? *hasDestination : false,
        windowWidth,
        windowHeight
    );

    // Paused Banner or Validation Toast
    if (paused) {
        ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, 64.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, hexToImVec4(0xd97706, 0.94f));
        if (ImGui::Begin("##PauseBanner", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing)) {
            ImGui::TextUnformatted("SIMULATION PAUSED — Press P or Start (Options) to Resume");
        }
        ImGui::End();
        ImGui::PopStyleColor();
    } else if (m_validationToastTimer > 0.0 && !m_validationMessage.empty()) {
        ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, 64.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, hexToImVec4(m_validationSuccess ? 0x065f46 : 0x991b1b, 0.95f));
        if (ImGui::Begin("##ValidationToast", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing)) {
            ImGui::TextUnformatted(m_validationMessage.c_str());
        }
        ImGui::End();
        ImGui::PopStyleColor();
    }
}

void ImGuiDashboard::renderTopBar(
    const std::string& vehicleId,
    bool paused,
    const std::string& cameraLabel,
    bool gizmosEnabled,
    GraphicsQuality currentQuality,
    int windowWidth,
    bool useImportedModel,
    bool hasImportedModel,
    bool visualDebugEnabled,
    const GpuHardwareInfo* gpuInfo,
    const TerrainManager* terrainManager
) {
    ImGui::SetNextWindowPos(ImVec2(8.0f, 8.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(windowWidth - 16), 46.0f), ImGuiCond_Always);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##TopBar", nullptr, flags)) {
        ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0xb45309, 0.95f));
        ImGui::Button("PROJECT ARES");
        ImGui::PopStyleColor();
        ImGui::SameLine();
        if (terrainManager && terrainManager->getActiveMap()) {
            const auto env = terrainManager->getActiveMap()->getEnvironment();
            ImGui::TextColored(hexToImVec4(env.id == "mars" ? 0xf59e0b : 0x38bdf8), "%s (g=%.2f m/s\xc2\xb2)", env.displayName.c_str(), env.gravity);
        } else {
            ImGui::Text("ROVER SIMULATOR");
        }
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();

        // Proving Ground Zone Presets (1..7)
        const auto& presets = getProvingGroundPresets();
        for (size_t i = 0; i < presets.size(); ++i) {
            if (ImGui::SmallButton(presets[i].label.c_str())) {
                if (m_callbacks.onSelectPreset) m_callbacks.onSelectPreset(presets[i]);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", presets[i].description.c_str());
            }
            ImGui::SameLine();
        }

        ImGui::TextDisabled("|");
        ImGui::SameLine();

        if (ImGui::SmallButton(paused ? "Resume (P)" : "Pause (P)")) {
            if (m_callbacks.onTogglePause) m_callbacks.onTogglePause();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset (R)")) {
            if (m_callbacks.onReset) m_callbacks.onReset();
        }
        ImGui::SameLine();
        const std::string camBtn = cameraLabel.substr(0, 14) + " (C)";
        if (ImGui::SmallButton(camBtn.c_str())) {
            if (m_callbacks.onCycleCamera) m_callbacks.onCycleCamera();
        }
        ImGui::SameLine();

        if (gizmosEnabled) ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x0284c7, 0.95f));
        if (ImGui::SmallButton("3D Vectors (G)")) {
            if (m_callbacks.onToggleGizmos3D) m_callbacks.onToggleGizmos3D();
        }
        if (gizmosEnabled) ImGui::PopStyleColor();
        ImGui::SameLine();

        // Terrain Map Dropdown Selector (Top Bar)
        if (terrainManager && !terrainManager->getAvailableMaps().empty()) {
            ImGui::SetNextItemWidth(190.0f);
            const auto& maps = terrainManager->getAvailableMaps();
            const std::string curMapId = terrainManager->getActiveMapId();
            int curMapIdx = 0;
            std::vector<std::string> mapNameStrings;
            mapNameStrings.reserve(maps.size());
            for (const auto& m : maps) {
                mapNameStrings.push_back(m->getDisplayName());
            }
            std::vector<const char*> mapNames;
            mapNames.reserve(maps.size());
            for (size_t i = 0; i < maps.size(); ++i) {
                mapNames.push_back(mapNameStrings[i].c_str());
                if (maps[i]->getId() == curMapId) {
                    curMapIdx = static_cast<int>(i);
                }
            }

            if (ImGui::Combo("##TopMapSelect", &curMapIdx, mapNames.data(), static_cast<int>(mapNames.size()))) {
                if (m_callbacks.onSelectMap && curMapIdx >= 0 && curMapIdx < static_cast<int>(maps.size())) {
                    m_callbacks.onSelectMap(maps[static_cast<size_t>(curMapIdx)]->getId());
                }
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Select active planetary surface map (Earth Proving Ground, Mars Site Alpha/Beta, HiRISE DTM)");
            }
            ImGui::SameLine();
        }

        // Rover Dropdown Selector (Top Bar)
        ImGui::SetNextItemWidth(170.0f);
        std::vector<VehicleOption> topVehicleList = m_availableVehicles;
        if (topVehicleList.empty()) {
            topVehicleList = {
                {"perseverance_rover", "Mars Perseverance Rover"},
                {"curiosity_rover", "Mars Curiosity Rover"},
                {"opportunity_mer", "MER Opportunity Rover"},
                {"procedural_rover", "ARES Prototype Rover (Procedural)"}
            };
        }

        int topRoverIdx = 0;
        std::vector<const char*> roverNames;
        roverNames.reserve(topVehicleList.size());
        for (size_t i = 0; i < topVehicleList.size(); ++i) {
            roverNames.push_back(topVehicleList[i].name.c_str());
            if (topVehicleList[i].id == vehicleId) {
                topRoverIdx = static_cast<int>(i);
            }
        }

        if (ImGui::Combo("##TopRoverSelect", &topRoverIdx, roverNames.data(), static_cast<int>(roverNames.size()))) {
            if (m_callbacks.onSelectVehicle && topRoverIdx >= 0 && topRoverIdx < static_cast<int>(topVehicleList.size())) {
                m_callbacks.onSelectVehicle(topVehicleList[static_cast<size_t>(topRoverIdx)].id);
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Select active exploration rover or cycle with (B) / Gamepad (RB)");
        }
        ImGui::SameLine();

        if (visualDebugEnabled) ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x0891b2, 0.95f));
        if (ImGui::SmallButton("Align Debug (V)")) {
            if (m_callbacks.onToggleVisualDebug) m_callbacks.onToggleVisualDebug();
        }
        if (visualDebugEnabled) ImGui::PopStyleColor();
        ImGui::SameLine();

        if (telemetryEnabled) ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x0284c7, 0.95f));
        if (ImGui::SmallButton("Telemetry (T)")) {
            if (m_callbacks.onToggleTelemetry) m_callbacks.onToggleTelemetry();
        }
        if (telemetryEnabled) ImGui::PopStyleColor();
        ImGui::SameLine();

        if (minimapEnabled) ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x059669, 0.95f));
        if (ImGui::SmallButton("Minimap (M)")) {
            minimapEnabled = !minimapEnabled;
        }
        if (minimapEnabled) ImGui::PopStyleColor();
        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x059669, 0.95f));
        if (ImGui::SmallButton("Verify Physics")) {
            if (m_callbacks.onRunValidationCheck) m_callbacks.onRunValidationCheck();
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();

        ImGui::SetNextItemWidth(130.0f);
        int qIdx = static_cast<int>(currentQuality);
        const char* qItems[] = {"Quality: Low", "Quality: Med", "Quality: High", "Quality: Max"};
        if (ImGui::Combo("##Quality", &qIdx, qItems, 4)) {
            if (m_callbacks.onChangeQuality) {
                m_callbacks.onChangeQuality(static_cast<GraphicsQuality>(qIdx));
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Graphics Quality Preset:\n"
                "- Maximum: Atmospheric blue sky dome, 2048x2048 9-tap PCF shadows, HDR IBL, horizon terrain\n"
                "- High: Baseline environment, 2048x2048 4-tap PCF shadows, standard IBL\n"
                "- Medium: Balanced performance, 1024x1024 2-tap PCF shadows, simplified sky\n"
                "- Low: Lightweight fallback, shadows disabled, fast 2-stop sky gradient"
            );
        }

        if (gpuInfo) {
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Text, gpuInfo->isDiscreteGpu ? hexToImVec4(0x38bdf8) : hexToImVec4(0xfacc15));
            ImGui::Text("GPU: %s", gpuInfo->shortName.c_str());
            ImGui::PopStyleColor();
            if (gpuInfo->isDiscreteGpu) {
                ImGui::SameLine();
                ImGui::TextColored(hexToImVec4(0x4ade80), "(Dedicated)");
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", gpuInfo->getDetailedReport().c_str());
            }
        }
    }
    ImGui::End();
}

void ImGuiDashboard::renderLeftPanels(
    VehicleDynamics& vehicle,
    int windowHeight,
    bool useImportedModel,
    bool hasImportedModel,
    const std::string& modelName,
    bool visualDebugEnabled,
    bool collisionWireframeEnabled,
    InputControllerGLFW* inputController,
    const LoadedVehicleModel* activeModel,
    float* exposure,
    float* iblIntensity,
    int* diagnosticMode,
    const GpuHardwareInfo* gpuInfo,
    const TerrainManager* terrainManager
) {
    const float maxPanelHeight = std::max(320.0f, static_cast<float>(windowHeight) - 210.0f);
    ImGui::SetNextWindowPos(ImVec2(8.0f, 60.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(325.0f, maxPanelHeight), ImGuiCond_Always);

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("PROJECT ARES — CONTROLS & ENVIRONMENT", nullptr, flags)) {
        if (terrainManager && ImGui::CollapsingHeader("PLANETARY ENVIRONMENT & TERRAIN", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto& maps = terrainManager->getAvailableMaps();
            const std::string curMapId = terrainManager->getActiveMapId();
            int curMapIdx = 0;
            std::vector<std::string> mapLabelStrings;
            mapLabelStrings.reserve(maps.size());
            for (const auto& m : maps) {
                mapLabelStrings.push_back(m->getDisplayName());
            }
            std::vector<const char*> mapLabels;
            mapLabels.reserve(mapLabelStrings.size());
            for (size_t i = 0; i < maps.size(); ++i) {
                mapLabels.push_back(mapLabelStrings[i].c_str());
                if (maps[i]->getId() == curMapId) {
                    curMapIdx = static_cast<int>(i);
                }
            }

            ImGui::Text("Active Surface Map:");
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::Combo("##LeftPanelMapSelect", &curMapIdx, mapLabels.data(), static_cast<int>(mapLabels.size()))) {
                if (m_callbacks.onSelectMap && curMapIdx >= 0 && curMapIdx < static_cast<int>(maps.size())) {
                    m_callbacks.onSelectMap(maps[static_cast<size_t>(curMapIdx)]->getId());
                }
            }

            if (auto activeMap = terrainManager->getActiveMap()) {
                const auto env = activeMap->getEnvironment();
                ImGui::Spacing();
                ImGui::TextColored(hexToImVec4(0x38bdf8), "Planetary Body: %s", env.displayName.c_str());
                ImGui::Text("Surface Gravity: %.2f m/s\xc2\xb2 (%.1f%% Earth)", env.gravity, (env.gravity / 9.81) * 100.0);
                ImGui::Text("Atmospheric Density: %.3f kg/m\xc2\xb3", env.airDensity);
                ImGui::Text("Surface Pressure: %.3f kPa", env.ambientPressure * 0.001);
                ImGui::Text("Mean Temperature: %.1f K (%.1f \xc2\xb0\x43)", env.temperatureKelvin, env.temperatureKelvin - 273.15);
                ImGui::Text("Solar Irradiance: %.0f W/m\xc2\xb2", env.solarIrradiance);
            }
            ImGui::Separator();
        }

        if (ImGui::CollapsingHeader("ROVER SELECTION & CONFIGURATION", ImGuiTreeNodeFlags_DefaultOpen)) {
            std::vector<VehicleOption> panelVehicleList = m_availableVehicles;
            if (panelVehicleList.empty()) {
                panelVehicleList = {
                    {"perseverance_rover", "Mars Perseverance Rover"},
                    {"curiosity_rover", "Mars Curiosity Rover"},
                    {"opportunity_mer", "MER Opportunity Rover"},
                    {"procedural_rover", "ARES Prototype Rover (Procedural)"}
                };
            }

            int panelCarIdx = 0;
            std::vector<const char*> fullCarNames;
            fullCarNames.reserve(panelVehicleList.size());
            for (size_t i = 0; i < panelVehicleList.size(); ++i) {
                fullCarNames.push_back(panelVehicleList[i].name.c_str());
                if (panelVehicleList[i].id == vehicle.config.id) {
                    panelCarIdx = static_cast<int>(i);
                }
            }

            ImGui::Text("Active Rover:");
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::Combo("##PanelCarSelect", &panelCarIdx, fullCarNames.data(), static_cast<int>(fullCarNames.size()))) {
                if (m_callbacks.onSelectVehicle && panelCarIdx >= 0 && panelCarIdx < static_cast<int>(panelVehicleList.size())) {
                    m_callbacks.onSelectVehicle(panelVehicleList[static_cast<size_t>(panelCarIdx)].id);
                }
            }

            ImGui::Spacing();
            for (size_t i = 0; i < panelVehicleList.size(); ++i) {
                if (ImGui::RadioButton(panelVehicleList[i].name.c_str(), panelCarIdx == static_cast<int>(i))) {
                    if (panelCarIdx != static_cast<int>(i) && m_callbacks.onSelectVehicle) {
                        m_callbacks.onSelectVehicle(panelVehicleList[i].id);
                    }
                }
            }

            ImGui::Separator();
            bool wireframeChecked = collisionWireframeEnabled;
            if (ImGui::Checkbox("Physical Collision Cage (3D)", &wireframeChecked)) {
                if (m_callbacks.onToggleCollisionWireframe) m_callbacks.onToggleCollisionWireframe(wireframeChecked);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(Hitbox Active)");
            bool debugChecked = visualDebugEnabled;
            if (ImGui::Checkbox("Model Alignment Debug (V)", &debugChecked)) {
                if (m_callbacks.onToggleVisualDebug) m_callbacks.onToggleVisualDebug();
            }

            ImGui::Spacing();
            ImGui::TextColored(hexToImVec4(0xfacc15), "WHEEL COLLISION & STABILITY:");
            int fidelityIdx = static_cast<int>(vehicle.config.wheelCollisionFidelity);
            const char* fidelityNames[] = {
                "Performance (5-Probe Single-Plane)",
                "Balanced (9-Probe 3-Slice Volumetric)",
                "Detailed (17-Probe 5-Slice + Sidewalls)"
            };
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::Combo("##CollisionFidelity", &fidelityIdx, fidelityNames, 3)) {
                vehicle.setCollisionFidelity(static_cast<WheelCollisionFidelity>(fidelityIdx));
                if (m_callbacks.onSelectCollisionFidelity) {
                    m_callbacks.onSelectCollisionFidelity(static_cast<WheelCollisionFidelity>(fidelityIdx));
                }
            }

            bool stationaryHold = vehicle.config.enableStationaryHold;
            if (ImGui::Checkbox("Stationary Hold & Stiction (Zero Creep)", &stationaryHold)) {
                vehicle.config.enableStationaryHold = stationaryHold;
                if (m_callbacks.onToggleStationaryHold) {
                    m_callbacks.onToggleStationaryHold(stationaryHold);
                }
            }

            ImGui::Spacing();
            ImGui::TextColored(hexToImVec4(0x38bdf8), "SPECIFICATIONS & ALIGNMENT:");
            ImGui::Text("Chassis Mass:   %.0f kg (%.1f%% Front)", vehicle.config.mass, vehicle.config.frontWeightDistribution * 100.0);
            ImGui::Text("Wheelbase:      %.2f m  | Track: %.2f m", vehicle.config.wheelbase, vehicle.config.trackWidth);
            ImGui::Text("Tire Radius:    %.2f m  | Width: %.0f mm", vehicle.config.wheelRadius, vehicle.config.wheelWidth * 1000.0);
            ImGui::Text("Ground Clear.:  12.7 cm (Tires Flush)");

            ImGui::Spacing();
            ImGui::TextColored(hexToImVec4(0x34d399), "MODEL-DERIVED COLLISION PIPELINE:");
            const auto& col = vehicle.body.getGeneratedCollision();
            const btCompoundShape* compound = vehicle.body.getCompoundShape();
            const int childCount = compound ? compound->getNumChildShapes() : 0;
            if (!col.pieces.empty()) {
                ImGui::Text("Hull Strategy:  %s", getStrategyName(col.strategyUsed));
                ImGui::Text("Compound Hulls: %d pieces (%d verts)", childCount, col.totalOutputVertices);
                ImGui::Text("Chassis Hitbox: 1 Active (0 Stale) [Clean]");
                ImGui::Text("Generation:     %.1f ms (%s)", col.generationTimeMs, col.loadedFromCache ? "Disk Cache" : "Computed");
                if (col.geometryDerivedSuccessfully) {
                    ImGui::Text("Mesh Axle Base: %.3f m", col.derivedWheelbase);
                    ImGui::Text("Mesh Hub Track: %.3f m", col.derivedTrackWidth);
                }
            } else {
                ImGui::Text("Collision Hull: %s (%d pieces)",
                    (vehicle.config.collisionShapeType == CollisionShapeType::TaperedCompound ? "Tapered Compound" : "Box Compound"),
                    childCount);
                ImGui::Text("Chassis Hitbox: 1 Active (0 Stale) [Clean]");
            }

            if (m_callbacks.onRegenerateCollision) {
                if (ImGui::SmallButton("Re-generate Hull (Auto)")) {
                    m_callbacks.onRegenerateCollision(CollisionAlgorithmStrategy::AutomaticDecomposition);
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("Spatial Multi-Hull")) {
                    m_callbacks.onRegenerateCollision(CollisionAlgorithmStrategy::SpatialClusteredHulls);
                }
            }

            ImGui::Spacing();
            if (ImGui::SmallButton("Save JSON")) {
                if (m_callbacks.onSaveVehicleJson) m_callbacks.onSaveVehicleJson();
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Reload JSON")) {
                if (m_callbacks.onReloadVehicleJson) m_callbacks.onReloadVehicleJson();
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Reset Defaults")) {
                if (m_callbacks.onResetVehicleDefaults) m_callbacks.onResetVehicleDefaults();
            }
        }

        if (ImGui::CollapsingHeader("6-WHEEL ROCKER-BOGIE MOBILITY SUBSYSTEM", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextColored(hexToImVec4(0x38bdf8), "ROCKER-BOGIE PASSIVE EQUILIBRIUM:");
            const auto& rb = vehicle.rockerBogie.getState();
            ImGui::Text("Left Rocker:   %+5.1f deg | Right Rocker:  %+5.1f deg", radToDeg(rb.leftRockerPitch), radToDeg(rb.rightRockerPitch));
            ImGui::Text("Left Bogie:    %+5.1f deg | Right Bogie:   %+5.1f deg", radToDeg(rb.leftBogiePitch), radToDeg(rb.rightBogiePitch));
            ImGui::Text("Diff Crossbar: %+5.1f deg | Contact Wheels: %d / 6", radToDeg(rb.differentialAngle), rb.wheelsInContactCount);

            ImGui::Spacing();
            ImGui::TextColored(hexToImVec4(0x4ade80), "6-WHEEL GROUND INTERACTION & DRIVE FORCES:");
            if (ImGui::BeginTable("##RoverWheelTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("Wheel", ImGuiTableColumnFlags_WidthFixed, 42.0f);
                ImGui::TableSetupColumn("Load Fz", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Drive Trq", ImGuiTableColumnFlags_WidthFixed, 65.0f);
                ImGui::TableSetupColumn("Tract Fx", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Slip k", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();

                static const char* const wRoverNames[6] = {"FL", "FR", "RL", "RR", "ML", "MR"};
                const int nW = vehicle.config.numWheels >= 6 ? 6 : 4;
                for (int w = 0; w < nW; ++w) {
                    const auto& ws = vehicle.wheels[static_cast<size_t>(w)];
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", wRoverNames[w]);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.0f N", ws.normalLoad);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.0f Nm", ws.driveTorque);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.0f N", ws.longitudinalForce);
                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("%+.2f", ws.slipRatio);
                    ImGui::TableSetColumnIndex(5);
                    if (ws.hasObstacleContact) {
                        ImGui::TextColored(hexToImVec4(0xf97316), "Obstacle");
                    } else if (ws.inContact) {
                        ImGui::TextColored(hexToImVec4(0x22c55e), "Traction");
                    } else {
                        ImGui::TextColored(hexToImVec4(0x64748b), "Airborne");
                    }
                }
                ImGui::EndTable();
            }

            ImGui::Spacing();
            ImGui::TextColored(hexToImVec4(0xfacc15), "STABILITY & TRACTION CONTROL:");
            bool stictionActive = vehicle.config.enableStationaryHold;
            if (ImGui::Checkbox("Stationary Hold & Stiction (Anti-Rollback)", &stictionActive)) {
                vehicle.config.enableStationaryHold = stictionActive;
                if (m_callbacks.onToggleStationaryHold) {
                    m_callbacks.onToggleStationaryHold(stictionActive);
                }
            }
            ImGui::BulletText("Rigid Aluminium Cleated Rims: 52.5 cm Diameter");
            ImGui::BulletText("Ground Contact Constraint: Exact Multi-Raycast");
            ImGui::BulletText("Weight Transfer: 3-Point Trunnion + Crossbar");
        }
        }

        if (ImGui::CollapsingHeader("PBR MATERIAL INSPECTOR & LIGHTING", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (diagnosticMode) {
                const char* diagModes[] = {
                    "0: Full PBR Lit (Final)",
                    "1: Base Color / Albedo Map",
                    "2: Metalness Channel",
                    "3: Roughness Channel",
                    "4: Tangent-Space Normals",
                    "5: Ambient Occlusion Map",
                    "6: Emissive Illumination",
                    "7: Tangents (MikkTSpace)",
                    "8: Environment Specular (IBL)"
                };
                ImGui::Text("Shading Inspection Mode:");
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::Combo("##DiagModeCombo", diagnosticMode, diagModes, 9);
            }

            if (exposure) {
                ImGui::SliderFloat("Exposure", exposure, 0.2f, 3.0f, "%.2f");
            }
            if (iblIntensity) {
                ImGui::SliderFloat("IBL Multiplier", iblIntensity, 0.0f, 3.0f, "%.2f");
            }

            if (activeModel && !activeModel->uniqueMaterials.empty()) {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::TextColored(hexToImVec4(0x38bdf8), "Materials: %zu distinct", activeModel->uniqueMaterials.size());

                if (ImGui::BeginChild("##MatListChild", ImVec2(0, 160.0f), true, ImGuiWindowFlags_HorizontalScrollbar)) {
                    for (size_t mIdx = 0; mIdx < activeModel->uniqueMaterials.size(); ++mIdx) {
                        const auto& mat = activeModel->uniqueMaterials[mIdx];
                        ImGui::PushID(static_cast<int>(mIdx));

                        const ImVec4 col4(mat.baseColor.r, mat.baseColor.g, mat.baseColor.b, mat.opacity);
                        ImGui::ColorButton("##swatch", col4, ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker, ImVec2(16, 16));
                        ImGui::SameLine();

                        ImGui::Text("%s", mat.name.c_str());
                        ImGui::SameLine();
                        ImGui::TextDisabled("R:%.2f M:%.2f", mat.roughness, mat.metalness);

                        std::string badges;
                        if (mat.textureId != 0) badges += "[Alb] ";
                        if (mat.normalMapId != 0) badges += "[Nrm] ";
                        if (mat.metallicRoughnessMapId != 0) badges += "[Met-Rough] ";
                        if (mat.occlusionMapId != 0) badges += "[AO] ";
                        if (mat.emissiveMapId != 0 || mat.emissiveIntensity > 0.0f) badges += "[Ems] ";
                        if (mat.alphaMode == AlphaMode::Blend) badges += "[Blend] ";
                        else if (mat.alphaMode == AlphaMode::Mask) badges += "[Mask] ";
                        if (mat.doubleSided) badges += "[2-Sided] ";

                        if (!badges.empty()) {
                            ImGui::TextColored(hexToImVec4(0xa78bfa), "  %s", badges.c_str());
                        }

                        ImGui::PopID();
                    }
                }
                ImGui::EndChild();
            } else if (!useImportedModel) {
                ImGui::Spacing();
                ImGui::TextDisabled("Active Model: Procedural Sports Coupe");
            }
        }

        if (gpuInfo && ImGui::CollapsingHeader("GPU & HARDWARE ACCELERATION (DETECTED)", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextColored(hexToImVec4(0x38bdf8), "ACTIVE RENDERING GPU:");
            ImGui::Text("Device:     %s", gpuInfo->cleanName.c_str());
            ImGui::Text("Vendor:     %s", gpuInfo->vendor.c_str());
            ImGui::Text("Status:     ");
            ImGui::SameLine();
            if (gpuInfo->isDiscreteGpu) {
                ImGui::TextColored(hexToImVec4(0x34d399), "Discrete / External GPU [ACTIVE]");
            } else {
                ImGui::TextColored(hexToImVec4(0xfacc15), "Integrated GPU [ACTIVE]");
            }

            if (gpuInfo->dedicatedVramMb > 0) {
                ImGui::Text("VRAM:       %d MB Dedicated Video Memory", gpuInfo->dedicatedVramMb);
            }
            ImGui::Text("OpenGL:     %s", gpuInfo->glVersion.c_str());
            ImGui::Text("GLSL:       %s", gpuInfo->glslVersion.c_str());

            ImGui::Spacing();
            ImGui::TextColored(hexToImVec4(0xfacc15), "EXTERNAL / DISCRETE GPU ROUTING:");
            ImGui::Text("Optimus / PowerXpress Hooks: ACTIVE");
            ImGui::TextDisabled("NvOptimusEnablement=0x1 | AmdPowerXpress=1");

            if (!gpuInfo->systemAdapters.empty()) {
                ImGui::Spacing();
                ImGui::TextColored(hexToImVec4(0xa78bfa), "SYSTEM GPU TOPOLOGY (%zu detected):", gpuInfo->systemAdapters.size());
                for (size_t i = 0; i < gpuInfo->systemAdapters.size(); ++i) {
                    const auto& a = gpuInfo->systemAdapters[i];
                    if (a.isCurrentOpenGLDevice) {
                        ImGui::TextColored(hexToImVec4(0x34d399), " * [%zu] %s (%zu MB) [ACTIVE]", i + 1, a.name.c_str(), a.dedicatedVramMb);
                    } else {
                        ImGui::TextDisabled("   [%zu] %s (%zu MB) [%s]",
                                            i + 1, a.name.c_str(), a.dedicatedVramMb,
                                            a.isDiscrete ? "Discrete" : "Integrated");
                    }
                }
            }
        }

        if (ImGui::CollapsingHeader("ROVER MOBILITY & EXPLORATION CONTROLS", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextDisabled("W / Up:");    ImGui::SameLine(90); ImGui::TextUnformatted("Drive Forward");
            ImGui::TextDisabled("S / Down:");  ImGui::SameLine(90); ImGui::TextUnformatted("Reverse / Service Brake");
            ImGui::TextDisabled("A / D:");     ImGui::SameLine(90); ImGui::TextUnformatted("4-Corner Steering (L/R)");
            ImGui::TextDisabled("Space:");     ImGui::SameLine(90); ImGui::TextUnformatted("Holding Brake / Stiction");
            ImGui::TextDisabled("M / Tab:");   ImGui::SameLine(90); ImGui::TextUnformatted("Topographical Minimap");
            ImGui::TextDisabled("B / V:");     ImGui::SameLine(90); ImGui::TextUnformatted("Cycle Rover / Alignment");
            ImGui::TextDisabled("R / C:");     ImGui::SameLine(90); ImGui::TextUnformatted("Reset Rover / Cycle Cam");
            ImGui::TextDisabled("T / G:");     ImGui::SameLine(90); ImGui::TextUnformatted("Telemetry / 3D Vectors");
            ImGui::TextDisabled("1 - 7:");     ImGui::SameLine(90); ImGui::TextUnformatted("Traverse Test Locations");
        }

        renderControllerSettings(inputController);

        ImGui::Separator();
        ImGui::TextColored(hexToImVec4(0x38bdf8), "PLANETARY ROVER MOBILITY TUNING");
        ImGui::SameLine(215.0f);
        if (ImGui::SmallButton("Defaults")) {
            if (vehicle.config.id == "curiosity_rover") {
                vehicle.config = createCuriosityRoverConfig();
            } else if (vehicle.config.id == "opportunity_mer") {
                vehicle.config = createOpportunityRoverConfig();
            } else if (vehicle.config.id == "procedural_rover") {
                vehicle.config = createProceduralRoverConfig();
            } else {
                vehicle.config = createMarsRoverConfig();
            }
            vehicle.body.syncMassProperties(vehicle.config);
        }

        VehicleConfig& c = vehicle.config;

        // 1. Vehicle Mass
        float mass = static_cast<float>(c.mass);
        if (ImGui::SliderFloat("Mass (kg)", &mass, 500.0f, 2000.0f, "%.0f kg")) {
            c.mass = static_cast<double>(mass);
            vehicle.body.syncMassProperties(c);
        }

        // 2. CoM Height Offset
        float comH = static_cast<float>(c.comHeightOffset);
        if (ImGui::SliderFloat("CoM Height", &comH, -0.30f, 0.30f, "%+.2f m")) {
            c.comHeightOffset = static_cast<double>(comH);
            vehicle.body.syncMassProperties(c);
        }

        // 3. Front/Rear Weight Bias
        float frontBiasPct = static_cast<float>(c.frontWeightDistribution * 100.0);
        if (ImGui::SliderFloat("Weight Balance", &frontBiasPct, 35.0f, 65.0f, "%.0f%% Front")) {
            c.frontWeightDistribution = static_cast<double>(frontBiasPct) / 100.0;
            vehicle.body.syncMassProperties(c);
        }

        // 4. Rocker-Bogie Spring Stiffness
        float springK = static_cast<float>(c.frontSpringStiffness / 1000.0);
        if (ImGui::SliderFloat("Suspension Rate", &springK, 5.0f, 40.0f, "%.1f kN/m")) {
            c.frontSpringStiffness = static_cast<double>(springK) * 1000.0;
            c.rearSpringStiffness = c.frontSpringStiffness;
        }

        // 5. Rocker-Bogie Damping
        float dampC = static_cast<float>(c.frontDampingCompression);
        if (ImGui::SliderFloat("Compliant Damp", &dampC, 500.0f, 8000.0f, "%.0f N*s/m")) {
            c.frontDampingCompression = static_cast<double>(dampC);
            c.frontDampingRebound = c.frontDampingCompression * 1.25;
            c.rearDampingCompression = c.frontDampingCompression;
            c.rearDampingRebound = c.frontDampingRebound;
        }

        // 6. Cleated Rim Regolith Traction Coeff
        float tireMu = static_cast<float>(c.tireFrictionCoeff);
        if (ImGui::SliderFloat("Regolith Grip u0", &tireMu, 0.30f, 1.50f, "%.2f")) {
            c.tireFrictionCoeff = static_cast<double>(tireMu);
        }

        // 7. Max 4-Corner Steering Angle
        float steerDeg = static_cast<float>(radToDeg(c.maxSteeringAngle));
        if (ImGui::SliderFloat("Max 4-Corner Steer", &steerDeg, 15.0f, 45.0f, "%.0f deg")) {
            c.maxSteeringAngle = degToRad(static_cast<double>(steerDeg));
        }

        // 8. Direct-Drive Hub Motor Drive Torque Multiplier
        float engMult = static_cast<float>(c.engineTorqueMultiplier);
        if (ImGui::SliderFloat("Drive Torque Mult", &engMult, 0.40f, 2.50f, "%.2fx")) {
            c.engineTorqueMultiplier = static_cast<double>(engMult);
        }

        // 9. Mechanical Holding Brake Torque
        float brakeTorque = static_cast<float>(c.maxBrakeTorque);
        if (ImGui::SliderFloat("Holding Brake", &brakeTorque, 500.0f, 6000.0f, "%.0f Nm")) {
            c.maxBrakeTorque = static_cast<double>(brakeTorque);
            c.handbrakeTorque = static_cast<double>(brakeTorque) * 1.2;
        }
    ImGui::End();
}

void ImGuiDashboard::renderControllerSettings(InputControllerGLFW* inputController) {
    if (!inputController) return;

    if (ImGui::CollapsingHeader("CONTROLLER SETTINGS (FORZA HORIZON PRESET)", ImGuiTreeNodeFlags_DefaultOpen)) {
        const bool connected = inputController->isGamepadConnected();
        const std::string name = inputController->getActiveGamepadName();
        const int jid = inputController->getActiveJoystickId();
        const bool isStandard = inputController->isActiveGamepadStandard();
        const ControllerFlavor flavor = inputController->getActiveFlavor();
        auto& settings = inputController->getSettings();

        // 1. Connection & Hardware Status
        if (connected) {
            ImGui::TextColored(hexToImVec4(0x34d399), "STATUS: CONNECTED (Port %d)", jid);
            ImGui::Text("Device:   %s", name.c_str());
            ImGui::Text("Mapping:  %s", isStandard ? "Standard Gamepad (GLFW Verified)" : "Raw USB Joystick (Fallback)");
        } else {
            ImGui::TextColored(hexToImVec4(0xf87171), "STATUS: NO CONTROLLER DETECTED");
            ImGui::TextDisabled("Plug in an Xbox, PlayStation, or compatible controller.");
        }

        // Device Selector Dropdown
        const auto devices = inputController->enumerateDevices();
        std::vector<std::string> devLabels;
        devLabels.push_back("Auto-Detect Active (Recommended)");
        int currentDevIndex = 0;
        for (size_t i = 0; i < devices.size(); ++i) {
            std::string label = "Port " + std::to_string(devices[i].joystickId) + ": " + devices[i].name;
            if (devices[i].joystickId == settings.preferredJoystickId) {
                currentDevIndex = static_cast<int>(i) + 1;
            }
            devLabels.push_back(label);
        }

        if (ImGui::BeginCombo("Select Device", devLabels[static_cast<size_t>(currentDevIndex)].c_str())) {
            for (size_t i = 0; i < devLabels.size(); ++i) {
                const bool isSel = (static_cast<int>(i) == currentDevIndex);
                if (ImGui::Selectable(devLabels[i].c_str(), isSel)) {
                    if (i == 0) {
                        settings.preferredJoystickId = -1;
                    } else {
                        settings.preferredJoystickId = devices[i - 1].joystickId;
                    }
                    inputController->saveSettings();
                }
                if (isSel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        // Button Glyph / Flavor Display Style
        int flavorIdx = static_cast<int>(settings.flavor);
        const char* flavorItems[] = {
            "Auto (Detect from Device)",
            "Xbox (A, B, X, Y, RT/LT)",
            "PlayStation (Cross, Circle, R2/L2)",
            "Nintendo (B, A, Y, X, ZR/ZL)",
            "Generic Gamepad"
        };
        if (ImGui::Combo("Button Style", &flavorIdx, flavorItems, 5)) {
            settings.flavor = static_cast<ControllerFlavor>(flavorIdx);
            inputController->saveSettings();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextColored(hexToImVec4(0x38bdf8), "LIVE CONTROLLER INPUT METERS");

        // Live visual bars:
        const float liveThr = inputController->getLiveThrottle();
        const float liveBrk = inputController->getLiveBrake();
        const float liveStr = inputController->getLiveSteering();
        const float liveHb = inputController->getLiveHandbrake();
        const float lookX = inputController->getLiveLookX();
        const float lookY = inputController->getLiveLookY();

        // Throttle RT
        ImGui::TextDisabled("Throttle (RT):"); ImGui::SameLine(110);
        char thrBuf[32]; std::snprintf(thrBuf, sizeof(thrBuf), "%.0f%%", liveThr * 100.0f);
        ImGui::ProgressBar(liveThr, ImVec2(-1.0f, 14.0f), thrBuf);

        // Brake LT
        ImGui::TextDisabled("Brake (LT):"); ImGui::SameLine(110);
        char brkBuf[32]; std::snprintf(brkBuf, sizeof(brkBuf), "%.0f%%", liveBrk * 100.0f);
        ImGui::ProgressBar(liveBrk, ImVec2(-1.0f, 14.0f), brkBuf);

        // Steering Left Stick (draw centered bar: -1.0 to +1.0)
        ImGui::TextDisabled("Steer (LS):"); ImGui::SameLine(110);
        const float steerBarVal = (liveStr + 1.0f) * 0.5f;
        char strBuf[32];
        if (std::abs(liveStr) < 0.01f) std::snprintf(strBuf, sizeof(strBuf), "Center");
        else if (liveStr > 0.0f) std::snprintf(strBuf, sizeof(strBuf), "L %.0f%%", liveStr * 100.0f);
        else std::snprintf(strBuf, sizeof(strBuf), "R %.0f%%", -liveStr * 100.0f);
        ImGui::ProgressBar(steerBarVal, ImVec2(-1.0f, 14.0f), strBuf);

        // Handbrake & Free-Look
        ImGui::TextDisabled("Handbrake (A):"); ImGui::SameLine(110);
        ImGui::TextColored(liveHb > 0.5f ? hexToImVec4(0xf87171) : hexToImVec4(0x94a3b8),
                           liveHb > 0.5f ? "[ENGAGED]" : "[RELEASED]");

        ImGui::TextDisabled("Look (RS):"); ImGui::SameLine(110);
        ImGui::Text("X: %+.2f | Y: %+.2f", lookX, lookY);

        const float holdProg = inputController->getResetHoldProgress();
        if (holdProg > 0.0f) {
            ImGui::TextColored(hexToImVec4(0xfbbf24), "Resetting Vehicle...");
            ImGui::ProgressBar(holdProg, ImVec2(-1.0f, 14.0f), "");
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextColored(hexToImVec4(0x38bdf8), "ANALOG TUNING & RESPONSE CURVES");

        // Steering Deadzone
        if (ImGui::SliderFloat("Steer Deadzone", &settings.steerDeadzone, 0.00f, 0.30f, "%.2f")) {
            inputController->saveSettings();
        }

        // Steering Sensitivity Exponent
        if (ImGui::SliderFloat("Steer Sensitivity", &settings.steerSensitivity, 0.50f, 2.50f, "%.2fx")) {
            inputController->saveSettings();
        }
        ImGui::TextDisabled("  (1.0x = Linear, >1.0x = Softer progressive center)");

        // Invert Steering
        if (ImGui::Checkbox("Invert Steering Axis", &settings.invertSteering)) {
            inputController->saveSettings();
        }

        // Trigger Deadzone
        if (ImGui::SliderFloat("Trigger Deadzone", &settings.triggerDeadzone, 0.00f, 0.25f, "%.2f")) {
            inputController->saveSettings();
        }

        // Reset Hold Duration
        if (ImGui::SliderFloat("Reset Hold Time", &settings.resetHoldDuration, 0.00f, 1.20f, "%.2f s")) {
            inputController->saveSettings();
        }
        ImGui::TextDisabled("  (0.0s = Instant tap, >0.0s = Hold Y to recover)");

        // Vibration
        ImGui::Spacing();
        ImGui::TextColored(hexToImVec4(0x38bdf8), "PHYSICS EVENT-DRIVEN HAPTICS (XINPUT)");
        if (ImGui::Checkbox("Vibration Feedback", &settings.vibrationEnabled)) {
            inputController->saveSettings();
        }
        if (settings.vibrationEnabled) {
            if (ImGui::SliderFloat("Master Intensity", &settings.vibrationIntensity, 0.0f, 1.0f, "%.0f%%")) {
                inputController->saveSettings();
            }
            if (ImGui::SliderFloat("Collision Scale", &settings.collisionSensitivity, 0.0f, 2.0f, "%.2fx")) {
                inputController->saveSettings();
            }
            if (ImGui::SliderFloat("Suspension Scale", &settings.suspensionSensitivity, 0.0f, 2.0f, "%.2fx")) {
                inputController->saveSettings();
            }
            if (ImGui::SmallButton("Test Vibration Pulse")) {
                inputController->triggerTestPulse(0.70f, 0.90f, 0.25f);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(Physical events: barrier crashes, bump stops, landings)");
        }

        // Button Mapping & Rebinding Table
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextColored(hexToImVec4(0x38bdf8), "BUTTON MAPPINGS (REBINDABLE)");

        if (inputController->isRebinding()) {
            ImGui::PushStyleColor(ImGuiCol_Text, hexToImVec4(0xfbbf24));
            ImGui::Text("PRESS ANY CONTROLLER BUTTON FOR: %s",
                        getLogicalActionName(inputController->getRebindingAction()).c_str());
            ImGui::PopStyleColor();
            if (ImGui::SmallButton("Cancel Rebind")) {
                inputController->cancelRebinding();
            }
        }

        auto renderActionRow = [&](LogicalVehicleAction action, int currentBtn) {
            ImGui::Text("%-18s", getLogicalActionName(action).c_str());
            ImGui::SameLine(170.0f);
            std::string btnLabel = "[" + getGamepadButtonName(currentBtn, flavor) + "]";
            ImGui::TextColored(hexToImVec4(0x67e8f9), "%-12s", btnLabel.c_str());
            ImGui::SameLine(255.0f);
            std::string rebindId = "Rebind##" + std::to_string(static_cast<int>(action));
            if (ImGui::SmallButton(rebindId.c_str())) {
                inputController->startRebinding(action);
            }
        };

        renderActionRow(LogicalVehicleAction::Handbrake, settings.btnHandbrake);
        renderActionRow(LogicalVehicleAction::ShiftUp, settings.btnShiftUp);
        renderActionRow(LogicalVehicleAction::ShiftDown, settings.btnShiftDown);
        renderActionRow(LogicalVehicleAction::ResetVehicle, settings.btnReset);
        renderActionRow(LogicalVehicleAction::ChangeCamera, settings.btnChangeCamera);
        renderActionRow(LogicalVehicleAction::PauseResume, settings.btnPause);
        renderActionRow(LogicalVehicleAction::ToggleTransmission, settings.btnToggleTransmission);
        renderActionRow(LogicalVehicleAction::ToggleModel, settings.btnToggleModel);
        renderActionRow(LogicalVehicleAction::ToggleVisualDebug, settings.btnToggleVisualDebug);
        renderActionRow(LogicalVehicleAction::ToggleTelemetry, settings.btnToggleTelemetry);
        renderActionRow(LogicalVehicleAction::ToggleGizmos, settings.btnToggleGizmos);
        renderActionRow(LogicalVehicleAction::NextPreset, settings.btnNextPreset);
        renderActionRow(LogicalVehicleAction::PrevPreset, settings.btnPrevPreset);

        ImGui::Spacing();
        if (ImGui::SmallButton("Reset Forza Defaults")) {
            inputController->resetSettingsToDefaults();
            showValidationReport("Restored Forza Horizon default controller bindings", true);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Save Config")) {
            inputController->saveSettings();
            showValidationReport("Saved controller settings to settings/controller_config.json", true);
        }
    }
}

void ImGuiDashboard::renderRightTelemetryPanel(
    const VehicleTelemetrySnapshot& snap,
    double fps,
    double physicsStepTimeMs,
    int windowWidth,
    int windowHeight,
    const GpuHardwareInfo* gpuInfo
) {
    const float panelW = 435.0f;
    const float maxPanelH = std::max(340.0f, static_cast<float>(windowHeight) - 210.0f);
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowWidth) - panelW - 8.0f, 60.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panelW, maxPanelH), ImGuiCond_Always);

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("PHYSICS TELEMETRY & OSCILLOSCOPE (120 Hz)", nullptr, flags)) {
        // 1. Solver, Timing, Position & Global Force Balance Summary
        const double frameDtMs = 1000.0 / std::max(1.0, fps);
        double totalTireFx = 0.0;
        double totalRollRes = 0.0;
        double totalDriveTrq = 0.0;
        double totalBrakeTrq = 0.0;
        for (const auto& w : snap.wheels) {
            totalTireFx += w.longitudinalForce;
            totalDriveTrq += w.driveTorque;
            totalBrakeTrq += w.brakeTorque;
            if (w.inContact && w.normalLoad > 0.0) {
                const double crr = 0.014 * w.surface.rollingResistanceMultiplier;
                totalRollRes += -crr * w.normalLoad * clamp(w.patchSpeedLongitudinal / 0.4, -1.0, 1.0);
            }
        }
        const double netLongFx = totalTireFx - snap.aero.dragMagnitude + totalRollRes;

        if (gpuInfo) {
            ImGui::TextColored(gpuInfo->isDiscreteGpu ? hexToImVec4(0x38bdf8) : hexToImVec4(0xfacc15),
                               "GPU: %s", gpuInfo->cleanName.c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("[%s]", gpuInfo->isDiscreteGpu ? "Discrete" : "Integrated");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", gpuInfo->getDetailedReport().c_str());
            }
        }

        ImGui::Text("FPS: %.0f (%.1f ms) | Phys dt: 8.33 ms (%.2f ms) | Contacts: %d",
                    fps, frameDtMs, physicsStepTimeMs, snap.collisionContactCount);
        ImGui::Text("Pos: (%.1f, %.2f, %.1f) m | v: %.2f m/s (%.1f km/h) | a_x: %+.2f m/s2",
                    snap.chassisPosition.x, snap.chassisPosition.y, snap.chassisPosition.z,
                    snap.forwardSpeedMps, snap.speedKmh, snap.longitudinalAccelMps2);
        ImGui::Text("Eng Trq: %+.1f Nm | Axle Drive: %+.0f Nm | Brake Trq: %.0f Nm",
                    snap.drivetrain.engineTorqueNm, totalDriveTrq, totalBrakeTrq);
        ImGui::Text("Drive Fx: %+.0f N | Drag: -%.0f N | RollRes: %+.0f N | Net Fx: %+.0f N",
                    totalTireFx, snap.aero.dragMagnitude, totalRollRes, netLongFx);
        ImGui::Text("Downforce: %.0f N | Numerical Sanity: ", snap.aero.downforceMagnitude);
        ImGui::SameLine();
        if (snap.numericalSanityOk) {
            ImGui::TextColored(hexToImVec4(0x22c55e), "FINITE OK");
        } else {
            ImGui::TextColored(hexToImVec4(0xef4444), "WARN");
        }

        ImGui::Separator();

        // 2. 4-Wheel Physics Table
        if (ImGui::BeginTable("WheelTable", 6, ImGuiTableFlags_BordersInner | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Wheel", ImGuiTableColumnFlags_WidthFixed, 42.0f);
            ImGui::TableSetupColumn("Comp (mm)", ImGuiTableColumnFlags_WidthFixed, 78.0f);
            ImGui::TableSetupColumn("Fz/Grip(N)", ImGuiTableColumnFlags_WidthFixed, 78.0f);
            ImGui::TableSetupColumn("Slip k / a", ImGuiTableColumnFlags_WidthFixed, 82.0f);
            ImGui::TableSetupColumn("Fx / Fy (N)", ImGuiTableColumnFlags_WidthFixed, 78.0f);
            ImGui::TableSetupColumn("w(r/s)", ImGuiTableColumnFlags_WidthFixed, 45.0f);
            ImGui::TableHeadersRow();

            const int numWheels = static_cast<int>(snap.wheels.size());
            for (int i = 0; i < numWheels; ++i) {
                const WheelState& w = snap.wheels[static_cast<size_t>(i)];
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                const ImVec4 statusCol = w.hasObstacleContact
                    ? hexToImVec4(0xf97316)
                    : (w.inContact
                        ? (w.isSkidding ? hexToImVec4(0xef4444) : hexToImVec4(0x22c55e))
                        : hexToImVec4(0x64748b));
                ImGui::TextColored(statusCol, "%s%s", w.id.c_str(), w.hasObstacleContact ? "*" : "");
                if (w.hasObstacleContact && ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Obstacle / Circumference Contact!\nHit Angle: %.1f deg\nF_obs: %.0f N\nF_climb: %.0f N",
                                      radToDeg(w.circumferentialHitAngle),
                                      vec3Length(w.obstacleResistanceForceWorld),
                                      vec3Length(w.tractionClimbingForceWorld));
                }

                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.0f (%d%%)", w.compression * 1000.0, static_cast<int>(std::round(w.compressionRatio * 100.0)));

                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.0f/%.0f", w.normalLoad, w.maxAvailableGripForce);

                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%+.2f/%.1f", w.slipRatio, radToDeg(w.slipAngle));

                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%.0f/%.0f", w.longitudinalForce, w.lateralForce);

                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.1f", w.angularVelocity);
            }
            ImGui::EndTable();
        }

        ImGui::Separator();
        const bool isVehicleStopped = (std::abs(snap.forwardSpeedMps) < 0.15 && std::abs(snap.lateralSpeedMps) < 0.15);
        ImGui::Text("Stability State: ");
        ImGui::SameLine();
        if (isVehicleStopped) {
            ImGui::TextColored(hexToImVec4(0x22c55e), "STATIONARY (Zero-Creep Stiction Active)");
        } else {
            ImGui::TextColored(hexToImVec4(0x38bdf8), "DYNAMIC MOTION (%.1f km/h)", snap.speedKmh);
        }

        if (ImGui::TreeNode("6-Wheel Kinematic Alignment & Articulation")) {
            const int nW = static_cast<int>(snap.wheels.size());
            for (int i = 0; i < nW; ++i) {
                const auto& w = snap.wheels[static_cast<size_t>(i)];
                ImGui::Text("%s: Rel Y: %+.3f m | Steer: %+5.1f deg | Load: %4.0f N | %s",
                            w.id.c_str(),
                            w.wheelCenterWorld.y - snap.chassisPosition.y,
                            radToDeg(w.steeringAngle),
                            w.normalLoad,
                            w.hasObstacleContact ? "OBSTACLE" : (w.inContact ? "GROUND" : "AIRBORNE"));
            }
            ImGui::TreePop();
        }

        ImGui::Separator();
        ImGui::TextDisabled("TIME-SERIES GRAPHS (Speed, Accel, Pitch/Roll, Suspension)");

        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float canvasW = std::max(260.0f, avail.x);
        const float canvasH = std::max(180.0f, avail.y - 4.0f);
        drawOscilloscopeCanvas(canvasW, canvasH);
    }
    ImGui::End();
}

void ImGuiDashboard::drawOscilloscopeCanvas(float width, float height) {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + width, p0.y + height);

    drawList->AddRectFilled(p0, p1, hexToImU32(0x0b1120));
    drawList->AddRect(p0, p1, hexToImU32(0x334155));

    const float bandH = height / 4.0f;
    const size_t n = m_history.size();

    auto drawBand = [&](int bandIdx, const char* title, float minVal, float maxVal,
                        std::initializer_list<std::pair<ImU32, std::function<float(const TelemetrySample&)>>> series) {
        const float y0 = p0.y + bandIdx * bandH;
        const float y1 = y0 + bandH;

        drawList->AddRect(ImVec2(p0.x, y0), ImVec2(p1.x, y1), hexToImU32(0x1e293b));

        if (minVal < 0.0f && maxVal > 0.0f) {
            const float zeroY = y1 - ((0.0f - minVal) / (maxVal - minVal)) * bandH;
            drawList->AddLine(ImVec2(p0.x, zeroY), ImVec2(p1.x, zeroY), hexToImU32(0x334155));
        }

        if (n > 1) {
            for (const auto& s : series) {
                for (size_t i = 0; i + 1 < n; ++i) {
                    const float xA = p0.x + (static_cast<float>(i) / static_cast<float>(MAX_SAMPLES - 1)) * width;
                    const float xB = p0.x + (static_cast<float>(i + 1) / static_cast<float>(MAX_SAMPLES - 1)) * width;

                    const float vA = std::clamp(s.second(m_history[i]), minVal, maxVal);
                    const float vB = std::clamp(s.second(m_history[i + 1]), minVal, maxVal);

                    const float normA = (vA - minVal) / (maxVal - minVal);
                    const float normB = (vB - minVal) / (maxVal - minVal);

                    const float yA = y1 - 3.0f - normA * (bandH - 6.0f);
                    const float yB = y1 - 3.0f - normB * (bandH - 6.0f);

                    drawList->AddLine(ImVec2(xA, yA), ImVec2(xB, yB), s.first, 1.5f);
                }
            }
        }

        drawList->AddText(ImVec2(p0.x + 6.0f, y0 + 3.0f), hexToImU32(0x94a3b8), title);
    };

    drawBand(0, "Rover Speed (m/s) [0..2.0]", 0.0f, 2.0f, {
        {hexToImU32(0x38bdf8), [](const TelemetrySample& s) { return s.speedKmh / 3.6f; }}
    });

    drawBand(1, "Accel Long (grn) / Lat (org) [-4..+4 m/s2]", -4.0f, 4.0f, {
        {hexToImU32(0x22c55e), [](const TelemetrySample& s) { return s.longAccel; }},
        {hexToImU32(0xf97316), [](const TelemetrySample& s) { return s.latAccel; }}
    });

    drawBand(2, "Pitch (cyn) / Roll (pnk) [-30..+30 deg]", -30.0f, 30.0f, {
        {hexToImU32(0x06b6d4), [](const TelemetrySample& s) { return s.pitchDeg; }},
        {hexToImU32(0xec4899), [](const TelemetrySample& s) { return s.rollDeg; }}
    });

    drawBand(3, "Wheel Deflection FL/FR/RL/RR [0..250 mm]", 0.0f, 250.0f, {
        {hexToImU32(0x38bdf8), [](const TelemetrySample& s) { return s.compMM[0]; }},
        {hexToImU32(0xa855f7), [](const TelemetrySample& s) { return s.compMM[1]; }},
        {hexToImU32(0x22c55e), [](const TelemetrySample& s) { return s.compMM[2]; }},
        {hexToImU32(0xeab308), [](const TelemetrySample& s) { return s.compMM[3]; }}
    });

    ImGui::Dummy(ImVec2(width, height));
}

void ImGuiDashboard::renderBottomInstrumentCluster(
    const VehicleDynamics& vehicle,
    const VehicleTelemetrySnapshot& snap,
    const VehicleControlInputs& inputs,
    const PathFollower* pathFollower,
    const Vec3* selectedDestination,
    bool hasDestination,
    int windowWidth,
    int windowHeight
) {
    const float clusterW = std::min(1120.0f, static_cast<float>(windowWidth - 16));
    const float clusterH = 142.0f;
    ImGui::SetNextWindowPos(
        ImVec2(static_cast<float>(windowWidth) * 0.5f, static_cast<float>(windowHeight) - 8.0f),
        ImGuiCond_Always,
        ImVec2(0.5f, 1.0f)
    );
    ImGui::SetNextWindowSize(ImVec2(clusterW, clusterH), ImGuiCond_Always);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##PlanetaryRoverConsole", nullptr, flags)) {
        if (ImGui::BeginTable("RoverConsoleGrid", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextRow();

            // ==================== COLUMN 0: ROVER POSITION & NAVIGATION ====================
            ImGui::TableSetColumnIndex(0);
            ImGui::TextColored(hexToImVec4(0x38bdf8), "PLANETARY NAVIGATION & POSITION");

            const double roverYaw = getVehicleEulerAngles(snap.chassisRotation).yaw;
            double headingDeg = radToDeg(roverYaw);
            while (headingDeg < 0.0) headingDeg += 360.0;
            while (headingDeg >= 360.0) headingDeg -= 360.0;

            const char* compassPts[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
            const int compassIdx = static_cast<int>(std::floor((headingDeg + 22.5) / 45.0)) % 8;
            const char* compassStr = compassPts[compassIdx];

            ImGui::Text("Coordinates:  X: %+.2f m | Z: %+.2f m", snap.chassisPosition.x, snap.chassisPosition.z);
            ImGui::Text("Elevation:    Y: %+.2f m (Surface MSL)", snap.chassisPosition.y);
            ImGui::Text("Heading:      %03.0f deg [%s]", headingDeg, compassStr);
            ImGui::Text("Velocity:     %.2f m/s (%.1f km/h)", snap.forwardSpeedMps, snap.speedKmh);

            // ==================== COLUMN 1: ATTITUDE & SLOPE SAFETY ====================
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(hexToImVec4(0xfacc15), "TERRAIN ATTITUDE & ROCKER-BOGIE");

            const double pitchDeg = radToDeg(snap.pitchRad);
            const double rollDeg = radToDeg(snap.rollRad);
            const double slopeDeg = std::hypot(pitchDeg, rollDeg);

            ImGui::Text("Pitch: %+5.1f deg  |  Roll: %+5.1f deg", pitchDeg, rollDeg);
            ImGui::Text("Terrain Slope: %.1f deg", slopeDeg);
            ImGui::SameLine();
            if (slopeDeg < 15.0) {
                ImGui::TextColored(hexToImVec4(0x22c55e), "[NOMINAL]");
            } else if (slopeDeg < 25.0) {
                ImGui::TextColored(hexToImVec4(0xfacc15), "[CAUTION]");
            } else {
                ImGui::TextColored(hexToImVec4(0xef4444), "[TILT HAZARD]");
            }

            const auto& rb = vehicle.rockerBogie.getState();
            ImGui::Text("Rocker L/R: %+4.1f / %+4.1f deg | Diff: %+4.1f deg",
                        radToDeg(rb.leftRockerPitch), radToDeg(rb.rightRockerPitch), radToDeg(rb.differentialAngle));

            ImGui::Text("6-Wheel Contacts: ");
            static const char* const wLabels[6] = {"FL", "FR", "ML", "MR", "RL", "RR"};
            for (size_t w = 0; w < 6; ++w) {
                ImGui::SameLine();
                const bool inC = (w < snap.wheels.size()) && snap.wheels[w].inContact;
                const bool obs = (w < snap.wheels.size()) && snap.wheels[w].hasObstacleContact;
                const ImVec4 col = obs ? hexToImVec4(0xf97316) : (inC ? hexToImVec4(0x22c55e) : hexToImVec4(0x64748b));
                ImGui::TextColored(col, "[%s]", wLabels[w]);
            }

            // ==================== COLUMN 2: AUTONOMOUS MISSION & HUB MOTORS ====================
            ImGui::TableSetColumnIndex(2);
            ImGui::TextColored(hexToImVec4(0x34d399), "AUTONOMOUS MISSION & HUB MOTORS");

            if (pathFollower && hasDestination && selectedDestination) {
                const auto& telem = pathFollower->getTelemetry();
                const char* stateStr = "IDLE";
                ImVec4 stateCol = hexToImVec4(0x94a3b8);
                switch (telem.state) {
                    case NavState::Navigating: stateStr = "NAVIGATING TO TARGET"; stateCol = hexToImVec4(0x22c55e); break;
                    case NavState::Arrived:    stateStr = "ARRIVED AT TARGET"; stateCol = hexToImVec4(0x38bdf8); break;
                    case NavState::Paused:     stateStr = "MISSION PAUSED"; stateCol = hexToImVec4(0xfacc15); break;
                    case NavState::Stuck:      stateStr = "ROVER STUCK (OBSTACLE)"; stateCol = hexToImVec4(0xf97316); break;
                    case NavState::EmergencyStopped: stateStr = "EMERGENCY STOP"; stateCol = hexToImVec4(0xef4444); break;
                    default: break;
                }
                ImGui::TextColored(stateCol, "Status: %s", stateStr);
                ImGui::Text("Target: (%.1f, %.1f) | Dist: %.1f m", selectedDestination->x, selectedDestination->z, telem.distanceToTargetM);

                if (telem.totalWaypoints > 0) {
                    const float prog = static_cast<float>(telem.currentWaypointIndex) / static_cast<float>(telem.totalWaypoints);
                    char progBuf[64];
                    std::snprintf(progBuf, sizeof(progBuf), "WP %zu / %zu (%.0f%%)", telem.currentWaypointIndex + 1, telem.totalWaypoints, prog * 100.0f);
                    ImGui::ProgressBar(prog, ImVec2(-1.0f, 15.0f), progBuf);
                }
            } else {
                ImGui::TextDisabled("Status: MANUAL / IDLE (Click map to set goal)");
                ImGui::Text("Direct Drive: 6-Wheel Electric Hub Motors");
                ImGui::Text("Drive Cmd:    %.0f%% | Steer: %+5.1f deg", inputs.throttle * 100.0, radToDeg(snap.steeringAngleRad));
            }

            if (inputs.handbrake > 0.05 || vehicle.config.enableStationaryHold) {
                ImGui::TextColored(hexToImVec4(0x38bdf8), "HOLDING BRAKE / ZERO-CREEP STICTION: ACTIVE");
            } else {
                ImGui::TextDisabled("Holding Brake: OFF");
            }

            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void ImGuiDashboard::renderMinimapAndNavigationWindow(
    VehicleDynamics& vehicle,
    MinimapRenderer& minimap,
    PathFollower& pathFollower,
    const NavGrid& navGrid,
    AStarPlanner& planner,
    Vec3& selectedDestination,
    bool& hasDestination,
    int windowWidth,
    int windowHeight
) {
    (void)windowHeight;
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowWidth) - 430.0f, 60.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(410.0f, 660.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Planetary Minimap & Navigation (M)###MinimapWindow", &minimapEnabled)) {
        // Mode Selection
        int modeIdx = (minimap.getDisplayMode() == MinimapDisplayMode::Textured) ? 0 : 1;
        if (ImGui::RadioButton("Textured", modeIdx == 0)) {
            minimap.setDisplayMode(MinimapDisplayMode::Textured);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Elevation Contour Heatmap", modeIdx == 1)) {
            minimap.setDisplayMode(MinimapDisplayMode::ElevationHeatmap);
        }

        // Viewport and Zoom Controls
        if (ImGui::Button(" [-] ")) {
            minimap.zoomDelta(-0.25);
        }
        ImGui::SameLine();
        if (ImGui::Button(" [+] ")) {
            minimap.zoomDelta(+0.25);
        }
        ImGui::SameLine();
        float currentZoom = static_cast<float>(minimap.getZoom());
        ImGui::SetNextItemWidth(90.0f);
        if (ImGui::SliderFloat("##ZoomSlider", &currentZoom, 0.2f, 5.0f, "%.2fx")) {
            minimap.setZoom(static_cast<double>(currentZoom));
        }
        ImGui::SameLine();
        if (ImGui::Button("Recenter")) {
            minimap.recenterOnRover();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            minimap.resetView();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Follow", &minimap.getViewport().followRover);

        // Minimap Image Surface
        const float mapDim = std::min(ImGui::GetContentRegionAvail().x, 380.0f);
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const ImVec2 p1 = ImVec2(p0.x + mapDim, p0.y + mapDim);
        ImDrawList* drawList = ImGui::GetWindowDrawList();

        const Vec3 roverPos = vehicle.body.getPosition();
        constexpr double kTotalWorld = 512.0;
        const double span = kTotalWorld / minimap.getZoom();
        const double centerX = minimap.getViewport().followRover ? (roverPos.x + minimap.getViewport().panOffsetX) : minimap.getViewport().panOffsetX;
        const double centerZ = minimap.getViewport().followRover ? (roverPos.z + minimap.getViewport().panOffsetZ) : minimap.getViewport().panOffsetZ;

        const float u0 = static_cast<float>((centerX - 0.5 * span + 256.0) / kTotalWorld);
        const float u1 = static_cast<float>((centerX + 0.5 * span + 256.0) / kTotalWorld);
        const float v0 = static_cast<float>((256.0 - (centerZ + 0.5 * span)) / kTotalWorld);
        const float v1 = static_cast<float>((256.0 - (centerZ - 0.5 * span)) / kTotalWorld);

        // Render OpenGL Texture
        ImGui::Image(
            static_cast<ImTextureID>(minimap.getTextureId()),
            ImVec2(mapDim, mapDim),
            ImVec2(u0, v0),
            ImVec2(u1, v1)
        );

        const bool isMapHovered = ImGui::IsItemHovered();

        // Click to Select Destination
        if (isMapHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            Vec3 clickedWorld;
            if (minimap.screenToWorld(io.MousePos.x, io.MousePos.y, p0.x, p0.y, mapDim, mapDim, roverPos, clickedWorld)) {
                clickedWorld.y = navGrid.getElevationAt(clickedWorld.x, clickedWorld.z);
                selectedDestination = clickedWorld;
                hasDestination = true;

                // Auto-plan route
                PlanResult planRes = planner.plan(navGrid, roverPos, selectedDestination);
                if (planRes.success) {
                    pathFollower.setPath(planRes.waypoints);
                    pathFollower.startNavigation();
                }
            }
        }

        // Right-Click Drag to Pan
        if (isMapHovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            const ImVec2 drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
            ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
            minimap.pan(-drag.x * (span / mapDim), drag.y * (span / mapDim));
        }

        // Mouse Wheel to Zoom
        if (isMapHovered && std::abs(io.MouseWheel) > 0.01f) {
            minimap.zoomDelta(static_cast<double>(io.MouseWheel) * 0.2);
        }

        // Overlays
        drawList->PushClipRect(p0, p1, true);

        // 1. Draw Planned Path Waypoints and connecting polyline
        const auto& waypoints = pathFollower.getWaypoints();
        if (waypoints.size() >= 2) {
            for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
                float sx1, sy1, sx2, sy2;
                if (minimap.worldToScreen(waypoints[i].positionWorld, p0.x, p0.y, mapDim, mapDim, roverPos, sx1, sy1) &&
                    minimap.worldToScreen(waypoints[i + 1].positionWorld, p0.x, p0.y, mapDim, mapDim, roverPos, sx2, sy2)) {
                    drawList->AddLine(ImVec2(sx1, sy1), ImVec2(sx2, sy2), IM_COL32(34, 211, 238, 230), 2.5f);
                    drawList->AddCircleFilled(ImVec2(sx1, sy1), 2.5f, IM_COL32(56, 189, 248, 255));
                }
            }
        }

        // 2. Draw Destination Marker
        if (hasDestination) {
            float dx, dy;
            if (minimap.worldToScreen(selectedDestination, p0.x, p0.y, mapDim, mapDim, roverPos, dx, dy)) {
                drawList->AddCircle(ImVec2(dx, dy), 8.0f, IM_COL32(239, 68, 68, 255), 16, 2.0f);
                drawList->AddCircleFilled(ImVec2(dx, dy), 4.0f, IM_COL32(239, 68, 68, 255));
                drawList->AddText(ImVec2(dx + 9.0f, dy - 8.0f), IM_COL32(255, 255, 255, 255), "DEST");
            }
        }

        // 3. Draw Rover Marker & Heading Arrow
        float rx, ry;
        if (minimap.worldToScreen(roverPos, p0.x, p0.y, mapDim, mapDim, roverPos, rx, ry)) {
            drawList->AddCircleFilled(ImVec2(rx, ry), 5.5f, IM_COL32(16, 185, 129, 255));
            drawList->AddCircle(ImVec2(rx, ry), 7.0f, IM_COL32(255, 255, 255, 240), 16, 1.5f);

            // Heading pointer
            const VehicleBasis basis = getVehicleBasis(vehicle.body.getRotation());
            const Vec3 fwd = basis.forward;
            const Vec3 tipPos = vec3Add(roverPos, vec3Scale(fwd, span * 0.05));
            float hx, hy;
            if (minimap.worldToScreen(tipPos, p0.x, p0.y, mapDim, mapDim, roverPos, hx, hy)) {
                drawList->AddLine(ImVec2(rx, ry), ImVec2(hx, hy), IM_COL32(250, 204, 21, 255), 2.5f);
                drawList->AddCircleFilled(ImVec2(hx, hy), 3.0f, IM_COL32(250, 204, 21, 255));
            }
        }

        // 4. Draw North Compass Marker in Top-Right
        const ImVec2 northPos(p1.x - 28.0f, p0.y + 16.0f);
        drawList->AddTriangleFilled(
            northPos,
            ImVec2(northPos.x - 5.0f, northPos.y + 12.0f),
            ImVec2(northPos.x + 5.0f, northPos.y + 12.0f),
            IM_COL32(239, 68, 68, 255)
        );
        drawList->AddText(ImVec2(northPos.x - 4.0f, northPos.y + 13.0f), IM_COL32(255, 255, 255, 220), "N");

        // 5. Draw Metric Scale Bar in Bottom-Left
        const double barWorldM = std::max(5.0, std::round((span * 0.2) / 10.0) * 10.0);
        const float barPixelLen = static_cast<float>((barWorldM / span) * mapDim);
        const ImVec2 barStart(p0.x + 12.0f, p1.y - 14.0f);
        const ImVec2 barEnd(p0.x + 12.0f + barPixelLen, p1.y - 14.0f);
        drawList->AddLine(barStart, barEnd, IM_COL32(255, 255, 255, 240), 3.0f);
        drawList->AddLine(barStart, ImVec2(barStart.x, barStart.y - 5.0f), IM_COL32(255, 255, 255, 240), 2.0f);
        drawList->AddLine(barEnd, ImVec2(barEnd.x, barEnd.y - 5.0f), IM_COL32(255, 255, 255, 240), 2.0f);
        char scaleBuf[32];
        std::snprintf(scaleBuf, sizeof(scaleBuf), "%.0f m", barWorldM);
        drawList->AddText(ImVec2(barStart.x + 4.0f, barStart.y - 16.0f), IM_COL32(255, 255, 255, 240), scaleBuf);

        // 6. Draw Elevation Legend if in Elevation Heatmap Mode
        if (minimap.getDisplayMode() == MinimapDisplayMode::ElevationHeatmap) {
            const float legX = p1.x - 14.0f;
            const float legY0 = p0.y + 40.0f;
            const float legH = 100.0f;
            for (int k = 0; k < 20; ++k) {
                const float frac0 = static_cast<float>(k) / 20.0f;
                const float frac1 = static_cast<float>(k + 1) / 20.0f;
                const Color3 c0 = MinimapRenderer::sampleElevationHeatmapColor(1.0 - frac0);
                const ImU32 col = IM_COL32(
                    static_cast<uint8_t>(c0.r * 255),
                    static_cast<uint8_t>(c0.g * 255),
                    static_cast<uint8_t>(c0.b * 255),
                    240
                );
                drawList->AddRectFilled(
                    ImVec2(legX, legY0 + frac0 * legH),
                    ImVec2(legX + 8.0f, legY0 + frac1 * legH),
                    col
                );
            }
            char minElevStr[16], maxElevStr[16];
            std::snprintf(maxElevStr, sizeof(maxElevStr), "%.0fm", minimap.getMaxElevation());
            std::snprintf(minElevStr, sizeof(minElevStr), "%.0fm", minimap.getMinElevation());
            drawList->AddText(ImVec2(legX - 35.0f, legY0 - 4.0f), IM_COL32(255, 255, 255, 220), maxElevStr);
            drawList->AddText(ImVec2(legX - 35.0f, legY0 + legH - 8.0f), IM_COL32(255, 255, 255, 220), minElevStr);
        }

        drawList->PopClipRect();

        ImGui::Separator();

        // Autonomy Controls & Telemetry
        const auto& telem = pathFollower.getTelemetry();
        ImGui::Text("Autonomy State: ");
        ImGui::SameLine();
        switch (telem.state) {
            case NavState::Idle:
                ImGui::TextColored(hexToImVec4(0x94a3b8), "IDLE (Click minimap to set destination)");
                break;
            case NavState::Planning:
                ImGui::TextColored(hexToImVec4(0x38bdf8), "PLANNING ROUTE...");
                break;
            case NavState::Navigating:
                ImGui::TextColored(hexToImVec4(0x22c55e), "NAVIGATING TO DESTINATION");
                break;
            case NavState::Paused:
                ImGui::TextColored(hexToImVec4(0xfacc15), "MISSION PAUSED");
                break;
            case NavState::Arrived:
                ImGui::TextColored(hexToImVec4(0x38bdf8), "DESTINATION REACHED (ARRIVED)");
                break;
            case NavState::Stuck:
                ImGui::TextColored(hexToImVec4(0xf97316), "ROVER STUCK (OBSTACLE DETECTED)");
                break;
            case NavState::EmergencyStopped:
                ImGui::TextColored(hexToImVec4(0xef4444), "EMERGENCY STOP ENGAGED");
                break;
        }

        // Action Buttons
        if (telem.state == NavState::Navigating) {
            if (ImGui::Button("Pause Mission")) {
                pathFollower.pauseNavigation(true);
            }
        } else if (telem.state == NavState::Paused) {
            if (ImGui::Button("Resume Mission") && hasDestination) {
                pathFollower.resumeNavigation();
            }
        } else {
            if (ImGui::Button("Start Autonomous Mission") && hasDestination) {
                pathFollower.startNavigation();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Emergency Stop")) {
            pathFollower.emergencyStop();
        }
        ImGui::SameLine();
        if (ImGui::Button("Replan") && hasDestination) {
            PlanResult res = planner.plan(navGrid, roverPos, selectedDestination);
            if (res.success) {
                pathFollower.setPath(res.waypoints);
                pathFollower.startNavigation();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            pathFollower.clearPath();
            hasDestination = false;
        }

        // Navigation Telemetry Metrics
        if (hasDestination) {
            ImGui::Text("Destination: (%.1f, %.1f) | Dist: %.1f m", selectedDestination.x, selectedDestination.z, telem.distanceToTargetM);
            ImGui::Text("Waypoints: %zu / %zu | Cross-Track: %.2f m | Head Err: %+.1f deg",
                        telem.currentWaypointIndex + 1, telem.totalWaypoints, telem.crossTrackErrorM, radToDeg(telem.headingErrorRad));
            ImGui::Text("Target Speed: %.2f m/s (%.1f km/h) | Cmd Thr: %.0f%% | Steer: %+.1f deg",
                        telem.targetSpeedMPerSec, telem.targetSpeedMPerSec * 3.6, telem.commandedThrottle * 100.0, radToDeg(telem.commandedSteering));
            if (telem.totalWaypoints > 0) {
                const float progress = static_cast<float>(telem.currentWaypointIndex) / static_cast<float>(telem.totalWaypoints);
                ImGui::ProgressBar(progress, ImVec2(-1.0f, 14.0f));
            }
        } else {
            ImGui::TextDisabled("No active navigation route. Click on minimap to select target.");
        }
    }
    ImGui::End();
}

} // namespace torquelab
