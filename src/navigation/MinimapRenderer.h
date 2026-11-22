#pragma once
/**
 * PROJECT MANTIS — Interactive Topographical Minimap & Route Visualization.
 *
 * Implements a 2D top-down planetary minimap supporting:
 * - Mode A: Textured planetary terrain surface (Mars iron-oxide / Moon lunar basalt).
 * - Mode B: Topographical elevation contour heatmap (perceptually uniform color scale).
 * - Interactive zooming (0.2x .. 5.0x), continuous panning, and recentering on rover.
 * - Bidirectional screen-to-world coordinate conversion for click-to-navigate destination selection.
 * - Dynamic route polyline, waypoint markers, rover orientation icon, and scale bar overlays.
 */

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "navigation/AStarPlanner.h"
#include "physics/Math3D.h"
#include "renderer/Texture2D.h"
#include "terrain/TerrainMap.h"

namespace torquelab {

enum class MinimapDisplayMode {
    Textured,         // Mode A: Planetary surface color and regolith
    ElevationHeatmap  // Mode B: Continuous topographical elevation contour gradient
};

struct MinimapViewport {
    double zoom{1.0};            // Zoom level [0.2x .. 5.0x]
    double panOffsetX{0.0};      // World-space pan offset X (m)
    double panOffsetZ{0.0};      // World-space pan offset Z (m)
    bool followRover{true};      // Automatically centers viewport on rover
};

class MinimapRenderer {
public:
    MinimapRenderer();
    ~MinimapRenderer();

    MinimapRenderer(const MinimapRenderer&) = delete;
    MinimapRenderer& operator=(const MinimapRenderer&) = delete;

    /**
     * Generates or updates the 2D minimap background texture from the active terrain map.
     */
    void updateTerrain(const ITerrainMap& terrainMap);

    /**
     * Switches display mode between Textured and Elevation Heatmap.
     */
    void setDisplayMode(MinimapDisplayMode mode);
    MinimapDisplayMode getDisplayMode() const { return m_displayMode; }

    /** Viewport controls */
    void setZoom(double zoom);
    double getZoom() const { return m_viewport.zoom; }
    void zoomDelta(double delta);
    void pan(double deltaWorldX, double deltaWorldZ);
    void recenterOnRover();
    void resetView();

    MinimapViewport& getViewport() { return m_viewport; }
    const MinimapViewport& getViewport() const { return m_viewport; }

    /**
     * Converts clicked screen coordinates within the minimap window to authoritative world-space coordinates.
     */
    bool screenToWorld(
        float screenX, float screenY,
        float mapWindowX, float mapWindowY,
        float mapWindowWidth, float mapWindowHeight,
        const Vec3& roverWorldPos,
        Vec3& outWorldPos
    ) const;

    /**
     * Converts world-space coordinates to screen pixel coordinates within the minimap window.
     */
    bool worldToScreen(
        const Vec3& worldPos,
        float mapWindowX, float mapWindowY,
        float mapWindowWidth, float mapWindowHeight,
        const Vec3& roverWorldPos,
        float& outScreenX, float& outScreenY
    ) const;

    /** OpenGL texture handle for ImGui::Image rendering */
    unsigned int getTextureId() const { return m_glTextureId; }
    int getTextureResolution() const { return m_texResolution; }

    float getMinElevation() const { return m_minElevation; }
    float getMaxElevation() const { return m_maxElevation; }

    /**
     * Returns RGB color for elevation heatmap corresponding to normalized value [0..1]
     */
    static Color3 sampleElevationHeatmapColor(double normalizedElevation);

private:
    void rebuildTexture();
    Color3 computeTexturedColor(double wx, double wz, double elevation, const Vec3& normal, SurfaceType mat) const;

    int m_texResolution{512};
    MinimapDisplayMode m_displayMode{MinimapDisplayMode::ElevationHeatmap};
    MinimapViewport m_viewport;

    const ITerrainMap* m_cachedTerrainMap{nullptr};
    std::string m_cachedMapId;

    float m_minElevation{-25.0f};
    float m_maxElevation{25.0f};

    std::vector<uint8_t> m_pixelBuffer; // 512x512x4 RGBA
    unsigned int m_glTextureId{0};
    bool m_isDirty{true};
};

} // namespace torquelab
