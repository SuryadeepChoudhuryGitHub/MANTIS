#include "navigation/MinimapRenderer.h"
#include <algorithm>
#include <cmath>
#include <glad/glad.h>
#include "terrain/ProceduralPlanetMap.h"

namespace torquelab {

namespace {

inline Color3 color3Lerp(const Color3& a, const Color3& b, float t) {
    return Color3{a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t};
}

inline Color3 color3Scale(const Color3& c, float s) {
    return Color3{c.r * s, c.g * s, c.b * s};
}

} // namespace

MinimapRenderer::MinimapRenderer() {
    m_pixelBuffer.resize(static_cast<size_t>(m_texResolution * m_texResolution * 4), 0);
}

MinimapRenderer::~MinimapRenderer() {
    if (m_glTextureId != 0) {
        glDeleteTextures(1, &m_glTextureId);
        m_glTextureId = 0;
    }
}

void MinimapRenderer::updateTerrain(const ITerrainMap& terrainMap) {
    m_cachedTerrainMap = &terrainMap;
    m_cachedMapId = terrainMap.getId();
    m_isDirty = true;
    rebuildTexture();
}

void MinimapRenderer::setDisplayMode(MinimapDisplayMode mode) {
    if (m_displayMode != mode) {
        m_displayMode = mode;
        m_isDirty = true;
        rebuildTexture();
    }
}

void MinimapRenderer::setZoom(double zoom) {
    m_viewport.zoom = clamp(zoom, 0.2, 5.0);
}

void MinimapRenderer::zoomDelta(double delta) {
    setZoom(m_viewport.zoom * (1.0 + delta));
}

void MinimapRenderer::pan(double deltaWorldX, double deltaWorldZ) {
    m_viewport.panOffsetX += deltaWorldX;
    m_viewport.panOffsetZ += deltaWorldZ;
    m_viewport.followRover = false;
}

void MinimapRenderer::recenterOnRover() {
    m_viewport.panOffsetX = 0.0;
    m_viewport.panOffsetZ = 0.0;
    m_viewport.followRover = true;
}

void MinimapRenderer::resetView() {
    m_viewport.zoom = 1.0;
    recenterOnRover();
}

Color3 MinimapRenderer::sampleElevationHeatmapColor(double normalizedElevation) {
    const double t = clamp(normalizedElevation, 0.0, 1.0);

    // Multi-stop continuous perceptually uniform elevation color gradient:
    // [0.00 .. 0.25]: Deep Blue -> Cyan (Deep depressions, craters, impact basins)
    // [0.25 .. 0.50]: Cyan -> Green     (Low valleys, level regolith plains)
    // [0.50 .. 0.75]: Green -> Yellow   (Rolling hills, moderate elevation)
    // [0.75 .. 0.90]: Yellow -> Orange  (High terrain ridges, elevated plateaus)
    // [0.90 .. 1.00]: Orange -> Deep Red / White (Peak crater rims, mountain crests)

    if (t < 0.25) {
        const double f = t / 0.25;
        return color3Lerp(Color3{0.05f, 0.12f, 0.55f}, Color3{0.00f, 0.75f, 0.85f}, static_cast<float>(f));
    } else if (t < 0.50) {
        const double f = (t - 0.25) / 0.25;
        return color3Lerp(Color3{0.00f, 0.75f, 0.85f}, Color3{0.15f, 0.82f, 0.25f}, static_cast<float>(f));
    } else if (t < 0.75) {
        const double f = (t - 0.50) / 0.25;
        return color3Lerp(Color3{0.15f, 0.82f, 0.25f}, Color3{0.95f, 0.90f, 0.15f}, static_cast<float>(f));
    } else if (t < 0.90) {
        const double f = (t - 0.75) / 0.15;
        return color3Lerp(Color3{0.95f, 0.90f, 0.15f}, Color3{0.95f, 0.45f, 0.05f}, static_cast<float>(f));
    } else {
        const double f = (t - 0.90) / 0.10;
        return color3Lerp(Color3{0.95f, 0.45f, 0.05f}, Color3{0.98f, 0.12f, 0.10f}, static_cast<float>(f));
    }
}

Color3 MinimapRenderer::computeTexturedColor(
    double wx, double wz,
    double elevation,
    const Vec3& normal,
    SurfaceType mat
) const {
    const bool isMoon = (m_cachedMapId.find("moon") != std::string::npos);

    // Directional lighting from top-left sun
    const Vec3 sunDir = vec3Normalize(Vec3{0.6, 0.8, -0.4});
    const double nDotL = clamp(vec3Dot(normal, sunDir), 0.25, 1.0);

    if (isMoon) {
        // Lunar color palette: dark basalt maria to bright highland anorthosite
        Color3 baseCol{0.42f, 0.42f, 0.44f};
        if (mat == SurfaceType::Gravel) {
            baseCol = Color3{0.60f, 0.60f, 0.62f}; // High-albedo crater ejecta
        } else if (normal.y < 0.90) {
            baseCol = Color3{0.28f, 0.28f, 0.30f}; // Shadowed crater rim slopes
        }
        return color3Scale(baseCol, static_cast<float>(nDotL));
    } else {
        // Mars color palette: oxidized iron-rust, dusty ochre, and dark basalt rocks
        Color3 baseCol{0.72f, 0.35f, 0.20f}; // Ochre-red regolith
        if (mat == SurfaceType::Gravel) {
            baseCol = Color3{0.82f, 0.48f, 0.25f}; // Light drift sand / ejecta
        } else if (normal.y < 0.88) {
            baseCol = Color3{0.42f, 0.22f, 0.16f}; // Dark exposed basalt bedrock
        }
        return color3Scale(baseCol, static_cast<float>(nDotL));
    }
}

void MinimapRenderer::rebuildTexture() {
    if (!m_cachedTerrainMap) return;

    const int N = m_texResolution;
    constexpr double kWorldExtent = 512.0;
    const double halfExtent = kWorldExtent * 0.5;
    const double gridSpacing = kWorldExtent / static_cast<double>(N - 1);

    // 1. First Pass: Compute Elevation Range
    m_minElevation = 1e9f;
    m_maxElevation = -1e9f;

    std::vector<float> elevGrid(static_cast<size_t>(N * N));
    std::vector<Vec3> normGrid(static_cast<size_t>(N * N));
    std::vector<SurfaceType> matGrid(static_cast<size_t>(N * N));

    for (int r = 0; r < N; ++r) {
        // r = 0 corresponds to Top (+Z North) in texture space
        const double wz = halfExtent - static_cast<double>(r) * gridSpacing;
        for (int c = 0; c < N; ++c) {
            const double wx = -halfExtent + static_cast<double>(c) * gridSpacing;

            double elev = 0.0;
            Vec3 norm{0.0, 1.0, 0.0};
            m_cachedTerrainMap->getElevationAt(wx, wz, elev, norm);

            const size_t idx = static_cast<size_t>(r * N + c);
            elevGrid[idx] = static_cast<float>(elev);
            normGrid[idx] = norm;
            matGrid[idx] = m_cachedTerrainMap->getSurfaceMaterialAt(wx, wz);

            if (static_cast<float>(elev) < m_minElevation) m_minElevation = static_cast<float>(elev);
            if (static_cast<float>(elev) > m_maxElevation) m_maxElevation = static_cast<float>(elev);
        }
    }

    const double heightRange = std::max(1.0, static_cast<double>(m_maxElevation - m_minElevation));

    // 2. Second Pass: Generate RGBA Pixels
    for (int r = 0; r < N; ++r) {
        const double wz = halfExtent - static_cast<double>(r) * gridSpacing;
        for (int c = 0; c < N; ++c) {
            const double wx = -halfExtent + static_cast<double>(c) * gridSpacing;
            const size_t idx = static_cast<size_t>(r * N + c);

            const double elev = static_cast<double>(elevGrid[idx]);
            const double normElev = (elev - static_cast<double>(m_minElevation)) / heightRange;

            Color3 col;
            if (m_displayMode == MinimapDisplayMode::ElevationHeatmap) {
                col = sampleElevationHeatmapColor(normElev);
                // Subtle topographical contour lines every 2.0 meters
                const double contour = std::fmod(std::abs(elev), 2.0);
                if (contour < 0.12 || contour > 1.88) {
                    col = color3Scale(col, 0.70f); // Darker contour ring
                }
            } else {
                col = computeTexturedColor(wx, wz, elev, normGrid[idx], matGrid[idx]);
            }

            const size_t pxIdx = idx * 4;
            m_pixelBuffer[pxIdx + 0] = static_cast<uint8_t>(clamp(col.r * 255.0f, 0.0f, 255.0f));
            m_pixelBuffer[pxIdx + 1] = static_cast<uint8_t>(clamp(col.g * 255.0f, 0.0f, 255.0f));
            m_pixelBuffer[pxIdx + 2] = static_cast<uint8_t>(clamp(col.b * 255.0f, 0.0f, 255.0f));
            m_pixelBuffer[pxIdx + 3] = 255;
        }
    }

    // 3. Upload to OpenGL GPU Texture
    if (m_glTextureId == 0) {
        glGenTextures(1, &m_glTextureId);
    }

    glBindTexture(GL_TEXTURE_2D, m_glTextureId);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA8,
        N, N, 0,
        GL_RGBA, GL_UNSIGNED_BYTE,
        m_pixelBuffer.data()
    );

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    m_isDirty = false;
}

bool MinimapRenderer::screenToWorld(
    float screenX, float screenY,
    float mapWindowX, float mapWindowY,
    float mapWindowWidth, float mapWindowHeight,
    const Vec3& roverWorldPos,
    Vec3& outWorldPos
) const {
    if (mapWindowWidth <= 0.0f || mapWindowHeight <= 0.0f) return false;

    const float u = (screenX - mapWindowX) / mapWindowWidth;
    const float v = (screenY - mapWindowY) / mapWindowHeight;

    if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) return false;

    constexpr double kTotalWorld = 512.0;
    const double visibleWorldSpan = kTotalWorld / m_viewport.zoom;

    const double centerX = m_viewport.followRover ? (roverWorldPos.x + m_viewport.panOffsetX) : m_viewport.panOffsetX;
    const double centerZ = m_viewport.followRover ? (roverWorldPos.z + m_viewport.panOffsetZ) : m_viewport.panOffsetZ;

    const double wx = centerX + (static_cast<double>(u) - 0.5) * visibleWorldSpan;
    // Invert V: top of screen (v=0) is +Z (North), bottom (v=1) is -Z (South)
    const double wz = centerZ + (0.5 - static_cast<double>(v)) * visibleWorldSpan;

    double wy = 0.0;
    Vec3 normal{0.0, 1.0, 0.0};
    if (m_cachedTerrainMap) {
        m_cachedTerrainMap->getElevationAt(wx, wz, wy, normal);
    }

    outWorldPos = Vec3{wx, wy, wz};
    return true;
}

bool MinimapRenderer::worldToScreen(
    const Vec3& worldPos,
    float mapWindowX, float mapWindowY,
    float mapWindowWidth, float mapWindowHeight,
    const Vec3& roverWorldPos,
    float& outScreenX, float& outScreenY
) const {
    if (mapWindowWidth <= 0.0f || mapWindowHeight <= 0.0f) return false;

    constexpr double kTotalWorld = 512.0;
    const double visibleWorldSpan = kTotalWorld / m_viewport.zoom;

    const double centerX = m_viewport.followRover ? (roverWorldPos.x + m_viewport.panOffsetX) : m_viewport.panOffsetX;
    const double centerZ = m_viewport.followRover ? (roverWorldPos.z + m_viewport.panOffsetZ) : m_viewport.panOffsetZ;

    const double u = 0.5 + (worldPos.x - centerX) / visibleWorldSpan;
    const double v = 0.5 - (worldPos.z - centerZ) / visibleWorldSpan;

    outScreenX = mapWindowX + static_cast<float>(u) * mapWindowWidth;
    outScreenY = mapWindowY + static_cast<float>(v) * mapWindowHeight;

    return (u >= -0.1 && u <= 1.1 && v >= -0.1 && v <= 1.1);
}

} // namespace torquelab
