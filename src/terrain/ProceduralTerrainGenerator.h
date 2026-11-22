#pragma once
/**
 * PROJECT MANTIS — Procedural Planetary Terrain Generator.
 *
 * Implements deterministic multi-scale gradient noise (Fractal Brownian Motion),
 * analytical geological impact crater modelling, scattered rock obstacle generation,
 * and distinct Mars and Moon geological presets.
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "physics/Math3D.h"
#include "physics/PlanetaryEnvironment.h"
#include "physics/SurfaceSystem.h"

namespace torquelab {

// ============================================================================
// 1. Deterministic Multi-Scale Gradient Noise (Improved Perlin Noise & fBm)
// ============================================================================
class PerlinNoise {
public:
    explicit PerlinNoise(uint32_t seed = 1337);

    void reseed(uint32_t seed);

    /** Raw 2D gradient noise in range [-1.0, +1.0] */
    double sample2D(double x, double z) const;

    /** Fractal Brownian Motion: multi-octave layered noise in range [-1.0, +1.0] */
    double sampleFbm2D(
        double x,
        double z,
        int octaves,
        double frequency,
        double persistence,
        double lacunarity
    ) const;

    /** Ridged multifractal noise for steep crater rims and sharp rocky ridges */
    double sampleRidged2D(
        double x,
        double z,
        int octaves,
        double frequency,
        double persistence,
        double lacunarity
    ) const;

    /** Domain-warped coordinates to break axial alignment and simulate geological flow */
    Vec2 sampleDomainWarp2D(double x, double z, double warpStrength, double warpFreq) const;

private:
    static double fade(double t) {
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }
    static double grad(int hash, double x, double z);

    std::array<int, 512> m_perm;
    uint32_t m_seed{1337};
};

// ============================================================================
// 2. Analytical Geological Impact Crater Representation
// ============================================================================
struct GeologicalCrater {
    Vec2 center{0.0, 0.0};       // World (X, Z) center position (m)
    double radius{25.0};         // Crater cavity radius (m)
    double depth{6.0};           // Central depression depth (m)
    double rimHeight{1.5};       // Elevated raised rim peak height (m)
    double rimWidth{5.0};        // Width of raised rim ridge (m)
    double irregularity{0.15};   // Non-circular harmonic boundary distortion
    double seedPhase{0.0};       // Angular distortion phase offset (rad)

    /**
     * Calculates the signed elevation adjustment at world position (x, z).
     * Returns true if point is within the crater's morphological sphere of influence.
     */
    bool evaluate(double x, double z, double& outDeltaElevation, double& outCraterFactor) const;
};

// ============================================================================
// 3. Procedural Rock and Boulder Obstacles
// ============================================================================
enum class RockObstacleType {
    SmallBasalt,      // R ~ 0.25 - 0.45m
    MediumBoulder,    // R ~ 0.50 - 0.85m
    LargeFormation    // R ~ 1.00 - 1.80m
};

struct RockObstacle {
    Vec3 position{0.0, 0.0, 0.0};     // World position (sitting flush on ground)
    Vec3 halfExtents{0.5, 0.4, 0.5};  // Dimensions in meters
    Quat rotation{0.0, 0.0, 0.0, 1.0};// World orientation
    RockObstacleType type{RockObstacleType::MediumBoulder};
    double mass{850.0};               // Density ~ 2800 kg/m^3 (basalt/anorthosite)
    int colliderHandle{-1};           // Bullet collision shape handle
    int variantSeed{0};               // Procedural geometry shape seed
    Color3 color{0.25f, 0.16f, 0.12f};// Dark muted basalt / rust rock tint

    std::vector<Vec3> getLocalVertices() const;
};

// ============================================================================
// 4. Procedural Planetary Configuration
// ============================================================================
struct ProceduralTerrainConfig {
    uint32_t seed{19760720};
    std::string presetName{"Mars"};

    int gridResolution{512};          // Heightfield grid cells along each axis (512x512)
    double worldSize{512.0};          // World dimension in meters (1 world unit = 1 meter)
    double baseElevation{0.0};        // Mean reference elevation (m)
    double elevationAmplitude{14.0};  // Peak-to-trough macro elevation variation (m)
    double baseFrequency{0.0055};     // Spatial frequency of lowest octave (1/m)
    int octaves{5};                   // Number of fBm octaves
    double persistence{0.48};         // Amplitude multiplier per octave
    double lacunarity{2.12};          // Frequency multiplier per octave
    double ridgeIntensity{0.30};      // Blend factor for ridged noise (0.0 = pure smooth, 1.0 = sharp)
    double domainWarpStrength{18.0};  // Coordinate perturbation distance (m)

    // Craters
    bool enableCraters{true};
    int numCraters{28};               // Number of analytical impact craters
    double minCraterRadius{6.0};      // Minimum crater radius (m)
    double maxCraterRadius{48.0};     // Maximum crater radius (m)

    // Scattered Rock Obstacles
    bool enableRocks{true};
    int numRocks{120};                // Deterministic scattered boulder count
    double minRockSize{0.35};         // Small rock half-extent (m)
    double maxRockSize{1.35};         // Large boulder half-extent (m)

    // Surface properties & Environment
    SurfaceType defaultSurface{SurfaceType::MarsRegolith};
    PlanetaryEnvironment environment{createMarsEnvironment()};

    // Factory Presets
    static ProceduralTerrainConfig createMarsPreset(uint32_t seed = 19760720);
    static ProceduralTerrainConfig createMoonPreset(uint32_t seed = 19690720);

    // Test Scenarios A through E
    static ProceduralTerrainConfig createScenarioA_FlatPlain(uint32_t seed = 101);
    static ProceduralTerrainConfig createScenarioB_RockyMars(uint32_t seed = 202);
    static ProceduralTerrainConfig createScenarioC_CrateredMoon(uint32_t seed = 303);
    static ProceduralTerrainConfig createScenarioD_DifficultTerrain(uint32_t seed = 404);
    static ProceduralTerrainConfig createScenarioE_NavigationChallenge(uint32_t seed = 505);
};

// ============================================================================
// 5. Authoritative Generated Planetary Terrain Data Container
// ============================================================================
class ProceduralTerrainData {
public:
    ProceduralTerrainData() = default;

    /** Returns true if valid heightfield data is present */
    bool isValid() const { return !m_heightfield.empty() && m_resolution > 1; }

    int getResolution() const { return m_resolution; }
    double getWorldSize() const { return m_worldSize; }
    double getGridSpacing() const { return m_gridSpacing; }
    float getMinHeight() const { return m_minHeight; }
    float getMaxHeight() const { return m_maxHeight; }
    const ProceduralTerrainConfig& getConfig() const { return m_config; }

    const std::vector<float>& getHeightfield() const { return m_heightfield; }
    const std::vector<Vec3>& getNormals() const { return m_normals; }
    const std::vector<SurfaceType>& getSurfaceMaterials() const { return m_surfaceMaterials; }
    const std::vector<GeologicalCrater>& getCraters() const { return m_craters; }
    const std::vector<RockObstacle>& getRocks() const { return m_rocks; }

    /** Authoritative height query with bilinear interpolation across grid cells */
    double sampleHeight(double worldX, double worldZ) const;

    /** Authoritative surface normal query with bilinear interpolation */
    Vec3 sampleNormal(double worldX, double worldZ) const;

    /** Surface material query based on slope, crater proximity, and rock distribution */
    SurfaceType sampleMaterial(double worldX, double worldZ) const;

    /** Slope inclination angle relative to horizontal plane (radians) */
    double sampleSlopeRad(double worldX, double worldZ) const;

    /** Checks whether coordinates fall within valid playable world bounds */
    bool isInside(double worldX, double worldZ) const;

    /** Spatial index query: returns all rocks within a radius of world coordinates */
    std::vector<const RockObstacle*> getRocksNear(double worldX, double worldZ, double radius) const;

private:
    friend class ProceduralTerrainGenerator;

    ProceduralTerrainConfig m_config;
    int m_resolution{0};
    double m_worldSize{0.0};
    double m_gridSpacing{1.0};
    float m_minHeight{0.0f};
    float m_maxHeight{0.0f};

    std::vector<float> m_heightfield;            // Resolution x Resolution row-major heights
    std::vector<Vec3> m_normals;                 // Precomputed seamless vertex normals
    std::vector<SurfaceType> m_surfaceMaterials; // Per-vertex material classification
    std::vector<GeologicalCrater> m_craters;     // Placed impact craters
    std::vector<RockObstacle> m_rocks;           // Placed solid rock obstacles
};

// ============================================================================
// 6. Master Terrain Generator Engine
// ============================================================================
class ProceduralTerrainGenerator {
public:
    static std::shared_ptr<ProceduralTerrainData> generate(const ProceduralTerrainConfig& config);
};

} // namespace torquelab
