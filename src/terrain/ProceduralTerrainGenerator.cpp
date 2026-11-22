#include "terrain/ProceduralTerrainGenerator.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include "renderer/Mesh.h"

namespace torquelab {

// ============================================================================
// PerlinNoise Implementation
// ============================================================================
PerlinNoise::PerlinNoise(uint32_t seed) {
    reseed(seed);
}

void PerlinNoise::reseed(uint32_t seed) {
    m_seed = seed;
    std::array<int, 256> p;
    std::iota(p.begin(), p.end(), 0);

    // Deterministic 64-bit SplitMix-based LCG for reproducible shuffling
    uint64_t state = static_cast<uint64_t>(seed) ^ 0x9E3779B97F4A7C15ULL;
    for (int i = 255; i > 0; --i) {
        state ^= (state >> 30);
        state *= 0xBF58476D1CE4E5B9ULL;
        state ^= (state >> 27);
        state *= 0x94D049BB133111EBULL;
        state ^= (state >> 31);
        const int j = static_cast<int>(state % static_cast<uint64_t>(i + 1));
        std::swap(p[static_cast<size_t>(i)], p[static_cast<size_t>(j)]);
    }

    for (int i = 0; i < 256; ++i) {
        m_perm[static_cast<size_t>(i)] = p[static_cast<size_t>(i)];
        m_perm[static_cast<size_t>(256 + i)] = p[static_cast<size_t>(i)];
    }
}

double PerlinNoise::grad(int hash, double x, double z) {
    // 8 gradient vectors around 2D unit circle
    switch (hash & 7) {
        case 0: return  x + z;
        case 1: return -x + z;
        case 2: return  x - z;
        case 3: return -x - z;
        case 4: return  x * 1.41421356;
        case 5: return -x * 1.41421356;
        case 6: return  z * 1.41421356;
        case 7: return -z * 1.41421356;
        default: return 0.0;
    }
}

double PerlinNoise::sample2D(double x, double z) const {
    const int X = static_cast<int>(std::floor(x)) & 255;
    const int Z = static_cast<int>(std::floor(z)) & 255;

    x -= std::floor(x);
    z -= std::floor(z);

    const double u = fade(x);
    const double v = fade(z);

    const int A  = m_perm[static_cast<size_t>(X)] + Z;
    const int AA = m_perm[static_cast<size_t>(A)];
    const int AB = m_perm[static_cast<size_t>(A + 1)];
    const int B  = m_perm[static_cast<size_t>(X + 1)] + Z;
    const int BA = m_perm[static_cast<size_t>(B)];
    const int BB = m_perm[static_cast<size_t>(B + 1)];

    const double gradAA = grad(m_perm[static_cast<size_t>(AA)], x, z);
    const double gradBA = grad(m_perm[static_cast<size_t>(BA)], x - 1.0, z);
    const double gradAB = grad(m_perm[static_cast<size_t>(AB)], x, z - 1.0);
    const double gradBB = grad(m_perm[static_cast<size_t>(BB)], x - 1.0, z - 1.0);

    const double lerpX1 = lerp(gradAA, gradBA, u);
    const double lerpX2 = lerp(gradAB, gradBB, u);
    return lerp(lerpX1, lerpX2, v);
}

double PerlinNoise::sampleFbm2D(
    double x,
    double z,
    int octaves,
    double frequency,
    double persistence,
    double lacunarity
) const {
    double total = 0.0;
    double amplitude = 1.0;
    double maxAmp = 0.0;
    double curFreq = frequency;

    for (int i = 0; i < octaves; ++i) {
        total += sample2D(x * curFreq, z * curFreq) * amplitude;
        maxAmp += amplitude;
        amplitude *= persistence;
        curFreq *= lacunarity;
    }

    return (maxAmp > 0.0) ? (total / maxAmp) : 0.0;
}

double PerlinNoise::sampleRidged2D(
    double x,
    double z,
    int octaves,
    double frequency,
    double persistence,
    double lacunarity
) const {
    double total = 0.0;
    double amplitude = 1.0;
    double maxAmp = 0.0;
    double curFreq = frequency;

    for (int i = 0; i < octaves; ++i) {
        double v = sample2D(x * curFreq, z * curFreq);
        v = 1.0 - std::abs(v);
        v *= v; // Square for sharp geological ridges
        total += v * amplitude;
        maxAmp += amplitude;
        amplitude *= persistence;
        curFreq *= lacunarity;
    }

    return (maxAmp > 0.0) ? (total / maxAmp) * 2.0 - 1.0 : 0.0;
}

Vec2 PerlinNoise::sampleDomainWarp2D(double x, double z, double warpStrength, double warpFreq) const {
    const double qx = sample2D((x + 0.0) * warpFreq, (z + 0.0) * warpFreq);
    const double qz = sample2D((x + 5.2) * warpFreq, (z + 1.3) * warpFreq);
    return Vec2{x + qx * warpStrength, z + qz * warpStrength};
}

// ============================================================================
// GeologicalCrater Implementation
// ============================================================================
bool GeologicalCrater::evaluate(double x, double z, double& outDeltaElevation, double& outCraterFactor) const {
    const double dx = x - center.x;
    const double dz = z - center.y;
    const double dist = std::sqrt(dx * dx + dz * dz);

    // Max morphological influence: interior cavity + elevated rim + surrounding ejecta blanket
    const double maxInfluence = radius * 2.6;
    if (dist > maxInfluence) {
        outDeltaElevation = 0.0;
        outCraterFactor = 0.0;
        return false;
    }

    // Angular orientation for asymmetric harmonic rim irregularity
    const double angle = std::atan2(dz, dx);
    const double irregularRadius = radius * (1.0 + irregularity * (std::cos(3.0 * angle + seedPhase) + 0.4 * std::sin(5.0 * angle + 2.0 * seedPhase)));

    const double normDist = dist / std::max(1.0, irregularRadius);

    if (normDist <= 1.0) {
        // Interior crater bowl: parabolic depression tapering up to rim
        // Parabolic depression: h(r) = -depth * (1 - normDist^2)
        const double depression = -depth * (1.0 - normDist * normDist);
        // Gaussian raised rim lip contribution
        const double rimDist = (dist - irregularRadius) / std::max(1.0, rimWidth);
        const double rimElevation = rimHeight * std::exp(-rimDist * rimDist * 2.0);

        outDeltaElevation = depression + rimElevation;
        outCraterFactor = 1.0 - normDist;
        return true;
    } else {
        // Exterior: elevated rim ridge + power-law ejecta blanket
        const double rimDist = (dist - irregularRadius) / std::max(1.0, rimWidth);
        const double rimElevation = rimHeight * std::exp(-rimDist * rimDist * 1.5);
        // Ejecta blanket tapering outward
        const double ejectaFactor = clamp((maxInfluence - dist) / (maxInfluence - irregularRadius), 0.0, 1.0);
        const double ejectaElevation = rimElevation * ejectaFactor;

        outDeltaElevation = ejectaElevation;
        outCraterFactor = ejectaFactor * 0.5;
        return (outDeltaElevation > 1e-4);
    }
}

// ============================================================================
// ProceduralTerrainConfig Factory Presets
// ============================================================================
ProceduralTerrainConfig ProceduralTerrainConfig::createMarsPreset(uint32_t seed) {
    ProceduralTerrainConfig cfg;
    cfg.seed = seed;
    cfg.presetName = "Mars Procedural";
    cfg.gridResolution = 512;
    cfg.worldSize = 512.0;
    cfg.baseElevation = 0.0;
    cfg.elevationAmplitude = 14.0;
    cfg.baseFrequency = 0.0055;
    cfg.octaves = 5;
    cfg.persistence = 0.48;
    cfg.lacunarity = 2.15;
    cfg.ridgeIntensity = 0.28;
    cfg.domainWarpStrength = 16.0;

    cfg.enableCraters = true;
    cfg.numCraters = 26;
    cfg.minCraterRadius = 8.0;
    cfg.maxCraterRadius = 45.0;

    cfg.enableRocks = true;
    cfg.numRocks = 130;
    cfg.minRockSize = 0.35;
    cfg.maxRockSize = 1.25;

    cfg.defaultSurface = SurfaceType::MarsRegolith;
    cfg.environment = createMarsEnvironment();
    return cfg;
}

ProceduralTerrainConfig ProceduralTerrainConfig::createMoonPreset(uint32_t seed) {
    ProceduralTerrainConfig cfg;
    cfg.seed = seed;
    cfg.presetName = "Moon Procedural";
    cfg.gridResolution = 512;
    cfg.worldSize = 512.0;
    cfg.baseElevation = 0.0;
    cfg.elevationAmplitude = 24.0;
    cfg.baseFrequency = 0.0065;
    cfg.octaves = 6;
    cfg.persistence = 0.52;
    cfg.lacunarity = 2.05;
    cfg.ridgeIntensity = 0.45; // Sharper lunar highlands
    cfg.domainWarpStrength = 10.0;

    cfg.enableCraters = true;
    cfg.numCraters = 42;       // Denser impact history on Moon
    cfg.minCraterRadius = 6.0;
    cfg.maxCraterRadius = 55.0;

    cfg.enableRocks = true;
    cfg.numRocks = 160;
    cfg.minRockSize = 0.40;
    cfg.maxRockSize = 1.50;

    cfg.defaultSurface = SurfaceType::Gravel; // Fine anorthosite lunar regolith
    cfg.environment = createMoonEnvironment();
    return cfg;
}

ProceduralTerrainConfig ProceduralTerrainConfig::createScenarioA_FlatPlain(uint32_t seed) {
    ProceduralTerrainConfig cfg = createMarsPreset(seed);
    cfg.presetName = "Scenario A: Flat Planetary Plain";
    cfg.elevationAmplitude = 1.5; // Very gentle undulating plain
    cfg.numCraters = 2;
    cfg.minCraterRadius = 5.0;
    cfg.maxCraterRadius = 12.0;
    cfg.numRocks = 20;
    cfg.ridgeIntensity = 0.05;
    return cfg;
}

ProceduralTerrainConfig ProceduralTerrainConfig::createScenarioB_RockyMars(uint32_t seed) {
    ProceduralTerrainConfig cfg = createMarsPreset(seed);
    cfg.presetName = "Scenario B: Rocky Mars Terrain";
    cfg.elevationAmplitude = 12.0;
    cfg.numCraters = 18;
    cfg.numRocks = 180; // Dense rock field
    return cfg;
}

ProceduralTerrainConfig ProceduralTerrainConfig::createScenarioC_CrateredMoon(uint32_t seed) {
    ProceduralTerrainConfig cfg = createMoonPreset(seed);
    cfg.presetName = "Scenario C: Cratered Moon Terrain";
    cfg.numCraters = 50;
    cfg.elevationAmplitude = 26.0;
    return cfg;
}

ProceduralTerrainConfig ProceduralTerrainConfig::createScenarioD_DifficultTerrain(uint32_t seed) {
    ProceduralTerrainConfig cfg = createMarsPreset(seed);
    cfg.presetName = "Scenario D: Difficult Terrain (Steep & Rugged)";
    cfg.elevationAmplitude = 28.0;
    cfg.ridgeIntensity = 0.60;
    cfg.numRocks = 220;
    return cfg;
}

ProceduralTerrainConfig ProceduralTerrainConfig::createScenarioE_NavigationChallenge(uint32_t seed) {
    ProceduralTerrainConfig cfg = createMarsPreset(seed);
    cfg.presetName = "Scenario E: Navigation Challenge";
    cfg.elevationAmplitude = 14.0;
    cfg.numCraters = 20;
    cfg.numRocks = 160;
    return cfg;
}

// ============================================================================
// ProceduralTerrainData Implementation
// ============================================================================
double ProceduralTerrainData::sampleHeight(double worldX, double worldZ) const {
    if (!isValid()) return 0.0;

    const double halfSize = m_worldSize * 0.5;
    const double normX = (worldX + halfSize) / m_gridSpacing;
    const double normZ = (worldZ + halfSize) / m_gridSpacing;

    const int c0 = clamp(static_cast<int>(std::floor(normX)), 0, m_resolution - 1);
    const int c1 = clamp(c0 + 1, 0, m_resolution - 1);
    const int r0 = clamp(static_cast<int>(std::floor(normZ)), 0, m_resolution - 1);
    const int r1 = clamp(r0 + 1, 0, m_resolution - 1);

    const double tx = normX - std::floor(normX);
    const double tz = normZ - std::floor(normZ);

    const double h00 = static_cast<double>(m_heightfield[static_cast<size_t>(r0 * m_resolution + c0)]);
    const double h10 = static_cast<double>(m_heightfield[static_cast<size_t>(r0 * m_resolution + c1)]);
    const double h01 = static_cast<double>(m_heightfield[static_cast<size_t>(r1 * m_resolution + c0)]);
    const double h11 = static_cast<double>(m_heightfield[static_cast<size_t>(r1 * m_resolution + c1)]);

    const double hBottom = lerp(h00, h10, tx);
    const double hTop    = lerp(h01, h11, tx);
    return lerp(hBottom, hTop, tz);
}

Vec3 ProceduralTerrainData::sampleNormal(double worldX, double worldZ) const {
    if (!isValid()) return Vec3{0.0, 1.0, 0.0};

    const double halfSize = m_worldSize * 0.5;
    const double normX = (worldX + halfSize) / m_gridSpacing;
    const double normZ = (worldZ + halfSize) / m_gridSpacing;

    const int c0 = clamp(static_cast<int>(std::floor(normX)), 0, m_resolution - 1);
    const int c1 = clamp(c0 + 1, 0, m_resolution - 1);
    const int r0 = clamp(static_cast<int>(std::floor(normZ)), 0, m_resolution - 1);
    const int r1 = clamp(r0 + 1, 0, m_resolution - 1);

    const double tx = normX - std::floor(normX);
    const double tz = normZ - std::floor(normZ);

    const Vec3& n00 = m_normals[static_cast<size_t>(r0 * m_resolution + c0)];
    const Vec3& n10 = m_normals[static_cast<size_t>(r0 * m_resolution + c1)];
    const Vec3& n01 = m_normals[static_cast<size_t>(r1 * m_resolution + c0)];
    const Vec3& n11 = m_normals[static_cast<size_t>(r1 * m_resolution + c1)];

    const Vec3 nBottom = vec3Lerp(n00, n10, tx);
    const Vec3 nTop    = vec3Lerp(n01, n11, tx);
    return vec3Normalize(vec3Lerp(nBottom, nTop, tz));
}

SurfaceType ProceduralTerrainData::sampleMaterial(double worldX, double worldZ) const {
    if (!isValid()) return m_config.defaultSurface;

    const double slope = sampleSlopeRad(worldX, worldZ);
    // Steep rocky slopes expose bedrock
    if (slope > 0.38) { // > 22 degrees
        return SurfaceType::DryAsphalt; // Representing solid bedrock friction
    }

    const double halfSize = m_worldSize * 0.5;
    const int c = clamp(static_cast<int>(std::round((worldX + halfSize) / m_gridSpacing)), 0, m_resolution - 1);
    const int r = clamp(static_cast<int>(std::round((worldZ + halfSize) / m_gridSpacing)), 0, m_resolution - 1);
    return m_surfaceMaterials[static_cast<size_t>(r * m_resolution + c)];
}

double ProceduralTerrainData::sampleSlopeRad(double worldX, double worldZ) const {
    const Vec3 norm = sampleNormal(worldX, worldZ);
    const double cosTheta = clamp(norm.y, -1.0, 1.0);
    return std::acos(cosTheta);
}

bool ProceduralTerrainData::isInside(double worldX, double worldZ) const {
    const double halfSize = m_worldSize * 0.5;
    return (std::abs(worldX) <= halfSize && std::abs(worldZ) <= halfSize);
}

std::vector<const RockObstacle*> ProceduralTerrainData::getRocksNear(double worldX, double worldZ, double radius) const {
    std::vector<const RockObstacle*> result;
    const double r2 = radius * radius;
    for (const auto& rock : m_rocks) {
        const double dx = rock.position.x - worldX;
        const double dz = rock.position.z - worldZ;
        if (dx * dx + dz * dz <= r2) {
            result.push_back(&rock);
        }
    }
    return result;
}

// ============================================================================
// ProceduralTerrainGenerator::generate
// ============================================================================
std::shared_ptr<ProceduralTerrainData> ProceduralTerrainGenerator::generate(const ProceduralTerrainConfig& config) {
    auto data = std::make_shared<ProceduralTerrainData>();
    data->m_config = config;
    data->m_resolution = std::max(64, config.gridResolution);
    data->m_worldSize = std::max(64.0, config.worldSize);
    data->m_gridSpacing = data->m_worldSize / static_cast<double>(data->m_resolution - 1);

    const int N = data->m_resolution;
    const size_t totalSamples = static_cast<size_t>(N * N);
    data->m_heightfield.resize(totalSamples, 0.0f);
    data->m_normals.resize(totalSamples, Vec3{0.0, 1.0, 0.0});
    data->m_surfaceMaterials.resize(totalSamples, config.defaultSurface);

    PerlinNoise noiseBase(config.seed);
    PerlinNoise noiseDetail(config.seed + 101);
    PerlinNoise noiseWarp(config.seed + 202);

    // 1. Generate Deterministic Craters
    if (config.enableCraters && config.numCraters > 0) {
        std::mt19937 craterRng(config.seed + 777);
        const double halfSize = config.worldSize * 0.5;
        // Keep crater centers safely inside playable terrain (exclude extreme borders)
        const double margin = 35.0;
        std::uniform_real_distribution<double> distPos(-halfSize + margin, halfSize - margin);
        std::uniform_real_distribution<double> distRadius(config.minCraterRadius, config.maxCraterRadius);
        std::uniform_real_distribution<double> distPhase(0.0, 6.2831853);
        std::uniform_real_distribution<double> distIrreg(0.08, 0.22);

        data->m_craters.reserve(static_cast<size_t>(config.numCraters));
        for (int i = 0; i < config.numCraters; ++i) {
            GeologicalCrater crater;
            crater.center = Vec2{distPos(craterRng), distPos(craterRng)};
            // Ensure rover spawn area at origin (0, 0) remains safe for initial deployment
            if (std::sqrt(crater.center.x * crater.center.x + crater.center.y * crater.center.y) < 20.0) {
                crater.center.x += (crater.center.x >= 0.0 ? 30.0 : -30.0);
            }
            crater.radius = distRadius(craterRng);
            // Geological depth-to-diameter ratio (typically 0.15 - 0.22 for simple impact craters)
            crater.depth = crater.radius * 0.22;
            crater.rimHeight = crater.depth * 0.24;
            crater.rimWidth = crater.radius * 0.25;
            crater.irregularity = distIrreg(craterRng);
            crater.seedPhase = distPhase(craterRng);
            data->m_craters.push_back(crater);
        }
    }

    // 2. Generate Continuous Heightfield Surface
    const double halfSize = config.worldSize * 0.5;
    data->m_minHeight = 1e9f;
    data->m_maxHeight = -1e9f;

    for (int r = 0; r < N; ++r) {
        const double wz = static_cast<double>(r) * data->m_gridSpacing - halfSize;
        for (int c = 0; c < N; ++c) {
            const double wx = static_cast<double>(c) * data->m_gridSpacing - halfSize;

            // Coordinate domain warping to break Cartesian grid artifacts
            const Vec2 warped = noiseWarp.sampleDomainWarp2D(wx, wz, config.domainWarpStrength, 0.0035);

            // Layer A: Macro elevation fBm
            const double fbm = noiseBase.sampleFbm2D(
                warped.x, warped.y,
                config.octaves,
                config.baseFrequency,
                config.persistence,
                config.lacunarity
            );

            // Layer B: Ridged multifractal for steep rocky outcrops and crater ridges
            const double ridged = noiseDetail.sampleRidged2D(
                warped.x, warped.y,
                config.octaves - 1,
                config.baseFrequency * 1.8,
                config.persistence,
                config.lacunarity
            );

            // Blend macro terrain and ridges
            double height = config.baseElevation + config.elevationAmplitude * lerp(fbm, ridged, config.ridgeIntensity);

            // Layer C: Morphological Impact Crater Depressions and Rims
            double maxCraterFactor = 0.0;
            for (const auto& crater : data->m_craters) {
                double deltaH = 0.0;
                double cFactor = 0.0;
                if (crater.evaluate(wx, wz, deltaH, cFactor)) {
                    height += deltaH;
                    if (cFactor > maxCraterFactor) maxCraterFactor = cFactor;
                }
            }

            const size_t idx = static_cast<size_t>(r * N + c);
            data->m_heightfield[idx] = static_cast<float>(height);

            // Material classification
            if (maxCraterFactor > 0.45) {
                data->m_surfaceMaterials[idx] = SurfaceType::Gravel; // Disturbed ejecta / shattered crater breccia
            } else {
                data->m_surfaceMaterials[idx] = config.defaultSurface;
            }

            if (static_cast<float>(height) < data->m_minHeight) data->m_minHeight = static_cast<float>(height);
            if (static_cast<float>(height) > data->m_maxHeight) data->m_maxHeight = static_cast<float>(height);
        }
    }

    // 3. Precompute Seamless Vertex Normals via Central Differences
    for (int r = 0; r < N; ++r) {
        const int rPrev = std::max(0, r - 1);
        const int rNext = std::min(N - 1, r + 1);
        for (int c = 0; c < N; ++c) {
            const int cPrev = std::max(0, c - 1);
            const int cNext = std::min(N - 1, c + 1);

            const double hL = static_cast<double>(data->m_heightfield[static_cast<size_t>(r * N + cPrev)]);
            const double hR = static_cast<double>(data->m_heightfield[static_cast<size_t>(r * N + cNext)]);
            const double hD = static_cast<double>(data->m_heightfield[static_cast<size_t>(rPrev * N + c)]);
            const double hU = static_cast<double>(data->m_heightfield[static_cast<size_t>(rNext * N + c)]);

            const double distH = static_cast<double>(cNext - cPrev) * data->m_gridSpacing;
            const double distV = static_cast<double>(rNext - rPrev) * data->m_gridSpacing;

            const double dx = (hR - hL) / distH;
            const double dz = (hU - hD) / distV;

            data->m_normals[static_cast<size_t>(r * N + c)] = vec3Normalize(Vec3{-dx, 1.0, -dz});
        }
    }

    // 4. Generate Scattered Solid Rock Obstacles with Spatial Noise Clustering
    if (config.enableRocks && config.numRocks > 0) {
        std::mt19937 rockRng(config.seed + 9999);
        PerlinNoise densityNoise(config.seed + 104729);

        const double margin = 20.0;
        std::uniform_real_distribution<double> distPos(-halfSize + margin, halfSize - margin);
        std::uniform_real_distribution<double> dist01(0.0, 1.0);
        std::uniform_real_distribution<double> distAngle(0.0, 6.2831853);

        const bool isMoon = (config.presetName.find("Moon") != std::string::npos);

        data->m_rocks.reserve(static_cast<size_t>(config.numRocks));
        const int maxAttempts = config.numRocks * 16;
        int attempts = 0;

        while (static_cast<int>(data->m_rocks.size()) < config.numRocks && attempts < maxAttempts) {
            ++attempts;
            const double rx = distPos(rockRng);
            const double rz = distPos(rockRng);

            // Safe radius around rover origin spawn (0, 0)
            if (std::sqrt(rx * rx + rz * rz) < 12.0) {
                continue;
            }

            // Continuous spatial density field from multi-octave fBm noise
            const double rawDensity = densityNoise.sampleFbm2D(rx, rz, 3, 0.012, 0.5, 2.0);
            double clusterVal = (rawDensity + 1.0) * 0.5; // [0.0, 1.0]

            // Geological impact crater rim ejecta clustering boost
            for (const auto& crater : data->m_craters) {
                const double dx = rx - crater.center.x;
                const double dz = rz - crater.center.y;
                const double dCrater = std::sqrt(dx * dx + dz * dz);
                if (dCrater >= crater.radius * 0.85 && dCrater <= crater.radius + crater.rimWidth * 2.5) {
                    clusterVal = std::min(1.0, clusterVal + 0.35);
                    break;
                }
            }

            // Acceptance probability based on spatial noise density
            // Sparse plains (<0.28): 5% accept
            // Moderate zones (0.28..0.65): 35% accept
            // Dense boulder clusters (>=0.65): 85% accept
            double acceptProb = 0.35;
            if (clusterVal < 0.28) {
                acceptProb = 0.05;
            } else if (clusterVal >= 0.65) {
                acceptProb = 0.85;
            }

            if (dist01(rockRng) > acceptProb) {
                continue;
            }

            const double groundH = data->sampleHeight(rx, rz);
            const Vec3 groundNorm = data->sampleNormal(rx, rz);

            // Exclude near-vertical cliffs where rocks cannot naturally sit stable
            if (groundNorm.y < 0.60) {
                continue;
            }

            // Natural size distribution:
            // ~60% small basalt stones (R ~ 0.25 - 0.45m)
            // ~28% medium boulders (R ~ 0.50 - 0.95m)
            // ~12% large boulder formations (R ~ 1.10 - 1.85m)
            const double sizeRoll = dist01(rockRng);
            double baseRadius = 0.0;
            RockObstacleType rockType = RockObstacleType::SmallBasalt;

            if (sizeRoll < 0.60) {
                baseRadius = std::uniform_real_distribution<double>(0.25, 0.45)(rockRng);
                rockType = RockObstacleType::SmallBasalt;
            } else if (sizeRoll < 0.88) {
                baseRadius = std::uniform_real_distribution<double>(0.50, 0.95)(rockRng);
                rockType = RockObstacleType::MediumBoulder;
            } else {
                baseRadius = std::uniform_real_distribution<double>(1.10, 1.85)(rockRng);
                rockType = RockObstacleType::LargeFormation;
            }

            // Scale within user-configured rock size boundaries
            const double scaleRange = std::max(0.1, config.maxRockSize - config.minRockSize);
            baseRadius = config.minRockSize + (baseRadius / 1.85) * scaleRange;

            // Natural irregular aspect ratio scaling
            const double sx = baseRadius * std::uniform_real_distribution<double>(0.80, 1.25)(rockRng);
            const double sy = baseRadius * std::uniform_real_distribution<double>(0.65, 0.85)(rockRng); // Weathered flatter profile
            const double sz = baseRadius * std::uniform_real_distribution<double>(0.80, 1.25)(rockRng);
            const double rEffective = std::max(sx, sz);

            // Inter-rock clearance check: prevent overlapping / interpenetrating colliders
            bool overlaps = false;
            for (const auto& existing : data->m_rocks) {
                const double d2 = (rx - existing.position.x) * (rx - existing.position.x) +
                                  (rz - existing.position.z) * (rz - existing.position.z);
                const double rMin = rEffective + std::max(existing.halfExtents.x, existing.halfExtents.z) + 0.35;
                if (d2 < rMin * rMin) {
                    overlaps = true;
                    break;
                }
            }
            if (overlaps) {
                continue;
            }

            RockObstacle rock;
            // Embed ~25-30% into the regolith for realistic contact stability (no floating, no excessive sinking)
            rock.position = Vec3{rx, groundH + sy * 0.25, rz};
            rock.halfExtents = Vec3{sx, sy, sz};
            rock.type = rockType;

            // Align rock up-axis with local terrain normal, plus random yaw around surface normal
            const Vec3 axis = vec3Cross(Vec3{0.0, 1.0, 0.0}, groundNorm);
            const double axisLen = vec3Length(axis);
            const Quat alignQ = (axisLen > 1e-5)
                ? quatFromAxisAngle(vec3Scale(axis, 1.0 / axisLen), std::acos(clamp(groundNorm.y, -1.0, 1.0)))
                : quatIdentity();
            const double yawAngle = distAngle(rockRng);
            const Quat yawQ = quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, yawAngle);
            rock.rotation = quatNormalize(quatMultiply(alignQ, yawQ));

            // Unique procedural shape seed for faceted low-poly morphology
            rock.variantSeed = static_cast<int>(rockRng() & 0x7FFFFFFF);

            // Natural muted planetary rock colors
            if (isMoon) {
                const int colorVariant = static_cast<int>(rockRng() % 3);
                if (colorVariant == 0) {
                    rock.color = Color3{0.32f, 0.32f, 0.35f}; // Anorthosite grey
                } else if (colorVariant == 1) {
                    rock.color = Color3{0.25f, 0.25f, 0.27f}; // Dark lunar basalt
                } else {
                    rock.color = Color3{0.20f, 0.20f, 0.22f}; // Impact melt breccia
                }
            } else {
                const int colorVariant = static_cast<int>(rockRng() % 4);
                if (colorVariant == 0) {
                    rock.color = Color3{0.28f, 0.16f, 0.12f}; // Muted reddish-brown
                } else if (colorVariant == 1) {
                    rock.color = Color3{0.32f, 0.18f, 0.14f}; // Dark rust
                } else if (colorVariant == 2) {
                    rock.color = Color3{0.22f, 0.18f, 0.16f}; // Brownish-grey
                } else {
                    rock.color = Color3{0.18f, 0.15f, 0.14f}; // Weathered dark basalt
                }
            }

            // Approximate physical mass: Volume * Basalt Density (2800 kg/m^3) * convex fill factor
            const double volume = 8.0 * sx * sy * sz * 0.65;
            rock.mass = volume * 2800.0;

            data->m_rocks.push_back(rock);
        }
    }

    return data;
}

std::vector<Vec3> RockObstacle::getLocalVertices() const {
    return Mesh::getLowPolyRockHullPoints(
        static_cast<float>(halfExtents.x),
        static_cast<float>(halfExtents.y),
        static_cast<float>(halfExtents.z),
        variantSeed
    );
}

} // namespace torquelab
