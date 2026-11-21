#include "terrain/TerrainManager.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h>
#include "physics/PhysicsWorld.h"
#include "physics/ResourcePath.h"
#include "renderer/ModelLoaderGLB.h"
#include "renderer/SkyRendererGL.h"
#include "renderer/TrackRendererGL.h"

namespace torquelab {

// ============================================================================
// 1. Earth Proving Ground Map (Baseline Terrestrial Test Facility)
// ============================================================================
class ProvingGroundMap : public ITerrainMap {
public:
    std::string getId() const override { return "proving_ground"; }
    std::string getDisplayName() const override { return "Earth Proving Ground"; }
    std::string getDescription() const override {
        return "Terrestrial 6-DOF vehicle dynamics facility (Asphalt Straight, Skidpad, Gravel, Ice, Slalom)";
    }
    TerrainResolutionLevel getResolutionLevel() const override {
        return TerrainResolutionLevel::Level2_LocalMesh;
    }
    PlanetaryEnvironment getEnvironment() const override {
        return createEarthEnvironment();
    }
    std::vector<TerrainSpawnPoint> getSpawnPoints() const override {
        std::vector<TerrainSpawnPoint> result;
        for (const auto& p : getProvingGroundPresets()) {
            result.push_back(TerrainSpawnPoint{p.id, p.label, p.description, p.position, p.yawDeg});
        }
        return result;
    }

    bool buildPhysics(PhysicsWorld& world) override {
        world.rebuildProvingGround();
        return true;
    }

    bool buildGraphics(TrackRendererGL& renderer) override {
        // Will be populated with track elements from PhysicsWorld
        return true;
    }
};

// ============================================================================
// 2. Mars Site Alpha Map (Photogrammetric 3D Surface Patch: mars_terrain0.glb)
// ============================================================================
class MarsSiteAlphaMap : public ITerrainMap {
public:
    std::string getId() const override { return "mars_site_alpha"; }
    std::string getDisplayName() const override { return "Mars Site Alpha (Photogrammetric Site)"; }
    std::string getDescription() const override {
        return "Decimeter-scale 3D surface patch with authentic boulders and scale bar from mars_terrain0.glb";
    }
    TerrainResolutionLevel getResolutionLevel() const override {
        return TerrainResolutionLevel::Level2_LocalMesh;
    }
    PlanetaryEnvironment getEnvironment() const override {
        return createMarsEnvironment();
    }
    std::vector<TerrainSpawnPoint> getSpawnPoints() const override {
        return {
            {"site_alpha_center", "1 Alpha Center", "Central exploration flat", Vec3{-15.0, 1.20, 0.0}, 0.0},
            {"site_alpha_boulder", "2 Boulder Field", "Rock obstacle navigation lane", Vec3{-22.0, 1.15, -4.0}, 45.0}
        };
    }

    bool buildPhysics(PhysicsWorld& world) override {
        const std::string path = ResourcePath::resolve("model/terrain/mars_terrain0.glb");
        LoadedVehicleModel model{};
        if (!ModelLoaderGLB::loadFromFile(path, model, "", false)) {
            std::cerr << "[MarsSiteAlphaMap] Error: Failed to load " << path << std::endl;
            return false;
        }

        const auto& raw = model.chassisCollisionMesh;
        if (raw.vertices.empty() || raw.indices.empty()) {
            std::cerr << "[MarsSiteAlphaMap] Error: No mesh geometry found in " << path << std::endl;
            return false;
        }

        // Convert Vec3 vertices to flat float buffer for Bullet
        const size_t vCount = raw.vertices.size();
        m_bvhVertices.resize(vCount * 3);
        for (size_t i = 0; i < vCount; ++i) {
            m_bvhVertices[i * 3 + 0] = static_cast<float>(raw.vertices[i].x);
            m_bvhVertices[i * 3 + 1] = static_cast<float>(raw.vertices[i].y);
            m_bvhVertices[i * 3 + 2] = static_cast<float>(raw.vertices[i].z);
        }
        m_bvhIndices = raw.indices;

        m_meshData = std::make_unique<btTriangleIndexVertexArray>(
            static_cast<int>(m_bvhIndices.size() / 3),
            reinterpret_cast<int*>(m_bvhIndices.data()),
            static_cast<int>(3 * sizeof(unsigned int)),
            static_cast<int>(vCount),
            m_bvhVertices.data(),
            static_cast<int>(3 * sizeof(float))
        );

        auto bvhShape = std::make_unique<btBvhTriangleMeshShape>(m_meshData.get(), true);
        world.addStaticCollider(
            std::move(bvhShape),
            Vec3{0.0, 0.0, 0.0},
            quatIdentity(),
            SurfaceType::MarsRegolith,
            true
        );

        return true;
    }

    bool buildGraphics(TrackRendererGL& renderer) override {
        const std::string path = ResourcePath::resolve("model/terrain/mars_terrain0.glb");
        m_loadedModel = LoadedVehicleModel{};
        if (!ModelLoaderGLB::loadFromFile(path, m_loadedModel, "", true)) {
            std::cerr << "[MarsSiteAlphaMap] Error: Failed to load visual mesh: " << path << std::endl;
            return false;
        }

        renderer.clearItems();
        for (auto& part : m_loadedModel.chassisParts) {
            TrackRendererGL::RenderableTrackItem item{};
            item.mesh = std::move(part.mesh);
            item.modelMatrix = part.localBindMatrix;
            item.material = part.material;
            item.material.receiveShadows = true;
            item.material.castShadows = true;
            renderer.addRenderableItem(std::move(item));
        }

        const auto env = getEnvironment();
        renderer.buildEnvironmentTerrain(env.groundHazeColor);
        return true;
    }

private:
    std::unique_ptr<btTriangleIndexVertexArray> m_meshData;
    std::vector<float> m_bvhVertices;
    std::vector<unsigned int> m_bvhIndices;
    LoadedVehicleModel m_loadedModel;
};

// ============================================================================
// 3. Mars Site Beta Map (High-Resolution 3D Surface Patch: mars_terrain1.glb)
// ============================================================================
class MarsSiteBetaMap : public ITerrainMap {
public:
    std::string getId() const override { return "mars_site_beta"; }
    std::string getDisplayName() const override { return "Mars Site Beta (Rock Field)"; }
    std::string getDescription() const override {
        return "High-density 3D photogrammetric rocky terrain with 8K texture from mars_terrain1.glb";
    }
    TerrainResolutionLevel getResolutionLevel() const override {
        return TerrainResolutionLevel::Level2_LocalMesh;
    }
    PlanetaryEnvironment getEnvironment() const override {
        return createMarsEnvironment();
    }
    std::vector<TerrainSpawnPoint> getSpawnPoints() const override {
        return {
            {"site_beta_center", "1 Beta Baseline", "Primary landing pad flat", Vec3{0.0, 1.50, 0.0}, 0.0},
            {"site_beta_ridge",  "2 Outcrop Ridge", "Steep basalt outcrop ascent", Vec3{-3.0, 1.80, 5.0}, 90.0}
        };
    }

    bool buildPhysics(PhysicsWorld& world) override {
        const std::string path = ResourcePath::resolve("model/terrain/mars_terrain1.glb");
        LoadedVehicleModel model{};
        if (!ModelLoaderGLB::loadFromFile(path, model, "", false)) {
            std::cerr << "[MarsSiteBetaMap] Error: Failed to load " << path << std::endl;
            return false;
        }

        const auto& raw = model.chassisCollisionMesh;
        if (raw.vertices.empty() || raw.indices.empty()) {
            std::cerr << "[MarsSiteBetaMap] Error: No mesh geometry found in " << path << std::endl;
            return false;
        }

        const size_t vCount = raw.vertices.size();
        m_bvhVertices.resize(vCount * 3);
        for (size_t i = 0; i < vCount; ++i) {
            m_bvhVertices[i * 3 + 0] = static_cast<float>(raw.vertices[i].x);
            m_bvhVertices[i * 3 + 1] = static_cast<float>(raw.vertices[i].y);
            m_bvhVertices[i * 3 + 2] = static_cast<float>(raw.vertices[i].z);
        }
        m_bvhIndices = raw.indices;

        m_meshData = std::make_unique<btTriangleIndexVertexArray>(
            static_cast<int>(m_bvhIndices.size() / 3),
            reinterpret_cast<int*>(m_bvhIndices.data()),
            static_cast<int>(3 * sizeof(unsigned int)),
            static_cast<int>(vCount),
            m_bvhVertices.data(),
            static_cast<int>(3 * sizeof(float))
        );

        auto bvhShape = std::make_unique<btBvhTriangleMeshShape>(m_meshData.get(), true);
        world.addStaticCollider(
            std::move(bvhShape),
            Vec3{0.0, 0.0, 0.0},
            quatIdentity(),
            SurfaceType::MarsBedrock,
            true
        );

        return true;
    }

    bool buildGraphics(TrackRendererGL& renderer) override {
        const std::string path = ResourcePath::resolve("model/terrain/mars_terrain1.glb");
        m_loadedModel = LoadedVehicleModel{};
        if (!ModelLoaderGLB::loadFromFile(path, m_loadedModel, "", true)) {
            std::cerr << "[MarsSiteBetaMap] Error: Failed to load visual mesh: " << path << std::endl;
            return false;
        }

        renderer.clearItems();
        for (auto& part : m_loadedModel.chassisParts) {
            TrackRendererGL::RenderableTrackItem item{};
            item.mesh = std::move(part.mesh);
            item.modelMatrix = part.localBindMatrix;
            item.material = part.material;
            item.material.receiveShadows = true;
            item.material.castShadows = true;
            renderer.addRenderableItem(std::move(item));
        }

        const auto env = getEnvironment();
        renderer.buildEnvironmentTerrain(env.groundHazeColor);
        return true;
    }

private:
    std::unique_ptr<btTriangleIndexVertexArray> m_meshData;
    std::vector<float> m_bvhVertices;
    std::vector<unsigned int> m_bvhIndices;
    LoadedVehicleModel m_loadedModel;
};

// ============================================================================
// 4. Mars Regional DTM Map (1 km x 1 km Preprocessed HiRISE DTM Heightfield)
// ============================================================================
class MarsRegionalDTMMap : public ITerrainMap {
public:
    std::string getId() const override { return "mars_jezero_alpha"; }
    std::string getDisplayName() const override { return "Mars Jezero Crater (HiRISE DTM)"; }
    std::string getDescription() const override {
        return "1 km x 1 km real NASA HiRISE Digital Terrain Model at 1.0 m/px resolution";
    }
    TerrainResolutionLevel getResolutionLevel() const override {
        return TerrainResolutionLevel::Level1_RegionalDTM;
    }
    PlanetaryEnvironment getEnvironment() const override {
        return createMarsEnvironment();
    }
    std::vector<TerrainSpawnPoint> getSpawnPoints() const override {
        return {
            {"dtm_center", "1 Crater Floor", "HiRISE sector center plateau", Vec3{0.0, 6.08 + 0.65, 0.0}, 0.0},
            {"dtm_ridge",  "2 Western Slope", "Gentle crater rim incline", Vec3{-150.0, 10.50, 0.0}, -45.0}
        };
    }

    bool loadBinaryData(const std::string& binPath) {
        if (!m_heightfieldData.empty()) return true;

        const std::string resolved = ResourcePath::resolve(binPath);
        std::ifstream file(resolved, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "[MarsRegionalDTMMap] Error opening " << resolved << " (requested: " << binPath << ")" << std::endl;
            return false;
        }

        file.seekg(0, std::ios::end);
        const std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        const size_t floatCount = static_cast<size_t>(size / sizeof(float));
        if (floatCount != 1024 * 1024) {
            std::cerr << "[MarsRegionalDTMMap] Unexpected float count: " << floatCount << std::endl;
            return false;
        }

        m_heightfieldData.resize(floatCount);
        file.read(reinterpret_cast<char*>(m_heightfieldData.data()), size);

        m_minHeight = m_heightfieldData[0];
        m_maxHeight = m_heightfieldData[0];
        for (float h : m_heightfieldData) {
            if (h < m_minHeight) m_minHeight = h;
            if (h > m_maxHeight) m_maxHeight = h;
        }

        std::cout << "[MarsRegionalDTMMap] Loaded " << floatCount << " heights. Range: ["
                  << m_minHeight << ", " << m_maxHeight << "] m." << std::endl;
        return true;
    }

    bool buildPhysics(PhysicsWorld& world) override {
        if (!loadBinaryData("model/terrain/mars_jezero_alpha.bin")) {
            return false;
        }

        // Create Bullet heightfield shape
        // upAxis = 1 (+Y), PHY_FLOAT
        auto hfShape = std::make_unique<btHeightfieldTerrainShape>(
            1024, 1024,
            m_heightfieldData.data(),
            1.0f,
            m_minHeight,
            m_maxHeight,
            1,
            PHY_FLOAT,
            false
        );
        hfShape->setLocalScaling(btVector3(1.0f, 1.0f, 1.0f));

        // In Bullet, heightfields with upAxis=1 center the bounding box vertically at (minHeight + maxHeight) * 0.5
        const double centerY = static_cast<double>((m_minHeight + m_maxHeight) * 0.5f);
        world.addStaticCollider(
            std::move(hfShape),
            Vec3{0.0, centerY, 0.0},
            quatIdentity(),
            SurfaceType::MarsRegolith,
            true
        );

        return true;
    }

    bool buildGraphics(TrackRendererGL& renderer) override {
        if (!loadBinaryData("model/terrain/mars_jezero_alpha.bin")) {
            return false;
        }

        renderer.clearItems();

        const int cols = 1024;
        const int rows = 1024;
        const float halfWidth = 0.5f * static_cast<float>(cols - 1);
        const float halfDepth = 0.5f * static_cast<float>(rows - 1);

        // Precompute seamless vertex normals across full dataset using central differences
        std::vector<Vec3> fullNormals(static_cast<size_t>(cols * rows));
        for (int r = 0; r < rows; ++r) {
            const int rPrev = std::max(0, r - 1);
            const int rNext = std::min(rows - 1, r + 1);
            for (int c = 0; c < cols; ++c) {
                const int cPrev = std::max(0, c - 1);
                const int cNext = std::min(cols - 1, c + 1);

                const float hL = m_heightfieldData[static_cast<size_t>(r * cols + cPrev)];
                const float hR = m_heightfieldData[static_cast<size_t>(r * cols + cNext)];
                const float hD = m_heightfieldData[static_cast<size_t>(rPrev * cols + c)];
                const float hU = m_heightfieldData[static_cast<size_t>(rNext * cols + c)];

                const float dx = (hR - hL) / static_cast<float>(cNext - cPrev);
                const float dz = (hU - hD) / static_cast<float>(rNext - rPrev);

                fullNormals[static_cast<size_t>(r * cols + c)] = vec3Normalize(Vec3{-static_cast<double>(dx), 1.0, -static_cast<double>(dz)});
            }
        }

        // Divide 1024x1024 terrain into an 8x8 grid of spatial chunks (128m x 128m each)
        const int numChunksX = 8;
        const int numChunksZ = 8;
        const int samplesPerChunkX = 128;
        const int samplesPerChunkZ = 128;
        const int step = 2; // 2m resolution per quad (65x65 vertices per chunk = 4,225 vertices)

        const MaterialProps regolithMat = renderer.getSurfaceMaterial(SurfaceType::MarsRegolith);

        for (int cz = 0; cz < numChunksZ; ++cz) {
            const int rStart = cz * samplesPerChunkZ;
            const int rEnd = (cz + 1) * samplesPerChunkZ; // shared border vertex with adjacent chunk
            const int chunkRows = (rEnd - rStart) / step + 1; // 65

            for (int cx = 0; cx < numChunksX; ++cx) {
                const int cStart = cx * samplesPerChunkX;
                const int cEnd = (cx + 1) * samplesPerChunkX; // shared border vertex with adjacent chunk
                const int chunkCols = (cEnd - cStart) / step + 1; // 65

                const float xMin = static_cast<float>(cStart) - halfWidth;
                const float xMax = static_cast<float>(cEnd) - halfWidth;
                const float zMin = static_cast<float>(rStart) - halfDepth;
                const float zMax = static_cast<float>(rEnd) - halfDepth;

                const float centerChunkX = (xMin + xMax) * 0.5f;
                const float centerChunkZ = (zMin + zMax) * 0.5f;

                float yMinChunk = 1e9f;
                float yMaxChunk = -1e9f;

                std::vector<VertexPNTU> chunkVertices;
                chunkVertices.reserve(static_cast<size_t>(chunkCols * chunkRows));

                for (int r = rStart; r <= rEnd; r += step) {
                    const int rClamped = std::min(r, rows - 1);
                    for (int c = cStart; c <= cEnd; c += step) {
                        const int cClamped = std::min(c, cols - 1);
                        const size_t sampleIdx = static_cast<size_t>(rClamped * cols + cClamped);
                        const float h = m_heightfieldData[sampleIdx];
                        if (h < yMinChunk) yMinChunk = h;
                        if (h > yMaxChunk) yMaxChunk = h;

                        // Chunk-local coordinates relative to chunk center for maximum floating-point precision
                        const float px = (static_cast<float>(c) - halfWidth) - centerChunkX;
                        const float pz = (static_cast<float>(r) - halfDepth) - centerChunkZ;
                        const float py = h;

                        const Vec3& norm = fullNormals[sampleIdx];

                        VertexPNTU v{};
                        v.px = px;
                        v.py = py;
                        v.pz = pz;
                        v.nx = static_cast<float>(norm.x);
                        v.ny = static_cast<float>(norm.y);
                        v.nz = static_cast<float>(norm.z);
                        v.u = static_cast<float>(c) * 0.25f;
                        v.v = static_cast<float>(r) * 0.25f;
                        chunkVertices.push_back(v);
                    }
                }

                std::vector<unsigned int> chunkIndices;
                chunkIndices.reserve(static_cast<size_t>((chunkCols - 1) * (chunkRows - 1) * 6));

                for (int j = 0; j < chunkRows - 1; ++j) {
                    for (int i = 0; i < chunkCols - 1; ++i) {
                        const unsigned int i0 = static_cast<unsigned int>(j * chunkCols + i);
                        const unsigned int i1 = static_cast<unsigned int>(j * chunkCols + (i + 1));
                        const unsigned int i2 = static_cast<unsigned int>((j + 1) * chunkCols + i);
                        const unsigned int i3 = static_cast<unsigned int>((j + 1) * chunkCols + (i + 1));

                        chunkIndices.push_back(i0);
                        chunkIndices.push_back(i2);
                        chunkIndices.push_back(i1);

                        chunkIndices.push_back(i1);
                        chunkIndices.push_back(i2);
                        chunkIndices.push_back(i3);
                    }
                }

                Mesh::computeTangents(chunkVertices, chunkIndices);

                TerrainChunk chunk{};
                chunk.chunkX = cx;
                chunk.chunkZ = cz;
                chunk.centerWorld = Vec3{centerChunkX, static_cast<double>((yMinChunk + yMaxChunk) * 0.5f), centerChunkZ};
                chunk.worldBounds.minPoint = Vec3{xMin, yMinChunk, zMin};
                chunk.worldBounds.maxPoint = Vec3{xMax, yMaxChunk, zMax};
                chunk.worldMatrix = Mat4::fromTranslation(Vec3{centerChunkX, 0.0, centerChunkZ});
                chunk.material = regolithMat;
                chunk.material.receiveShadows = true;
                chunk.material.castShadows = true;
                chunk.isVisible = true;

                chunk.mesh.upload(chunkVertices, chunkIndices);
                renderer.addTerrainChunk(std::move(chunk));
            }
        }

        const auto env = getEnvironment();
        renderer.buildEnvironmentTerrain(env.groundHazeColor, 512.0f, m_minHeight - 2.0f);
        return true;
    }

private:
    std::vector<float> m_heightfieldData;
    float m_minHeight{0.0f};
    float m_maxHeight{0.0f};
};

// ============================================================================
// TerrainManager Implementation
// ============================================================================
void TerrainManager::init() {
    m_maps.clear();
    m_maps.push_back(std::make_shared<ProvingGroundMap>());
    m_maps.push_back(std::make_shared<MarsSiteAlphaMap>());
    m_maps.push_back(std::make_shared<MarsSiteBetaMap>());
    m_maps.push_back(std::make_shared<MarsRegionalDTMMap>());
    m_activeMapIndex = 0;
}

std::shared_ptr<ITerrainMap> TerrainManager::getActiveMap() const {
    if (m_activeMapIndex < m_maps.size()) {
        return m_maps[m_activeMapIndex];
    }
    return nullptr;
}

std::string TerrainManager::getActiveMapId() const {
    if (auto m = getActiveMap()) {
        return m->getId();
    }
    return "proving_ground";
}

bool TerrainManager::switchMap(
    const std::string& mapId,
    PhysicsWorld& world,
    TrackRendererGL& trackRenderer,
    SkyRendererGL& skyRenderer
) {
    size_t targetIdx = m_maps.size();
    for (size_t i = 0; i < m_maps.size(); ++i) {
        if (m_maps[i]->getId() == mapId) {
            targetIdx = i;
            break;
        }
    }

    if (targetIdx >= m_maps.size()) {
        std::cerr << "[TerrainManager] Error: Map ID '" << mapId << "' not found." << std::endl;
        return false;
    }

    m_activeMapIndex = targetIdx;
    auto map = m_maps[m_activeMapIndex];

    std::cout << "[TerrainManager] Switching active map to: " << map->getDisplayName() << " (" << mapId << ")..." << std::endl;

    // 1. Clear static collision shapes
    world.clearStaticColliders();

    // 2. Build map physics
    map->buildPhysics(world);

    // 3. Build map visual graphics
    if (mapId == "proving_ground") {
        trackRenderer.rebuildProvingGround(world.trackElements);
    } else {
        map->buildGraphics(trackRenderer);
    }

    // 4. Update planetary environment
    const auto env = map->getEnvironment();
    world.setEnvironment(env);
    skyRenderer.setEnvironment(env);

    // 5. Teleport vehicle to primary spawn point
    const auto spawns = map->getSpawnPoints();
    if (!spawns.empty() && world.vehicle) {
        const auto& sp = spawns[0];
        SpawnPreset preset{sp.id, sp.label, sp.description, sp.position, sp.yawDeg};
        world.teleportToPreset(preset);
        for (int i = 0; i < 25; ++i) {
            world.stepFixed();
        }
    }

    std::cout << "[TerrainManager] Map switch complete. Environment: " << env.displayName
              << " (g=" << env.gravity << " m/s^2, rho=" << env.airDensity << " kg/m^3)." << std::endl;
    return true;
}

} // namespace torquelab
