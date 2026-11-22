#include "terrain/ProceduralPlanetMap.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include "physics/PhysicsWorld.h"
#include "renderer/TrackRendererGL.h"

namespace torquelab {

ProceduralPlanetMap::ProceduralPlanetMap(
    const std::string& id,
    const std::string& displayName,
    const std::string& description,
    const ProceduralTerrainConfig& config
) : m_id(id),
    m_displayName(displayName),
    m_description(description),
    m_config(config) {
    ensureGenerated();
}

void ProceduralPlanetMap::ensureGenerated() {
    if (!m_data || !m_data->isValid()) {
        m_data = ProceduralTerrainGenerator::generate(m_config);
    }
}

void ProceduralPlanetMap::regenerate(const ProceduralTerrainConfig& newConfig) {
    m_config = newConfig;
    m_data = ProceduralTerrainGenerator::generate(m_config);
}

std::vector<TerrainSpawnPoint> ProceduralPlanetMap::getSpawnPoints() const {
    const double spawnH0 = m_data ? m_data->sampleHeight(0.0, 0.0) : 0.0;
    const double spawnH1 = m_data ? m_data->sampleHeight(60.0, 60.0) : 0.0;
    const double spawnH2 = m_data ? m_data->sampleHeight(-60.0, 60.0) : 0.0;

    return {
        TerrainSpawnPoint{"spawn_origin", "Base Camp (Origin)", "Central deployment plain", Vec3{0.0, spawnH0 + 0.45, 0.0}, 0.0},
        TerrainSpawnPoint{"spawn_north_east", "Station Alpha", "North-East elevated terrace", Vec3{60.0, spawnH1 + 0.45, 60.0}, 45.0},
        TerrainSpawnPoint{"spawn_north_west", "Station Beta", "North-West geological basin", Vec3{-60.0, spawnH2 + 0.45, 60.0}, -45.0}
    };
}

bool ProceduralPlanetMap::buildPhysics(PhysicsWorld& world) {
    ensureGenerated();
    if (!m_data || !m_data->isValid()) return false;

    m_colliderHandles.clear();

    const int N = m_data->getResolution();
    const float minH = m_data->getMinHeight();
    const float maxH = m_data->getMaxHeight();

    // 1. Build Heightfield Collision Terrain
    auto hfShape = std::make_unique<btHeightfieldTerrainShape>(
        N, N,
        m_data->getHeightfield().data(),
        1.0f,
        minH,
        maxH,
        1, // upAxis = 1 (+Y)
        PHY_FLOAT,
        false
    );

    const float scalingXZ = static_cast<float>(m_data->getGridSpacing());
    hfShape->setLocalScaling(btVector3(scalingXZ, 1.0f, scalingXZ));

    const double centerY = static_cast<double>((minH + maxH) * 0.5f);
    const int hfHandle = world.addStaticCollider(
        std::move(hfShape),
        Vec3{0.0, centerY, 0.0},
        quatIdentity(),
        m_config.defaultSurface,
        true
    );
    m_colliderHandles.push_back(hfHandle);

    // 2. Build Solid Collision Rigid Bodies for Scattered Rocks & Boulders (1:1 Matching Convex Hulls)
    for (const auto& rock : m_data->getRocks()) {
        auto hullShape = std::make_unique<btConvexHullShape>();
        const auto localVerts = rock.getLocalVertices();
        for (const auto& v : localVerts) {
            hullShape->addPoint(btVector3(
                static_cast<btScalar>(v.x),
                static_cast<btScalar>(v.y),
                static_cast<btScalar>(v.z)
            ), false);
        }
        hullShape->recalcLocalAabb();
        hullShape->setMargin(static_cast<btScalar>(0.01)); // 10 mm skin margin for exact physical contact

        const int rockHandle = world.addStaticCollider(
            std::move(hullShape),
            rock.position,
            rock.rotation,
            SurfaceType::DryAsphalt, // High-friction solid rock
            false // Non-driveable obstacle
        );
        m_colliderHandles.push_back(rockHandle);
    }

    std::cout << "[ProceduralPlanetMap] Built " << m_colliderHandles.size() << " rock colliders in PhysicsWorld." << std::endl;
    return true;
}

bool ProceduralPlanetMap::buildGraphics(TrackRendererGL& renderer) {
    ensureGenerated();
    if (!m_data || !m_data->isValid()) return false;

    renderer.clearItems();
    renderer.clearTerrainChunks();

    const int N = m_data->getResolution();
    const auto& heights = m_data->getHeightfield();
    const auto& normals = m_data->getNormals();
    const double halfSize = m_data->getWorldSize() * 0.5;

    // Divide 512x512 into an 8x8 grid of spatial chunks (64 samples per chunk)
    const int numChunksX = 8;
    const int numChunksZ = 8;
    const int samplesPerChunkX = (N - 1) / numChunksX;
    const int samplesPerChunkZ = (N - 1) / numChunksZ;
    const int step = 1; // Full 1m vertex mesh resolution

    const MaterialProps surfaceMat = renderer.getSurfaceMaterial(m_config.defaultSurface);
    MaterialProps rockMat{};
    rockMat.textureId = 0;
    rockMat.roughness = 0.88f;
    rockMat.metalness = 0.04f;
    rockMat.doubleSided = true;
    rockMat.castShadows = true;
    rockMat.receiveShadows = true;

    for (int cz = 0; cz < numChunksZ; ++cz) {
        const int rStart = cz * samplesPerChunkZ;
        const int rEnd = (cz + 1) * samplesPerChunkZ;
        const int chunkRows = (rEnd - rStart) / step + 1;

        for (int cx = 0; cx < numChunksX; ++cx) {
            const int cStart = cx * samplesPerChunkX;
            const int cEnd = (cx + 1) * samplesPerChunkX;
            const int chunkCols = (cEnd - cStart) / step + 1;

            const float xMin = static_cast<float>(static_cast<double>(cStart) * m_data->getGridSpacing() - halfSize);
            const float xMax = static_cast<float>(static_cast<double>(cEnd) * m_data->getGridSpacing() - halfSize);
            const float zMin = static_cast<float>(static_cast<double>(rStart) * m_data->getGridSpacing() - halfSize);
            const float zMax = static_cast<float>(static_cast<double>(rEnd) * m_data->getGridSpacing() - halfSize);

            const float centerChunkX = (xMin + xMax) * 0.5f;
            const float centerChunkZ = (zMin + zMax) * 0.5f;

            float yMinChunk = 1e9f;
            float yMaxChunk = -1e9f;

            std::vector<VertexPNTU> chunkVertices;
            chunkVertices.reserve(static_cast<size_t>(chunkCols * chunkRows));

            for (int r = rStart; r <= rEnd; r += step) {
                const int rClamped = std::min(r, N - 1);
                for (int c = cStart; c <= cEnd; c += step) {
                    const int cClamped = std::min(c, N - 1);
                    const size_t sampleIdx = static_cast<size_t>(rClamped * N + cClamped);
                    const float h = heights[sampleIdx];
                    if (h < yMinChunk) yMinChunk = h;
                    if (h > yMaxChunk) yMaxChunk = h;

                    const float px = static_cast<float>(static_cast<double>(c) * m_data->getGridSpacing() - halfSize) - centerChunkX;
                    const float pz = static_cast<float>(static_cast<double>(r) * m_data->getGridSpacing() - halfSize) - centerChunkZ;
                    const float py = h;

                    const Vec3& norm = normals[sampleIdx];

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
            chunkIndices.reserve(static_cast<size_t>((chunkRows - 1) * (chunkCols - 1) * 6));
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
            chunk.material = surfaceMat;
            chunk.material.receiveShadows = true;
            chunk.material.castShadows = true;
            chunk.isVisible = true;

            chunk.mesh.upload(chunkVertices, chunkIndices);
            renderer.addTerrainChunk(std::move(chunk));
        }
    }

    // 3. Render Solid Rock Obstacles with Faceted Low-Poly Geometry
    for (const auto& rock : m_data->getRocks()) {
        TrackRendererGL::RenderableTrackItem item;
        item.mesh = Mesh::createLowPolyRock(
            static_cast<float>(rock.halfExtents.x),
            static_cast<float>(rock.halfExtents.y),
            static_cast<float>(rock.halfExtents.z),
            rock.variantSeed
        );
        item.modelMatrix = Mat4::fromTRS(rock.position, rock.rotation);
        item.material = rockMat;
        item.material.baseColor = rock.color;
        item.material.textureId = 0;
        item.material.doubleSided = true;
        item.material.roughness = 0.88f;
        item.material.metalness = 0.04f;
        item.material.receiveShadows = true;
        item.material.castShadows = true;
        renderer.addRenderableItem(std::move(item));
    }
    std::cout << "[ProceduralPlanetMap] Built " << m_data->getRocks().size() << " visual rock obstacle meshes." << std::endl;

    // 4. Perimeter Horizon Skirt
    const auto env = getEnvironment();
    renderer.buildEnvironmentTerrain(env.groundHazeColor, static_cast<float>(m_data->getWorldSize()), m_data->getMinHeight() - 2.0f);
    return true;
}

bool ProceduralPlanetMap::getElevationAt(double worldX, double worldZ, double& outElevation, Vec3& outNormal) const {
    if (!m_data || !m_data->isValid()) {
        outElevation = 0.0;
        outNormal = Vec3{0.0, 1.0, 0.0};
        return false;
    }
    outElevation = m_data->sampleHeight(worldX, worldZ);
    outNormal = m_data->sampleNormal(worldX, worldZ);
    return m_data->isInside(worldX, worldZ);
}

SurfaceType ProceduralPlanetMap::getSurfaceMaterialAt(double worldX, double worldZ) const {
    return m_data ? m_data->sampleMaterial(worldX, worldZ) : m_config.defaultSurface;
}

bool ProceduralPlanetMap::isPositionInsidePlayableTerrain(double worldX, double worldZ) const {
    return m_data ? m_data->isInside(worldX, worldZ) : false;
}

TerrainContactInfo ProceduralPlanetMap::getTerrainContactAt(const Vec3& worldPos, double probeRadius) const {
    TerrainContactInfo info;
    if (!m_data || !m_data->isValid()) return info;

    info.isInsidePlayableBounds = m_data->isInside(worldPos.x, worldPos.z);
    info.elevation = m_data->sampleHeight(worldPos.x, worldPos.z);
    info.surfaceNormal = m_data->sampleNormal(worldPos.x, worldPos.z);
    info.surfaceType = m_data->sampleMaterial(worldPos.x, worldPos.z);
    info.contactPointWorld = Vec3{worldPos.x, info.elevation, worldPos.z};
    info.penetrationDepth = std::max(0.0, (info.elevation + probeRadius) - worldPos.y);
    info.hasContact = (worldPos.y <= info.elevation + probeRadius + 0.02);
    info.slopeAngleRad = m_data->sampleSlopeRad(worldPos.x, worldPos.z);
    return info;
}

bool ProceduralPlanetMap::queryRayContact(const Vec3& rayOrigin, const Vec3& rayDir, double maxDistance, TerrainContactInfo& outContact) const {
    if (!m_data || !m_data->isValid() || maxDistance <= 0.0) return false;

    // Fast vertical ray optimization
    if (std::abs(rayDir.x) < 1e-4 && std::abs(rayDir.z) < 1e-4) {
        const double elev = m_data->sampleHeight(rayOrigin.x, rayOrigin.z);
        const double dist = (rayOrigin.y - elev);
        if (dist >= 0.0 && dist <= maxDistance) {
            outContact = getTerrainContactAt(Vec3{rayOrigin.x, elev, rayOrigin.z});
            return true;
        }
        return false;
    }

    // Step-march ray
    constexpr int kSteps = 32;
    const double stepDist = maxDistance / static_cast<double>(kSteps);
    Vec3 p = rayOrigin;
    for (int i = 0; i < kSteps; ++i) {
        p = vec3Add(p, vec3Scale(rayDir, stepDist));
        if (m_data->isInside(p.x, p.z)) {
            const double elev = m_data->sampleHeight(p.x, p.z);
            if (p.y <= elev) {
                outContact = getTerrainContactAt(Vec3{p.x, elev, p.z});
                return true;
            }
        }
    }
    return false;
}

} // namespace torquelab
