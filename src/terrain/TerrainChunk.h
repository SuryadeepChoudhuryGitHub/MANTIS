#pragma once
/**
 * PROJECT ARES — Terrain Chunk Representation.
 *
 * Encapsulates a spatially partitioned, bounded, and culled terrain mesh chunk
 * with stable world-space positioning and LOD support.
 */

#include <vector>
#include "physics/Math3D.h"
#include "renderer/Mesh.h"
#include "renderer/Shader.h"

namespace torquelab {

struct TerrainAABB {
    Vec3 minPoint{0.0, 0.0, 0.0};
    Vec3 maxPoint{0.0, 0.0, 0.0};

    bool contains(const Vec3& p) const {
        return (p.x >= minPoint.x && p.x <= maxPoint.x &&
                p.y >= minPoint.y && p.y <= maxPoint.y &&
                p.z >= minPoint.z && p.z <= maxPoint.z);
    }
};

struct TerrainChunk {
    int chunkX{0};                      // Grid column coordinate
    int chunkZ{0};                      // Grid row coordinate
    TerrainAABB worldBounds;            // World-space axis-aligned bounding box
    Vec3 centerWorld{0.0, 0.0, 0.0};    // Center point in world space
    Mat4 worldMatrix{Mat4::identity()}; // Immutable world transform (never parented to vehicle)
    Mesh mesh;                          // Uploaded OpenGL GPU geometry
    MaterialProps material;             // Material and surface texture properties
    bool isVisible{true};               // Visibility flag for frustum/distance culling

    TerrainChunk() = default;
    ~TerrainChunk() = default;

    TerrainChunk(TerrainChunk&&) noexcept = default;
    TerrainChunk& operator=(TerrainChunk&&) noexcept = default;

    TerrainChunk(const TerrainChunk&) = delete;
    TerrainChunk& operator=(const TerrainChunk&) = delete;
};

} // namespace torquelab
