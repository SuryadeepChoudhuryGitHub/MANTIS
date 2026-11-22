#include "renderer/TrackRendererGL.h"

namespace torquelab {

void TrackRendererGL::init(const std::vector<TrackBoxElement>& trackElements) {
    m_items.clear();
    createProceduralSurfaceMaterials();
    buildTrackMeshes(trackElements);
    buildReferenceMarkers();
    buildEnvironmentTerrain();
    updateSunTarget(Vec3{0.0, 0.0, 0.0});
}

void TrackRendererGL::createProceduralSurfaceMaterials() {
    auto setupMat = [&](SurfaceType type, unsigned int bgHex, unsigned int lineHex, float noiseAlpha, float repX, float repY, float rough, float metal, bool stripe = false) {
        const size_t idx = static_cast<size_t>(type);
        m_surfaceTextures[idx] = Texture2D::createProceduralGrid(bgHex, lineHex, noiseAlpha, stripe);

        MaterialProps mat{};
        mat.baseColor = Color3{1.0f, 1.0f, 1.0f};
        mat.textureId = m_surfaceTextures[idx].getId();
        mat.uvScaleX = repX;
        mat.uvScaleY = repY;
        mat.roughness = rough;
        mat.metalness = metal;
        mat.receiveShadows = true;
        mat.castShadows = false;
        m_surfaceMaterials[idx] = mat;
    };

    // Daytime PBR Outdoor Road & Planetary Surface Materials (Prominent Metric Ground Grid)
    setupMat(SurfaceType::DryAsphalt,   0x2e333d, 0x14181f, 0.05f,  6.0f,  6.0f, 0.74f, 0.08f, false);
    setupMat(SurfaceType::WetAsphalt,   0x1a2636, 0x0d141d, 0.04f,  4.0f,  4.0f, 0.14f, 0.25f, false);
    setupMat(SurfaceType::Gravel,       0x785a3e, 0x25170d, 0.08f,  4.0f,  4.0f, 0.92f, 0.02f, false);
    setupMat(SurfaceType::Ice,          0x86d9ff, 0x1e3a8a, 0.04f,  4.0f,  4.0f, 0.08f, 0.35f, false);
    setupMat(SurfaceType::Grass,        0x22462b, 0x0c1a10, 0.06f,  6.0f,  6.0f, 0.90f, 0.02f, false);
    setupMat(SurfaceType::RumbleStrip,  0xdc2626, 0xf8fafc, 0.02f,  1.0f, 25.0f, 0.65f, 0.05f, true);
    setupMat(SurfaceType::MarsDustFine, 0xd97c4e, 0x2a0f06, 0.08f,  2.0f,  2.0f, 0.88f, 0.00f, false);
    setupMat(SurfaceType::MarsRegolith, 0xb85d38, 0x220c05, 0.10f,  2.0f,  2.0f, 0.82f, 0.02f, false);
    setupMat(SurfaceType::MarsBedrock,  0x734839, 0x1a0904, 0.06f,  2.0f,  2.0f, 0.72f, 0.05f, false);
    setupMat(SurfaceType::MarsSandDune, 0xe89758, 0x2e1106, 0.12f,  2.0f,  2.0f, 0.90f, 0.00f, false);
}

void TrackRendererGL::buildTrackMeshes(const std::vector<TrackBoxElement>& elements) {
    for (const auto& el : elements) {
        RenderableTrackItem item{};
        item.mesh = Mesh::createBox(
            static_cast<float>(el.halfExtents.x * 2.0),
            static_cast<float>(el.halfExtents.y * 2.0),
            static_cast<float>(el.halfExtents.z * 2.0)
        );
        item.modelMatrix = Mat4::fromTRS(el.position, el.rotation);

        if (el.hasColorOverride) {
            MaterialProps mat{};
            mat.baseColor = Color3::fromHex(el.colorOverride);
            mat.roughness = (el.kind == TrackElementKind::Obstacle) ? 0.55f : 0.45f;
            mat.metalness = 0.15f;
            mat.receiveShadows = true;
            mat.castShadows = (el.kind == TrackElementKind::Obstacle ||
                               el.kind == TrackElementKind::Ramp ||
                               el.kind == TrackElementKind::Bump);
            item.material = mat;
        } else {
            item.material = m_surfaceMaterials[static_cast<size_t>(el.surfaceType)];
            item.material.castShadows = (el.kind == TrackElementKind::Obstacle ||
                                         el.kind == TrackElementKind::Ramp ||
                                         el.kind == TrackElementKind::Bump);
        }

        m_items.push_back(std::move(item));
    }
}

void TrackRendererGL::buildReferenceMarkers() {
    MaterialProps whiteLineMat{};
    whiteLineMat.baseColor = Color3::fromHex(0xe2e8f0);
    whiteLineMat.unlit = true;
    whiteLineMat.castShadows = false;
    whiteLineMat.receiveShadows = false;

    MaterialProps yellowLineMat{};
    yellowLineMat.baseColor = Color3::fromHex(0xfacc15);
    yellowLineMat.unlit = true;
    yellowLineMat.castShadows = false;
    yellowLineMat.receiveShadows = false;

    // 1. Start line at z = 2.5 m
    {
        RenderableTrackItem startLine{};
        startLine.mesh = Mesh::createPlaneXZ(18.0f, 0.8f);
        startLine.modelMatrix = Mat4::fromTRS(Vec3{0.0, 0.014, 2.5}, quatIdentity());
        startLine.material = whiteLineMat;
        m_items.push_back(std::move(startLine));
    }

    // 2. Center dashed lines along straightaway
    for (int z = -20; z <= 190; z += 10) {
        RenderableTrackItem dash{};
        dash.mesh = Mesh::createPlaneXZ(0.22f, 4.0f);
        dash.modelMatrix = Mat4::fromTRS(Vec3{0.0, 0.013, static_cast<double>(z)}, quatIdentity());
        dash.material = yellowLineMat;
        m_items.push_back(std::move(dash));
    }

    // 3. Distance marker posts every 25 m along main straight
    for (int dist = 25; dist <= 200; dist += 25) {
        const bool isMajor = (dist % 50 == 0);
        MaterialProps postMat{};
        postMat.baseColor = Color3::fromHex(isMajor ? 0x38bdf8 : 0x94a3b8);
        postMat.roughness = 0.4f;
        postMat.metalness = 0.15f;
        postMat.castShadows = true;
        postMat.receiveShadows = true;

        RenderableTrackItem postL{};
        postL.mesh = Mesh::createBox(0.35f, 1.8f, 0.35f);
        postL.modelMatrix = Mat4::fromTRS(Vec3{+12.2, 0.9, static_cast<double>(dist)}, quatIdentity());
        postL.material = postMat;
        m_items.push_back(std::move(postL));

        RenderableTrackItem postR{};
        postR.mesh = Mesh::createBox(0.35f, 1.8f, 0.35f);
        postR.modelMatrix = Mat4::fromTRS(Vec3{-12.2, 0.9, static_cast<double>(dist)}, quatIdentity());
        postR.material = postMat;
        m_items.push_back(std::move(postR));
    }

    // 4. Handling Circle reference ring on the north apron (x = 0, z = 240, radius = 28m)
    {
        RenderableTrackItem ring{};
        ring.mesh = Mesh::createRingXZ(27.6f, 28.2f, 64);
        ring.modelMatrix = Mat4::fromTRS(Vec3{0.0, 0.014, 240.0}, quatIdentity());
        ring.material = yellowLineMat;
        m_items.push_back(std::move(ring));
    }
}

void TrackRendererGL::clearItems() {
    m_items.clear();
    m_chunks.clear();
}

void TrackRendererGL::addRenderableItem(RenderableTrackItem item) {
    m_items.push_back(std::move(item));
}

void TrackRendererGL::clearTerrainChunks() {
    m_chunks.clear();
}

void TrackRendererGL::addTerrainChunk(TerrainChunk chunk) {
    m_chunks.push_back(std::move(chunk));
}

const MaterialProps& TrackRendererGL::getSurfaceMaterial(SurfaceType type) const {
    const size_t idx = static_cast<size_t>(type);
    if (idx < m_surfaceMaterials.size()) {
        return m_surfaceMaterials[idx];
    }
    return m_surfaceMaterials[0];
}

void TrackRendererGL::buildEnvironmentTerrain(Color3 groundColor, float innerCutoutHalfSize, float skirtElevation) {
    MaterialProps terrainMat{};
    terrainMat.baseColor = groundColor;
    terrainMat.uvScaleX = 240.0f;
    terrainMat.uvScaleY = 240.0f;
    terrainMat.roughness = 0.92f;
    terrainMat.metalness = 0.02f;
    terrainMat.receiveShadows = false;
    terrainMat.castShadows = false;

    if (innerCutoutHalfSize <= 0.0f) {
        RenderableTrackItem terrainSkirt{};
        terrainSkirt.mesh = Mesh::createPlaneXZ(3600.0f, 3600.0f);
        terrainSkirt.modelMatrix = Mat4::fromTRS(Vec3{0.0, static_cast<double>(skirtElevation), 0.0}, quatIdentity());
        terrainSkirt.material = terrainMat;
        m_items.push_back(std::move(terrainSkirt));
    } else {
        // Build 4 perimeter collar quads surrounding the inner cutout area [-innerCutoutHalfSize, innerCutoutHalfSize]
        // This completely prevents the flat skirt from cutting through high-relief DTM topography or covering valleys.
        const float outerHalfSize = 1800.0f;
        const float inHalf = innerCutoutHalfSize;
        const float collarDepth = outerHalfSize - inHalf; // e.g. 1800 - 512 = 1288m

        // North collar (+Z): spans full width, depth collarDepth
        RenderableTrackItem northSkirt{};
        northSkirt.mesh = Mesh::createPlaneXZ(outerHalfSize * 2.0f, collarDepth);
        northSkirt.modelMatrix = Mat4::fromTRS(Vec3{0.0, static_cast<double>(skirtElevation), inHalf + collarDepth * 0.5f}, quatIdentity());
        northSkirt.material = terrainMat;
        m_items.push_back(std::move(northSkirt));

        // South collar (-Z): spans full width, depth collarDepth
        RenderableTrackItem southSkirt{};
        southSkirt.mesh = Mesh::createPlaneXZ(outerHalfSize * 2.0f, collarDepth);
        southSkirt.modelMatrix = Mat4::fromTRS(Vec3{0.0, static_cast<double>(skirtElevation), -(inHalf + collarDepth * 0.5f)}, quatIdentity());
        southSkirt.material = terrainMat;
        m_items.push_back(std::move(southSkirt));

        // West collar (+X): spans between inner cutout Z range [-inHalf, inHalf]
        RenderableTrackItem westSkirt{};
        westSkirt.mesh = Mesh::createPlaneXZ(collarDepth, inHalf * 2.0f);
        westSkirt.modelMatrix = Mat4::fromTRS(Vec3{inHalf + collarDepth * 0.5f, static_cast<double>(skirtElevation), 0.0}, quatIdentity());
        westSkirt.material = terrainMat;
        m_items.push_back(std::move(westSkirt));

        // East collar (-X): spans between inner cutout Z range [-inHalf, inHalf]
        RenderableTrackItem eastSkirt{};
        eastSkirt.mesh = Mesh::createPlaneXZ(collarDepth, inHalf * 2.0f);
        eastSkirt.modelMatrix = Mat4::fromTRS(Vec3{-(inHalf + collarDepth * 0.5f), static_cast<double>(skirtElevation), 0.0}, quatIdentity());
        eastSkirt.material = terrainMat;
        m_items.push_back(std::move(eastSkirt));
    }
}

void TrackRendererGL::rebuildProvingGround(const std::vector<TrackBoxElement>& elements) {
    clearItems();
    buildTrackMeshes(elements);
    buildReferenceMarkers();
    buildEnvironmentTerrain(Color3::fromHex(0x22462b));
}

void TrackRendererGL::updateSunTarget(const Vec3& vehiclePos) {
    const Vec3 sunOffset{45.0, 75.0, 35.0};
    m_sunDirection = vec3Normalize(sunOffset);

    const Vec3 lightEye = vec3Add(vehiclePos, sunOffset);
    const Mat4 lightView = Mat4::lookAt(lightEye, vehiclePos, Vec3{0.0, 1.0, 0.0});
    const Mat4 lightProj = Mat4::ortho(-95.0f, 95.0f, -95.0f, 95.0f, 10.0f, 280.0f);
    m_lightSpaceMatrix = Mat4::multiply(lightProj, lightView);
}

float TrackRendererGL::getShadowTexelSize() const {
    if (m_quality == GraphicsQuality::Medium) {
        return 1.0f / 1024.0f;
    }
    return 1.0f / 2048.0f;
}

void TrackRendererGL::setQuality(GraphicsQuality quality, ShadowMapFBO& shadowFBO) {
    m_quality = quality;
    if (quality == GraphicsQuality::Maximum || quality == GraphicsQuality::High) {
        shadowFBO.resize(2048);
    } else if (quality == GraphicsQuality::Medium) {
        shadowFBO.resize(1024);
    }
}

void TrackRendererGL::renderShadowPass(Shader& depthShader) const {
    for (const auto& item : m_items) {
        if (!item.material.castShadows) continue;
        depthShader.setMat4("uModel", item.modelMatrix);
        item.mesh.draw();
    }
    for (const auto& chunk : m_chunks) {
        if (!chunk.material.castShadows || !chunk.isVisible) continue;
        depthShader.setMat4("uModel", chunk.worldMatrix);
        chunk.mesh.draw();
    }
}

void TrackRendererGL::renderMainPass(Shader& mainShader) const {
    for (const auto& item : m_items) {
        mainShader.applyMaterial(item.material);
        mainShader.setMat4("uModel", item.modelMatrix);
        item.mesh.draw();
    }
    for (const auto& chunk : m_chunks) {
        if (!chunk.isVisible) continue;
        mainShader.applyMaterial(chunk.material);
        mainShader.setMat4("uModel", chunk.worldMatrix);
        chunk.mesh.draw();
    }
}

} // namespace torquelab
