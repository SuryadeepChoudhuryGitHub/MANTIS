#include "renderer/Mesh.h"
#include <cmath>
#include <utility>

namespace torquelab {

Mesh::~Mesh() {
    release();
}

Mesh::Mesh(Mesh&& other) noexcept
    : m_vao(std::exchange(other.m_vao, 0))
    , m_vbo(std::exchange(other.m_vbo, 0))
    , m_ebo(std::exchange(other.m_ebo, 0))
    , m_indexCount(std::exchange(other.m_indexCount, 0))
{}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        release();
        m_vao = std::exchange(other.m_vao, 0);
        m_vbo = std::exchange(other.m_vbo, 0);
        m_ebo = std::exchange(other.m_ebo, 0);
        m_indexCount = std::exchange(other.m_indexCount, 0);
    }
    return *this;
}

void Mesh::release() {
    if (m_ebo != 0) {
        glDeleteBuffers(1, &m_ebo);
        m_ebo = 0;
    }
    if (m_vbo != 0) {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }
    if (m_vao != 0) {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }
    m_indexCount = 0;
}

void Mesh::upload(const std::vector<VertexPNTU>& vertices, const std::vector<unsigned int>& indices) {
    release();

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexPNTU)),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
        indices.data(),
        GL_STATIC_DRAW
    );

    // aPos (location = 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPNTU), reinterpret_cast<void*>(0));

    // aNormal (location = 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPNTU), reinterpret_cast<void*>(3 * sizeof(float)));

    // aTangent (location = 3)
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPNTU), reinterpret_cast<void*>(6 * sizeof(float)));

    // aTexCoord (location = 2)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPNTU), reinterpret_cast<void*>(10 * sizeof(float)));

    glBindVertexArray(0);
    m_indexCount = static_cast<GLsizei>(indices.size());
}

void Mesh::computeTangents(std::vector<VertexPNTU>& vertices, const std::vector<unsigned int>& indices) {
    if (vertices.empty() || indices.empty()) return;

    std::vector<Vec3> tan1(vertices.size(), Vec3{0, 0, 0});
    std::vector<Vec3> tan2(vertices.size(), Vec3{0, 0, 0});

    const size_t numTris = indices.size() / 3;
    for (size_t i = 0; i < numTris; ++i) {
        const unsigned int i1 = indices[i * 3 + 0];
        const unsigned int i2 = indices[i * 3 + 1];
        const unsigned int i3 = indices[i * 3 + 2];

        if (i1 >= vertices.size() || i2 >= vertices.size() || i3 >= vertices.size()) continue;

        const auto& v1 = vertices[i1];
        const auto& v2 = vertices[i2];
        const auto& v3 = vertices[i3];

        const float x1 = v2.px - v1.px;
        const float x2 = v3.px - v1.px;
        const float y1 = v2.py - v1.py;
        const float y2 = v3.py - v1.py;
        const float z1 = v2.pz - v1.pz;
        const float z2 = v3.pz - v1.pz;

        const float s1 = v2.u - v1.u;
        const float s2 = v3.u - v1.u;
        const float t1 = v2.v - v1.v;
        const float t2 = v3.v - v1.v;

        const float r = (s1 * t2 - s2 * t1);
        const float invR = (std::abs(r) > 1e-6f) ? (1.0f / r) : 0.0f;

        const Vec3 sdir{
            (t2 * x1 - t1 * x2) * invR,
            (t2 * y1 - t1 * y2) * invR,
            (t2 * z1 - t1 * z2) * invR
        };
        const Vec3 tdir{
            (s1 * x2 - s2 * x1) * invR,
            (s1 * y2 - s2 * y1) * invR,
            (s1 * z2 - s2 * z1) * invR
        };

        tan1[i1] = vec3Add(tan1[i1], sdir);
        tan1[i2] = vec3Add(tan1[i2], sdir);
        tan1[i3] = vec3Add(tan1[i3], sdir);

        tan2[i1] = vec3Add(tan2[i1], tdir);
        tan2[i2] = vec3Add(tan2[i2], tdir);
        tan2[i3] = vec3Add(tan2[i3], tdir);
    }

    for (size_t a = 0; a < vertices.size(); ++a) {
        auto& v = vertices[a];
        const Vec3 n{v.nx, v.ny, v.nz};
        const Vec3 t = tan1[a];

        if (vec3LengthSq(t) > 1e-6) {
            // Gram-Schmidt orthogonalize
            Vec3 tangent = vec3Normalize(vec3Sub(t, vec3Scale(n, vec3Dot(n, t))));
            const double handedness = (vec3Dot(vec3Cross(n, t), tan2[a]) < 0.0) ? -1.0 : 1.0;

            v.tx = static_cast<float>(tangent.x);
            v.ty = static_cast<float>(tangent.y);
            v.tz = static_cast<float>(tangent.z);
            v.tw = static_cast<float>(handedness);
        } else {
            Vec3 c1 = vec3Cross(n, Vec3{0, 0, 1});
            Vec3 c2 = vec3Cross(n, Vec3{0, 1, 0});
            Vec3 tangent = (vec3LengthSq(c1) > vec3LengthSq(c2)) ? vec3Normalize(c1) : vec3Normalize(c2);
            v.tx = static_cast<float>(tangent.x);
            v.ty = static_cast<float>(tangent.y);
            v.tz = static_cast<float>(tangent.z);
            v.tw = 1.0f;
        }
    }
}

void Mesh::draw() const {
    if (m_vao == 0 || m_indexCount == 0) {
        return;
    }
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

Mesh Mesh::createBox(float width, float height, float depth) {
    const float hx = width * 0.5f;
    const float hy = height * 0.5f;
    const float hz = depth * 0.5f;

    std::vector<VertexPNTU> verts{
        // +Z Front face
        {-hx, -hy,  hz,  0,  0,  1,  1, 0, 0, 1,  0, 0},
        { hx, -hy,  hz,  0,  0,  1,  1, 0, 0, 1,  1, 0},
        { hx,  hy,  hz,  0,  0,  1,  1, 0, 0, 1,  1, 1},
        {-hx,  hy,  hz,  0,  0,  1,  1, 0, 0, 1,  0, 1},
        // -Z Back face
        { hx, -hy, -hz,  0,  0, -1, -1, 0, 0, 1,  0, 0},
        {-hx, -hy, -hz,  0,  0, -1, -1, 0, 0, 1,  1, 0},
        {-hx,  hy, -hz,  0,  0, -1, -1, 0, 0, 1,  1, 1},
        { hx,  hy, -hz,  0,  0, -1, -1, 0, 0, 1,  0, 1},
        // +Y Top face
        {-hx,  hy,  hz,  0,  1,  0,  1, 0, 0, 1,  0, 0},
        { hx,  hy,  hz,  0,  1,  0,  1, 0, 0, 1,  1, 0},
        { hx,  hy, -hz,  0,  1,  0,  1, 0, 0, 1,  1, 1},
        {-hx,  hy, -hz,  0,  1,  0,  1, 0, 0, 1,  0, 1},
        // -Y Bottom face
        {-hx, -hy, -hz,  0, -1,  0,  1, 0, 0, 1,  0, 0},
        { hx, -hy, -hz,  0, -1,  0,  1, 0, 0, 1,  1, 0},
        { hx, -hy,  hz,  0, -1,  0,  1, 0, 0, 1,  1, 1},
        {-hx, -hy,  hz,  0, -1,  0,  1, 0, 0, 1,  0, 1},
        // +X Left face
        { hx, -hy,  hz,  1,  0,  0,  0, 0, -1, 1,  0, 0},
        { hx, -hy, -hz,  1,  0,  0,  0, 0, -1, 1,  1, 0},
        { hx,  hy, -hz,  1,  0,  0,  0, 0, -1, 1,  1, 1},
        { hx,  hy,  hz,  1,  0,  0,  0, 0, -1, 1,  0, 1},
        // -X Right face
        {-hx, -hy, -hz, -1,  0,  0,  0, 0,  1, 1,  0, 0},
        {-hx, -hy,  hz, -1,  0,  0,  0, 0,  1, 1,  1, 0},
        {-hx,  hy,  hz, -1,  0,  0,  0, 0,  1, 1,  1, 1},
        {-hx,  hy, -hz, -1,  0,  0,  0, 0,  1, 1,  0, 1}
    };

    std::vector<unsigned int> indices;
    indices.reserve(36);
    for (unsigned int f = 0; f < 6; ++f) {
        const unsigned int base = f * 4;
        indices.push_back(base + 0);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
        indices.push_back(base + 0);
        indices.push_back(base + 2);
        indices.push_back(base + 3);
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createCylinderY(float radiusTop, float radiusBottom, float height, int radialSegments) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    const float halfH = height * 0.5f;
    const int segs = std::max(6, radialSegments);

    // Side barrel
    for (int i = 0; i <= segs; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(segs);
        const float theta = u * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);

        verts.push_back(VertexPNU{radiusTop * ct, +halfH, radiusTop * st, ct, 0.0f, st, u, 1.0f});
        verts.push_back(VertexPNU{radiusBottom * ct, -halfH, radiusBottom * st, ct, 0.0f, st, u, 0.0f});
    }

    for (int i = 0; i < segs; ++i) {
        const unsigned int b = static_cast<unsigned int>(i * 2);
        indices.push_back(b + 0);
        indices.push_back(b + 1);
        indices.push_back(b + 2);
        indices.push_back(b + 1);
        indices.push_back(b + 3);
        indices.push_back(b + 2);
    }

    // Top cap (+Y)
    const unsigned int topCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{0.0f, +halfH, 0.0f, 0.0f, 1.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);
        verts.push_back(VertexPNU{radiusTop * ct, +halfH, radiusTop * st, 0.0f, 1.0f, 0.0f, ct * 0.5f + 0.5f, st * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(topCenterIdx);
        indices.push_back(topCenterIdx + 1 + static_cast<unsigned int>(i + 1));
        indices.push_back(topCenterIdx + 1 + static_cast<unsigned int>(i));
    }

    // Bottom cap (-Y)
    const unsigned int botCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{0.0f, -halfH, 0.0f, 0.0f, -1.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);
        verts.push_back(VertexPNU{radiusBottom * ct, -halfH, radiusBottom * st, 0.0f, -1.0f, 0.0f, ct * 0.5f + 0.5f, st * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(botCenterIdx);
        indices.push_back(botCenterIdx + 1 + static_cast<unsigned int>(i));
        indices.push_back(botCenterIdx + 1 + static_cast<unsigned int>(i + 1));
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createCylinderX(float radiusTop, float radiusBottom, float width, int radialSegments) {
    // Cylinder aligned along local X axis (for tires, rims, and brake rotors)
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    const float halfW = width * 0.5f;
    const int segs = std::max(8, radialSegments);

    // Barrel around X axis
    for (int i = 0; i <= segs; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(segs);
        const float theta = u * 2.0f * PI_F;
        const float cy = std::cos(theta);
        const float sz = std::sin(theta);

        verts.push_back(VertexPNU{+halfW, radiusTop * cy, radiusTop * sz, 0.0f, cy, sz, u, 1.0f});
        verts.push_back(VertexPNU{-halfW, radiusBottom * cy, radiusBottom * sz, 0.0f, cy, sz, u, 0.0f});
    }

    for (int i = 0; i < segs; ++i) {
        const unsigned int b = static_cast<unsigned int>(i * 2);
        indices.push_back(b + 0);
        indices.push_back(b + 1);
        indices.push_back(b + 2);
        indices.push_back(b + 1);
        indices.push_back(b + 3);
        indices.push_back(b + 2);
    }

    // +X cap
    const unsigned int posCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{+halfW, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float cy = std::cos(theta);
        const float sz = std::sin(theta);
        verts.push_back(VertexPNU{+halfW, radiusTop * cy, radiusTop * sz, 1.0f, 0.0f, 0.0f, cy * 0.5f + 0.5f, sz * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(posCenterIdx);
        indices.push_back(posCenterIdx + 1 + static_cast<unsigned int>(i));
        indices.push_back(posCenterIdx + 1 + static_cast<unsigned int>(i + 1));
    }

    // -X cap
    const unsigned int negCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{-halfW, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float cy = std::cos(theta);
        const float sz = std::sin(theta);
        verts.push_back(VertexPNU{-halfW, radiusBottom * cy, radiusBottom * sz, -1.0f, 0.0f, 0.0f, cy * 0.5f + 0.5f, sz * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(negCenterIdx);
        indices.push_back(negCenterIdx + 1 + static_cast<unsigned int>(i + 1));
        indices.push_back(negCenterIdx + 1 + static_cast<unsigned int>(i));
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createPneumaticTireMesh(
    float outerRadius,
    float rimRadius,
    float width,
    int radialSegments,
    int sidewallRings,
    int treadSegments
) {
    std::vector<VertexPNTU> verts;
    std::vector<unsigned int> indices;

    const float halfW = width * 0.5f;
    const int nRadial = std::max(16, radialSegments);
    const int nSide = std::max(3, sidewallRings);
    const int nTread = std::max(4, treadSegments);

    struct ProfilePoint {
        float x;
        float r;
        float nx;
        float nr;
    };
    std::vector<ProfilePoint> profile;

    // A. Inboard sidewall (from rim bead x = -halfW*0.75, r = rimRadius up to shoulder)
    for (int s = 0; s <= nSide; ++s) {
        const float t = static_cast<float>(s) / static_cast<float>(nSide);
        const float r = rimRadius + t * (outerRadius * 0.95f - rimRadius);
        const float bulge = std::sin(t * PI_F) * (halfW * 0.28f);
        const float x = -halfW * (0.75f + t * 0.17f) - bulge;
        const float nx = -std::cos(t * PI_F * 0.85f);
        const float nr = std::sin(t * PI_F * 0.85f);
        const float len = std::hypot(nx, nr);
        profile.push_back(ProfilePoint{x, r, nx / len, nr / len});
    }

    // B. Tread crown band (from inboard shoulder to outboard shoulder)
    for (int c = 1; c < nTread; ++c) {
        const float t = static_cast<float>(c) / static_cast<float>(nTread);
        const float uPos = -1.0f + 2.0f * t;
        const float x = uPos * (halfW * 0.92f);
        const float crownDrop = 0.004f * (uPos * uPos);
        const float r = outerRadius - crownDrop;
        const float nx = -0.15f * uPos;
        const float nr = 0.988f;
        const float len = std::hypot(nx, nr);
        profile.push_back(ProfilePoint{x, r, nx / len, nr / len});
    }

    // C. Outboard sidewall (from shoulder down to rim bead x = +halfW*0.75, r = rimRadius)
    for (int s = 0; s <= nSide; ++s) {
        const float t = static_cast<float>(s) / static_cast<float>(nSide);
        const float r = (outerRadius * 0.95f) - t * (outerRadius * 0.95f - rimRadius);
        const float tFromBead = 1.0f - t;
        const float bulge = std::sin(tFromBead * PI_F) * (halfW * 0.28f);
        const float x = halfW * (0.92f - t * 0.17f) + bulge;
        const float nx = std::cos(tFromBead * PI_F * 0.85f);
        const float nr = std::sin(tFromBead * PI_F * 0.85f);
        const float len = std::hypot(nx, nr);
        profile.push_back(ProfilePoint{x, r, nx / len, nr / len});
    }

    const int nProf = static_cast<int>(profile.size());

    // 2. Revolve profile around local X axis
    verts.reserve(static_cast<size_t>((nRadial + 1) * nProf));
    for (int i = 0; i <= nRadial; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(nRadial);
        const float phi = u * 2.0f * PI_F;
        const float cPhi = std::cos(phi);
        const float sPhi = std::sin(phi);

        for (int k = 0; k < nProf; ++k) {
            const auto& pt = profile[static_cast<size_t>(k)];
            const float px = pt.x;
            const float py = pt.r * cPhi;
            const float pz = pt.r * sPhi;

            const float nx = pt.nx;
            const float ny = pt.nr * cPhi;
            const float nz = pt.nr * sPhi;

            const float v = static_cast<float>(k) / static_cast<float>(nProf - 1);
            verts.push_back(VertexPNTU{px, py, pz, nx, ny, nz, 0.0f, 0.0f, 0.0f, 1.0f, u, v});
        }
    }

    // 3. Connect quad grid with indices
    indices.reserve(static_cast<size_t>(nRadial * (nProf - 1) * 6));
    for (int i = 0; i < nRadial; ++i) {
        const int ring0 = i * nProf;
        const int ring1 = (i + 1) * nProf;

        for (int k = 0; k < nProf - 1; ++k) {
            const unsigned int i00 = static_cast<unsigned int>(ring0 + k);
            const unsigned int i01 = static_cast<unsigned int>(ring0 + k + 1);
            const unsigned int i10 = static_cast<unsigned int>(ring1 + k);
            const unsigned int i11 = static_cast<unsigned int>(ring1 + k + 1);

            indices.push_back(i00);
            indices.push_back(i10);
            indices.push_back(i01);

            indices.push_back(i10);
            indices.push_back(i11);
            indices.push_back(i01);
        }
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

namespace {

inline uint32_t rockHash32(uint32_t a) {
    a = (a ^ 61) ^ (a >> 16);
    a = a + (a << 3);
    a = a ^ (a >> 4);
    a = a * 0x27d4eb2d;
    a = a ^ (a >> 15);
    return a;
}

inline double rockHashFloat(int seed, int index) {
    const uint32_t h = rockHash32(static_cast<uint32_t>(seed * 1103515245 + index * 12345));
    return static_cast<double>(h & 0xFFFF) / 65535.0;
}

} // namespace

std::vector<Vec3> Mesh::getLowPolyRockHullPoints(float halfWidth, float halfHeight, float halfDepth, int variantSeed) {
    constexpr double t = 1.618033988749895; // Golden ratio
    const double rawVerts[12][3] = {
        {-1.0,  t,   0.0}, { 1.0,  t,   0.0}, {-1.0, -t,   0.0}, { 1.0, -t,   0.0},
        { 0.0, -1.0,  t }, { 0.0,  1.0,  t }, { 0.0, -1.0, -t }, { 0.0,  1.0, -t },
        {  t,   0.0, -1.0}, {  t,   0.0,  1.0}, { -t,   0.0, -1.0}, { -t,   0.0,  1.0}
    };

    const int archetype = std::abs(variantSeed) % 5;
    std::vector<Vec3> points(12);

    for (int i = 0; i < 12; ++i) {
        Vec3 v{rawVerts[i][0], rawVerts[i][1], rawVerts[i][2]};
        v = vec3Normalize(v);

        const double rnd = rockHashFloat(variantSeed, i);
        double radialDisplacement = 0.78 + 0.44 * rnd; // [0.78, 1.22]

        // Morphological archetypes
        if (archetype == 0) {
            // Angular basalt boulder: sharper peaks and asymmetric cleavage
            if (v.y > 0.2) v.y *= 1.25;
            if (v.x * v.z > 0.0) radialDisplacement *= 1.15;
        } else if (archetype == 1) {
            // Tabular basalt slab: flattened vertically, wider perimeter
            v.y *= 0.55;
            v.x *= 1.25;
            v.z *= 1.25;
        } else if (archetype == 2) {
            // Wind-eroded martian ventifact: sharp keel ridge on one side, steep leeward slope
            const double keel = (v.x + v.z) * 0.7071;
            if (keel > 0.3) {
                radialDisplacement *= 1.30;
                v.y += 0.20;
            } else if (keel < -0.3) {
                radialDisplacement *= 0.80;
            }
        } else if (archetype == 3) {
            // Weathered rounded basalt stone: gentler displacement
            radialDisplacement = 0.88 + 0.24 * rnd;
        } else {
            // Fractured wedge rock: planar shear cut on negative X
            if (v.x < -0.2) {
                v.x = -0.2 + (v.x + 0.2) * 0.4;
            } else {
                radialDisplacement *= 1.18;
            }
        }

        // Flatter underside for flush stable seating into regolith
        if (v.y < 0.0) {
            v.y *= 0.65;
        }

        points[i] = Vec3{
            v.x * radialDisplacement * static_cast<double>(halfWidth),
            v.y * radialDisplacement * static_cast<double>(halfHeight),
            v.z * radialDisplacement * static_cast<double>(halfDepth)
        };
    }

    return points;
}

Mesh Mesh::createLowPolyRock(float halfWidth, float halfHeight, float halfDepth, int variantSeed) {
    const auto hull = getLowPolyRockHullPoints(halfWidth, halfHeight, halfDepth, variantSeed);

    constexpr int faces[20][3] = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
        {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
        {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
        {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
    };

    std::vector<VertexPNTU> verts;
    verts.reserve(60);
    std::vector<unsigned int> indices;
    indices.reserve(60);

    for (int f = 0; f < 20; ++f) {
        Vec3 vA = hull[static_cast<size_t>(faces[f][0])];
        Vec3 vB = hull[static_cast<size_t>(faces[f][1])];
        Vec3 vC = hull[static_cast<size_t>(faces[f][2])];

        // Exact face normal for crisp faceted low-poly rock lighting
        Vec3 edge1 = vec3Sub(vB, vA);
        Vec3 edge2 = vec3Sub(vC, vA);
        Vec3 faceNorm = vec3Cross(edge1, edge2);

        // Ensure outward pointing normal and strictly CCW winding from outside
        const Vec3 centroid = vec3Scale(vec3Add(vA, vec3Add(vB, vC)), 1.0 / 3.0);
        if (vec3Dot(faceNorm, centroid) < 0.0) {
            std::swap(vB, vC);
            edge1 = vec3Sub(vB, vA);
            edge2 = vec3Sub(vC, vA);
            faceNorm = vec3Cross(edge1, edge2);
        }

        const double nLen = vec3Length(faceNorm);
        if (nLen > 1e-7) {
            faceNorm = vec3Scale(faceNorm, 1.0 / nLen);
        } else {
            faceNorm = Vec3{0.0, 1.0, 0.0};
        }

        const float fnx = static_cast<float>(faceNorm.x);
        const float fny = static_cast<float>(faceNorm.y);
        const float fnz = static_cast<float>(faceNorm.z);

        const unsigned int baseIdx = static_cast<unsigned int>(verts.size());

        verts.push_back(VertexPNTU{
            static_cast<float>(vA.x), static_cast<float>(vA.y), static_cast<float>(vA.z),
            fnx, fny, fnz, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f
        });
        verts.push_back(VertexPNTU{
            static_cast<float>(vB.x), static_cast<float>(vB.y), static_cast<float>(vB.z),
            fnx, fny, fnz, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f
        });
        verts.push_back(VertexPNTU{
            static_cast<float>(vC.x), static_cast<float>(vC.y), static_cast<float>(vC.z),
            fnx, fny, fnz, 1.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f
        });

        indices.push_back(baseIdx + 0);
        indices.push_back(baseIdx + 1);
        indices.push_back(baseIdx + 2);
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createTorusZ(float radius, float tubeRadius, int radialSegments, int tubularSegments) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    for (int j = 0; j <= radialSegments; ++j) {
        const float v = static_cast<float>(j) / static_cast<float>(radialSegments) * 2.0f * PI_F;
        const float cv = std::cos(v);
        const float sv = std::sin(v);

        for (int i = 0; i <= tubularSegments; ++i) {
            const float u = static_cast<float>(i) / static_cast<float>(tubularSegments) * 2.0f * PI_F;
            const float cu = std::cos(u);
            const float su = std::sin(u);

            const float px = (radius + tubeRadius * cv) * cu;
            const float py = (radius + tubeRadius * cv) * su;
            const float pz = tubeRadius * sv;

            const float nx = cv * cu;
            const float ny = cv * su;
            const float nz = sv;

            verts.push_back(VertexPNU{
                px, py, pz,
                nx, ny, nz,
                static_cast<float>(i) / static_cast<float>(tubularSegments),
                static_cast<float>(j) / static_cast<float>(radialSegments)
            });
        }
    }

    for (int j = 1; j <= radialSegments; ++j) {
        for (int i = 1; i <= tubularSegments; ++i) {
            const unsigned int a = static_cast<unsigned int>((tubularSegments + 1) * j + i - 1);
            const unsigned int b = static_cast<unsigned int>((tubularSegments + 1) * (j - 1) + i - 1);
            const unsigned int c = static_cast<unsigned int>((tubularSegments + 1) * (j - 1) + i);
            const unsigned int d = static_cast<unsigned int>((tubularSegments + 1) * j + i);

            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(d);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(d);
        }
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createSphere(float radius, int rings, int sectors) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    for (int r = 0; r <= rings; ++r) {
        const float v = static_cast<float>(r) / static_cast<float>(rings);
        const float phi = v * PI_F;
        const float sp = std::sin(phi);
        const float cp = std::cos(phi);

        for (int s = 0; s <= sectors; ++s) {
            const float u = static_cast<float>(s) / static_cast<float>(sectors);
            const float theta = u * 2.0f * PI_F;
            const float nx = sp * std::cos(theta);
            const float ny = cp;
            const float nz = sp * std::sin(theta);

            verts.push_back(VertexPNU{
                radius * nx, radius * ny, radius * nz,
                nx, ny, nz,
                u, v
            });
        }
    }

    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            const unsigned int cur = static_cast<unsigned int>(r * (sectors + 1) + s);
            const unsigned int next = static_cast<unsigned int>((r + 1) * (sectors + 1) + s);

            indices.push_back(cur);
            indices.push_back(next);
            indices.push_back(cur + 1);

            indices.push_back(cur + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createPlaneXZ(float width, float depth) {
    const float hx = width * 0.5f;
    const float hz = depth * 0.5f;

    std::vector<VertexPNTU> verts{
        {-hx, 0.0f,  hz, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
        { hx, 0.0f,  hz, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f},
        { hx, 0.0f, -hz, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f},
        {-hx, 0.0f, -hz, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f}
    };
    const std::vector<unsigned int> indices{0, 1, 2, 0, 2, 3};

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createRingXZ(float innerRadius, float outerRadius, int segments) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= segments; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(segments);
        const float theta = u * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);

        verts.push_back(VertexPNU{innerRadius * ct, 0.0f, innerRadius * st, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, u, 0.0f});
        verts.push_back(VertexPNU{outerRadius * ct, 0.0f, outerRadius * st, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, u, 1.0f});
    }

    for (int i = 0; i < segments; ++i) {
        const unsigned int b = static_cast<unsigned int>(i * 2);
        indices.push_back(b + 0);
        indices.push_back(b + 1);
        indices.push_back(b + 2);
        indices.push_back(b + 1);
        indices.push_back(b + 3);
        indices.push_back(b + 2);
    }

    computeTangents(verts, indices);
    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

} // namespace torquelab
