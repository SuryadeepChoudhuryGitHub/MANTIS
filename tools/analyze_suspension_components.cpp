#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#define CGLTF_IMPLEMENTATION
#include "cgltf.h"
#include <draco/compression/decode.h>

struct Vec3 {
    double x, y, z;
};

struct Triangle {
    Vec3 v[3];
    int prim;
    std::string mat;
};

void analyzeNode(cgltf_node* node, const std::string& nodeName) {
    std::cout << "\n=======================================================\n";
    std::cout << "ANALYSIS OF NODE: '" << nodeName << "'\n";
    std::cout << "=======================================================\n";

    if (!node->mesh) return;

    // Collect all triangles in node space
    std::vector<Triangle> triangles;

    for (size_t p = 0; p < node->mesh->primitives_count; ++p) {
        const auto& prim = node->mesh->primitives[p];
        std::string matName = (prim.material && prim.material->name) ? prim.material->name : "none";
        if (!prim.has_draco_mesh_compression) continue;

        const auto& d = prim.draco_mesh_compression;
        const uint8_t* buf = (const uint8_t*)d.buffer_view->buffer->data + d.buffer_view->offset;
        size_t size = d.buffer_view->size;

        draco::DecoderBuffer decBuf;
        decBuf.Init((const char*)buf, size);
        draco::Decoder decoder;
        auto statusOrMesh = decoder.DecodeMeshFromBuffer(&decBuf);
        if (!statusOrMesh.ok()) continue;
        auto mesh = std::move(statusOrMesh).value();

        const draco::PointAttribute* posAttr = mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
        if (!posAttr) continue;

        for (draco::FaceIndex fi(0); fi < mesh->num_faces(); ++fi) {
            const auto& face = mesh->face(fi);
            Triangle tri;
            tri.prim = (int)p;
            tri.mat = matName;
            for (int k = 0; k < 3; ++k) {
                float v[3];
                posAttr->GetMappedValue(face[k], v);
                tri.v[k] = Vec3{v[0], v[1], v[2]};
            }
            triangles.push_back(tri);
        }
    }

    std::cout << "Total Triangles: " << triangles.size() << "\n";

    // Cluster into connected components
    // We quantize vertex positions to group close vertices (tolerance 1mm = 0.001)
    auto keyOf = [](const Vec3& p) {
        long long qx = (long long)std::round(p.x * 1000.0);
        long long qy = (long long)std::round(p.y * 1000.0);
        long long qz = (long long)std::round(p.z * 1000.0);
        return std::to_string(qx) + "_" + std::to_string(qy) + "_" + std::to_string(qz);
    };

    std::map<std::string, std::vector<size_t>> vToTri;
    for (size_t i = 0; i < triangles.size(); ++i) {
        for (int k = 0; k < 3; ++k) {
            vToTri[keyOf(triangles[i].v[k])].push_back(i);
        }
    }

    std::vector<bool> visited(triangles.size(), false);
    std::vector<std::vector<size_t>> components;

    for (size_t i = 0; i < triangles.size(); ++i) {
        if (visited[i]) continue;
        std::vector<size_t> comp;
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            size_t curr = q.front();
            q.pop();
            comp.push_back(curr);

            for (int k = 0; k < 3; ++k) {
                const auto& neighbors = vToTri[keyOf(triangles[curr].v[k])];
                for (size_t nxt : neighbors) {
                    if (!visited[nxt]) {
                        visited[nxt] = true;
                        q.push(nxt);
                    }
                }
            }
        }
        components.push_back(comp);
    }

    // Sort components by triangle count descending
    std::sort(components.begin(), components.end(), [](const auto& a, const auto& b) {
        return a.size() > b.size();
    });

    std::cout << "Identified " << components.size() << " connected components:\n";
    for (size_t c = 0; c < components.size(); ++c) {
        const auto& comp = components[c];
        if (comp.size() < 10) continue; // skip tiny specs

        Vec3 bMin{1e9, 1e9, 1e9}, bMax{-1e9, -1e9, -1e9};
        Vec3 sumPos{0, 0, 0};
        double totalVerts = 0;
        std::map<std::string, int> matCounts;

        for (size_t triIdx : comp) {
            const auto& tri = triangles[triIdx];
            matCounts[tri.mat]++;
            for (int k = 0; k < 3; ++k) {
                bMin.x = std::min(bMin.x, tri.v[k].x);
                bMin.y = std::min(bMin.y, tri.v[k].y);
                bMin.z = std::min(bMin.z, tri.v[k].z);
                bMax.x = std::max(bMax.x, tri.v[k].x);
                bMax.y = std::max(bMax.y, tri.v[k].y);
                bMax.z = std::max(bMax.z, tri.v[k].z);
                sumPos.x += tri.v[k].x;
                sumPos.y += tri.v[k].y;
                sumPos.z += tri.v[k].z;
                totalVerts += 1.0;
            }
        }

        Vec3 centroid{sumPos.x / totalVerts, sumPos.y / totalVerts, sumPos.z / totalVerts};
        Vec3 extents{bMax.x - bMin.x, bMax.y - bMin.y, bMax.z - bMin.z};

        std::cout << "Component " << c << " (" << comp.size() << " tris): "
                  << "Centroid=(" << centroid.x << ", " << centroid.y << ", " << centroid.z << ") "
                  << "Extents=(" << extents.x << ", " << extents.y << ", " << extents.z << ") "
                  << "Bounds: X=[" << bMin.x << ".." << bMax.x << "] Y=[" << bMin.y << ".." << bMax.y << "] Z=[" << bMin.z << ".." << bMax.z << "]\n";
        std::cout << "  Materials: ";
        for (const auto& kv : matCounts) std::cout << kv.first << ":" << kv.second << " ";
        std::cout << "\n";
    }
}

int main() {
    cgltf_options options{};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&options, "model/PerseveranceRover.glb", &data) != cgltf_result_success) return 1;
    if (cgltf_load_buffers(&options, data, "model/PerseveranceRover.glb") != cgltf_result_success) return 1;

    for (size_t n = 0; n < data->nodes_count; ++n) {
        std::string name = data->nodes[n].name ? data->nodes[n].name : "";
        if (name == "Wheels_objs" || name == "suspension") {
            analyzeNode(&data->nodes[n], name);
        }
    }

    cgltf_free(data);
    return 0;
}
