#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <queue>
#include <algorithm>
#define CGLTF_IMPLEMENTATION
#include "cgltf.h"
#include <draco/compression/decode.h>

struct Vec3 { double x, y, z; };
struct Triangle { Vec3 v[3]; };

int main() {
    cgltf_options options{};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&options, "model/PerseveranceRover.glb", &data) != cgltf_result_success) return 1;
    if (cgltf_load_buffers(&options, data, "model/PerseveranceRover.glb") != cgltf_result_success) return 1;

    cgltf_node* node = nullptr;
    for (size_t n = 0; n < data->nodes_count; ++n) {
        if (data->nodes[n].name && std::string(data->nodes[n].name) == "suspension") {
            node = &data->nodes[n];
            break;
        }
    }
    if (!node || !node->mesh) return 1;

    std::vector<Triangle> triangles;
    for (size_t p = 0; p < node->mesh->primitives_count; ++p) {
        const auto& prim = node->mesh->primitives[p];
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
            for (int k = 0; k < 3; ++k) {
                float v[3];
                posAttr->GetMappedValue(face[k], v);
                tri.v[k] = Vec3{v[0], v[1], v[2]};
            }
            triangles.push_back(tri);
        }
    }

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
                for (size_t nxt : vToTri[keyOf(triangles[curr].v[k])]) {
                    if (!visited[nxt]) {
                        visited[nxt] = true;
                        q.push(nxt);
                    }
                }
            }
        }
        components.push_back(comp);
    }

    std::map<std::string, int> roleCounts;
    for (const auto& comp : components) {
        Vec3 bMin{1e9, 1e9, 1e9}, bMax{-1e9, -1e9, -1e9};
        Vec3 sumPos{0, 0, 0};
        for (size_t triIdx : comp) {
            for (int k = 0; k < 3; ++k) {
                const auto& pt = triangles[triIdx].v[k];
                bMin.x = std::min(bMin.x, pt.x);
                bMin.y = std::min(bMin.y, pt.y);
                bMin.z = std::min(bMin.z, pt.z);
                bMax.x = std::max(bMax.x, pt.x);
                bMax.y = std::max(bMax.y, pt.y);
                bMax.z = std::max(bMax.z, pt.z);
                sumPos.x += pt.x;
                sumPos.y += pt.y;
                sumPos.z += pt.z;
            }
        }
        double totalVerts = comp.size() * 3.0;
        Vec3 cPos{sumPos.x / totalVerts, sumPos.y / totalVerts, sumPos.z / totalVerts};
        const bool isLeft = (cPos.x >= 0.0);

        std::string role;
        if (bMin.y < 0.35 && bMax.y > 0.40 && std::abs(cPos.x) >= 0.55 && std::abs(cPos.x) <= 0.70 && cPos.z >= 0.15 && cPos.z <= 0.35) {
            role = isLeft ? "Rover_DiffLink_L" : "Rover_DiffLink_R";
        } else if (bMin.y >= 0.40 || (cPos.y >= 0.38 && std::abs(cPos.x) < 0.70)) {
            role = "Rover_DiffBar";
        } else if (cPos.z > 0.95 && std::abs(cPos.x) > 0.65) {
            role = isLeft ? "Rover_Knuckle_FL" : "Rover_Knuckle_FR";
        } else if (cPos.z < -0.95 && std::abs(cPos.x) > 0.65) {
            role = isLeft ? "Rover_Knuckle_RL" : "Rover_Knuckle_RR";
        } else {
            if (cPos.z <= -0.505 || (cPos.z < 0.10 && cPos.y < -0.10 && std::abs(cPos.x) > 0.85)) {
                role = isLeft ? "Rover_Bogie_L" : "Rover_Bogie_R";
            } else {
                role = isLeft ? "Rover_Rocker_L" : "Rover_Rocker_R";
            }
        }
        roleCounts[role]++;
    }

    std::cout << "Components count by role in local node space:\n";
    for (const auto& kv : roleCounts) {
        std::cout << "  " << kv.first << ": " << kv.second << " components\n";
    }

    cgltf_free(data);
    return 0;
}
