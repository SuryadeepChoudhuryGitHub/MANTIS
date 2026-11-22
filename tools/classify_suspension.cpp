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

struct Vec3 {
    double x, y, z;
};

struct Triangle {
    Vec3 v[3];
    int prim;
    std::string mat;
};

enum class PartType {
    DiffBar,
    DiffLink_L,
    DiffLink_R,
    Rocker_L,
    Rocker_R,
    Bogie_L,
    Bogie_R,
    Knuckle_FL,
    Knuckle_FR,
    Knuckle_RL,
    Knuckle_RR,
    Unknown
};

const char* partTypeName(PartType pt) {
    switch (pt) {
        case PartType::DiffBar: return "DiffBar";
        case PartType::DiffLink_L: return "DiffLink_L";
        case PartType::DiffLink_R: return "DiffLink_R";
        case PartType::Rocker_L: return "Rocker_L";
        case PartType::Rocker_R: return "Rocker_R";
        case PartType::Bogie_L: return "Bogie_L";
        case PartType::Bogie_R: return "Bogie_R";
        case PartType::Knuckle_FL: return "Knuckle_FL";
        case PartType::Knuckle_FR: return "Knuckle_FR";
        case PartType::Knuckle_RL: return "Knuckle_RL";
        case PartType::Knuckle_RR: return "Knuckle_RR";
        default: return "Unknown";
    }
}

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

    Vec3 nodeTrans{node->translation[0], node->translation[1], node->translation[2]};

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

    struct PartStats {
        Vec3 bMin{1e9, 1e9, 1e9};
        Vec3 bMax{-1e9, -1e9, -1e9};
        Vec3 sumPos{0, 0, 0};
        size_t totalTris{0};
    };

    std::map<PartType, PartStats> partStats;

    for (size_t c = 0; c < components.size(); ++c) {
        const auto& comp = components[c];
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

        PartType pt = PartType::Unknown;
        const bool isLeft = (cPos.x >= 0.0);

        if (bMin.y < 0.35 && bMax.y > 0.40 && std::abs(cPos.x) >= 0.55 && std::abs(cPos.x) <= 0.70 && cPos.z >= 0.15 && cPos.z <= 0.35) {
            pt = isLeft ? PartType::DiffLink_L : PartType::DiffLink_R;
        } else if (bMin.y >= 0.40 || (cPos.y >= 0.38 && std::abs(cPos.x) < 0.70)) {
            pt = PartType::DiffBar;
        } else if (cPos.z > 0.95 && std::abs(cPos.x) > 0.65) {
            pt = isLeft ? PartType::Knuckle_FL : PartType::Knuckle_FR;
        } else if (cPos.z < -0.95 && std::abs(cPos.x) > 0.65) {
            pt = isLeft ? PartType::Knuckle_RL : PartType::Knuckle_RR;
        } else {
            if (cPos.z <= -0.505 || (cPos.z < 0.10 && cPos.y < -0.10 && std::abs(cPos.x) > 0.85)) {
                pt = isLeft ? PartType::Bogie_L : PartType::Bogie_R;
            } else {
                pt = isLeft ? PartType::Rocker_L : PartType::Rocker_R;
            }
        }

        auto& s = partStats[pt];
        s.bMin.x = std::min(s.bMin.x, bMin.x);
        s.bMin.y = std::min(s.bMin.y, bMin.y);
        s.bMin.z = std::min(s.bMin.z, bMin.z);
        s.bMax.x = std::max(s.bMax.x, bMax.x);
        s.bMax.y = std::max(s.bMax.y, bMax.y);
        s.bMax.z = std::max(s.bMax.z, bMax.z);
        s.sumPos.x += sumPos.x;
        s.sumPos.y += sumPos.y;
        s.sumPos.z += sumPos.z;
        s.totalTris += comp.size();
    }

    std::cout << "\n================ PART STATS IN MODEL COORDINATES ================\n";
    std::cout << "Node Translation: (" << nodeTrans.x << ", " << nodeTrans.y << ", " << nodeTrans.z << ")\n\n";

    for (const auto& kv : partStats) {
        const auto& s = kv.second;
        double totalVerts = s.totalTris * 3.0;
        Vec3 localCenter{s.sumPos.x / totalVerts, s.sumPos.y / totalVerts, s.sumPos.z / totalVerts};
        Vec3 modelCenter{localCenter.x + nodeTrans.x, localCenter.y + nodeTrans.y, localCenter.z + nodeTrans.z};
        Vec3 modelMin{s.bMin.x + nodeTrans.x, s.bMin.y + nodeTrans.y, s.bMin.z + nodeTrans.z};
        Vec3 modelMax{s.bMax.x + nodeTrans.x, s.bMax.y + nodeTrans.y, s.bMax.z + nodeTrans.z};

        std::cout << partTypeName(kv.first) << " (" << s.totalTris << " tris):\n"
                  << "  Model BMin:   (" << modelMin.x << ", " << modelMin.y << ", " << modelMin.z << ")\n"
                  << "  Model BMax:   (" << modelMax.x << ", " << modelMax.y << ", " << modelMax.z << ")\n"
                  << "  Model Center: (" << modelCenter.x << ", " << modelCenter.y << ", " << modelCenter.z << ")\n\n";
    }

    cgltf_free(data);
    return 0;
}
