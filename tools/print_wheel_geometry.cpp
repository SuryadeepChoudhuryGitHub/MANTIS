#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <draco/compression/decode.h>

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

int main() {
    cgltf_options options{};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, "model/PerseveranceRover.glb", &data);
    if (result != cgltf_result_success) return 1;
    result = cgltf_load_buffers(&options, data, "model/PerseveranceRover.glb");
    if (result != cgltf_result_success) return 1;

    for (size_t n = 0; n < data->nodes_count; ++n) {
        const cgltf_node* node = &data->nodes[n];
        if (!node->mesh || !node->name) continue;
        std::string name = node->name;
        if (name == "Wheels_objs" || name == "suspension") {
            std::cout << "\n=== Node " << name << " ===" << std::endl;
            const cgltf_mesh* mesh = node->mesh;
            for (size_t p = 0; p < mesh->primitives_count; ++p) {
                const cgltf_primitive& prim = mesh->primitives[p];
                if (prim.has_draco_mesh_compression) {
                    const cgltf_buffer_view* bv = prim.draco_mesh_compression.buffer_view;
                    const uint8_t* bData = cgltf_buffer_view_data(bv);
                    draco::DecoderBuffer decBuf;
                    decBuf.Init(reinterpret_cast<const char*>(bData), bv->size);
                    draco::Decoder decoder;
                    auto statusOr = decoder.DecodeMeshFromBuffer(&decBuf);
                    if (statusOr.ok()) {
                        auto dMesh = std::move(statusOr).value();
                        const draco::PointAttribute* posAttr = dMesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
                        std::vector<std::array<float, 3>> pts;
                        for (draco::PointIndex pt(0); pt < dMesh->num_points(); ++pt) {
                            std::array<float, 3> pos;
                            posAttr->GetMappedValue(pt, pos.data());
                            pts.push_back(pos);
                        }
                        std::cout << "  Prim " << p << ": points=" << pts.size() << ", faces=" << dMesh->num_faces() << std::endl;
                        // Cluster pts by side (X < 0 vs X > 0) and Z
                        for (int side = 0; side < 2; ++side) {
                            std::string sName = (side == 0) ? "Left (X < 0)" : "Right (X > 0)";
                            float xSign = (side == 0) ? -1.0f : 1.0f;
                            // Front (Z > 0.4), Mid (-0.4 <= Z <= 0.4), Rear (Z < -0.4)
                            for (int zSeg = 0; zSeg < 3; ++zSeg) {
                                std::string zName = (zSeg == 0) ? "Front (Z > 0.4)" : (zSeg == 1 ? "Mid (-0.4..0.4)" : "Rear (Z < -0.4)");
                                float minP[3]{1e9, 1e9, 1e9}, maxP[3]{-1e9, -1e9, -1e9};
                                double sum[3]{0, 0, 0};
                                int cnt = 0;
                                for (const auto& pt : pts) {
                                    if ((side == 0 && pt[0] < -0.2f) || (side == 1 && pt[0] > 0.2f)) {
                                        bool zMatch = (zSeg == 0 && pt[2] > 0.4f) ||
                                                      (zSeg == 1 && pt[2] >= -0.4f && pt[2] <= 0.4f) ||
                                                      (zSeg == 2 && pt[2] < -0.4f);
                                        if (zMatch) {
                                            cnt++;
                                            for (int i = 0; i < 3; ++i) {
                                                minP[i] = std::min(minP[i], pt[i]);
                                                maxP[i] = std::max(maxP[i], pt[i]);
                                                sum[i] += pt[i];
                                            }
                                        }
                                    }
                                }
                                if (cnt > 0) {
                                    std::printf("    [%s - %s]: cnt=%d, center=(%.3f, %.3f, %.3f), bounds X=[%.3f, %.3f], Y=[%.3f, %.3f], Z=[%.3f, %.3f]\n",
                                                sName.c_str(), zName.c_str(), cnt,
                                                sum[0]/cnt, sum[1]/cnt, sum[2]/cnt,
                                                minP[0], maxP[0], minP[1], maxP[1], minP[2], maxP[2]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    cgltf_free(data);
    return 0;
}
