#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <algorithm>
#define CGLTF_IMPLEMENTATION
#include "cgltf.h"
#include <draco/compression/decode.h>

struct Vec3 { double x, y, z; };

int main() {
    cgltf_options options{};
    cgltf_data* data = nullptr;
    cgltf_result res = cgltf_parse_file(&options, "model/PerseveranceRover.glb", &data);
    if (res != cgltf_result_success) {
        std::cerr << "Failed to parse glb\n";
        return 1;
    }
    res = cgltf_load_buffers(&options, data, "model/PerseveranceRover.glb");
    if (res != cgltf_result_success) {
        std::cerr << "Failed to load buffers\n";
        return 1;
    }

    std::cout << "Successfully parsed glb. Nodes: " << data->nodes_count << ", Meshes: " << data->meshes_count << "\n";

    for (size_t n = 0; n < data->nodes_count; ++n) {
        const auto& node = data->nodes[n];
        std::string name = node.name ? node.name : "unnamed";
        if (name == "suspension" || name == "Wheels_objs" || name.find("arm") != std::string::npos || name.find("Body") != std::string::npos) {
            std::cout << "\n=== Node " << n << ": '" << name << "' ===\n";
            if (!node.mesh) {
                std::cout << "  No mesh\n";
                continue;
            }
            std::cout << "  Mesh: '" << (node.mesh->name ? node.mesh->name : "unnamed") << "' with " << node.mesh->primitives_count << " prims\n";
            for (size_t p = 0; p < node.mesh->primitives_count; ++p) {
                const auto& prim = node.mesh->primitives[p];
                std::string matName = (prim.material && prim.material->name) ? prim.material->name : "none";
                std::cout << "  -- Prim " << p << " (mat: " << matName << ") --\n";

                // Check draco
                if (prim.has_draco_mesh_compression) {
                    const auto& d = prim.draco_mesh_compression;
                    const uint8_t* buf = (const uint8_t*)d.buffer_view->buffer->data + d.buffer_view->offset;
                    size_t size = d.buffer_view->size;

                    draco::DecoderBuffer decBuf;
                    decBuf.Init((const char*)buf, size);
                    draco::Decoder decoder;
                    auto statusOrMesh = decoder.DecodeMeshFromBuffer(&decBuf);
                    if (!statusOrMesh.ok()) {
                        std::cout << "    Draco decode error\n";
                        continue;
                    }
                    auto mesh = std::move(statusOrMesh).value();
                    size_t numFaces = mesh->num_faces();
                    size_t numVerts = mesh->num_points();
                    std::cout << "    Draco mesh decoded: " << numVerts << " points, " << numFaces << " faces\n";

                    const draco::PointAttribute* posAttr = mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
                    if (!posAttr) continue;

                    Vec3 bMin{1e9, 1e9, 1e9}, bMax{-1e9, -1e9, -1e9};
                    for (draco::PointIndex pi(0); pi < numVerts; ++pi) {
                        float v[3];
                        posAttr->GetMappedValue(pi, v);
                        bMin.x = std::min(bMin.x, (double)v[0]);
                        bMin.y = std::min(bMin.y, (double)v[1]);
                        bMin.z = std::min(bMin.z, (double)v[2]);
                        bMax.x = std::max(bMax.x, (double)v[0]);
                        bMax.y = std::max(bMax.y, (double)v[1]);
                        bMax.z = std::max(bMax.z, (double)v[2]);
                    }
                    std::cout << "    Local Bounds: X=[" << bMin.x << " .. " << bMax.x << "], Y=["
                              << bMin.y << " .. " << bMax.y << "], Z=[" << bMin.z << " .. " << bMax.z << "]\n";
                }
            }
        }
    }

    cgltf_free(data);
    return 0;
}
