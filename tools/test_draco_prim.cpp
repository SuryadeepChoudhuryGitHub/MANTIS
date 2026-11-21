#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>
#include <draco/compression/decode.h>

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

int main() {
    cgltf_options options{};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, "model/PerseveranceRover.glb", &data);
    if (result != cgltf_result_success) {
        std::cerr << "Failed to parse GLB" << std::endl;
        return 1;
    }
    result = cgltf_load_buffers(&options, data, "model/PerseveranceRover.glb");
    if (result != cgltf_result_success) {
        std::cerr << "Failed to load buffers" << std::endl;
        return 1;
    }

    std::cout << "Loaded GLB. Nodes: " << data->nodes_count << ", Meshes: " << data->meshes_count << std::endl;

    int dracoPrims = 0;
    for (size_t m = 0; m < data->meshes_count; ++m) {
        const cgltf_mesh& mesh = data->meshes[m];
        for (size_t p = 0; p < mesh.primitives_count; ++p) {
            const cgltf_primitive& prim = mesh.primitives[p];
            if (prim.has_draco_mesh_compression) {
                dracoPrims++;
                if (dracoPrims == 1) {
                    const cgltf_buffer_view* bv = prim.draco_mesh_compression.buffer_view;
                    const uint8_t* bufferData = cgltf_buffer_view_data(bv);
                    size_t bufferSize = bv->size;
                    std::cout << "First Draco primitive: size = " << bufferSize << " bytes" << std::endl;

                    draco::DecoderBuffer decBuf;
                    decBuf.Init(reinterpret_cast<const char*>(bufferData), bufferSize);
                    draco::Decoder decoder;
                    auto statusOrMesh = decoder.DecodeMeshFromBuffer(&decBuf);
                    if (!statusOrMesh.ok()) {
                        std::cerr << "Draco decode failed: " << statusOrMesh.status().error_msg() << std::endl;
                    } else {
                        auto dMesh = std::move(statusOrMesh).value();
                        std::cout << "Decoded successfully! Faces: " << dMesh->num_faces() 
                                  << ", Points: " << dMesh->num_points() << std::endl;
                        const draco::PointAttribute* posAttr = dMesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
                        if (posAttr) {
                            float minP[3]{1e9, 1e9, 1e9}, maxP[3]{-1e9, -1e9, -1e9};
                            for (draco::PointIndex pt(0); pt < dMesh->num_points(); ++pt) {
                                float val[3];
                                posAttr->GetMappedValue(pt, val);
                                for (int i = 0; i < 3; ++i) {
                                    if (val[i] < minP[i]) minP[i] = val[i];
                                    if (val[i] > maxP[i]) maxP[i] = val[i];
                                }
                            }
                            std::cout << "Position range: X=[" << minP[0] << ", " << maxP[0] << "], "
                                      << "Y=[" << minP[1] << ", " << maxP[1] << "], "
                                      << "Z=[" << minP[2] << ", " << maxP[2] << "]" << std::endl;
                        }
                    }
                }
            }
        }
    }
    std::cout << "Total Draco primitives: " << dracoPrims << std::endl;
    cgltf_free(data);
    return 0;
}
