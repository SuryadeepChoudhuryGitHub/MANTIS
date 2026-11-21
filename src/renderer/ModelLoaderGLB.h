#pragma once
/**
 * Torque Lab Native — glTF 2.0 / GLB Binary Vehicle Model Loader.
 *
 * Implements a robust, production-grade 3D model importer for Blender-exported vehicles.
 * Supports:
 * - Direct loading of .glb (binary) and .gltf files via cgltf
 * - Node hierarchy extraction, TRS local transforms, and scene graphs
 * - Extraction of mesh primitives into OpenGL VAO/VBO/EBO geometry (VertexPNU)
 * - Intelligent semantic role mapping (Chassis, Wheel_FL, Wheel_FR, Wheel_RL, Wheel_RR,
 *   BrakeCalipers, SteeringWheel, and studio plane exclusions)
 * - Optional manifest JSON support for user-defined per-car name mappings
 * - Physically Based Material (PBR) metallic/roughness and transparent glass classification
 * - Automatic wheel pivot computation and chassis alignment to physics center of mass
 */

#include <array>
#include <memory>
#include <string>
#include <vector>
#include "physics/CollisionMeshGeometry.h"
#include "physics/Math3D.h"
#include "renderer/Mesh.h"
#include "renderer/Shader.h"
#include "renderer/Texture2D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

enum class ModelPartRole {
    Chassis,         // Main chassis / body shell / doors / hood / trim
    Wheel_FL,        // Front-Left wheel assembly (rotates + steers)
    Wheel_FR,        // Front-Right wheel assembly (rotates + steers)
    Wheel_RL,        // Rear-Left wheel assembly (rotates with drive torque)
    Wheel_RR,        // Rear-Right wheel assembly (rotates with drive torque)
    Wheel_ML,        // Mid-Left wheel assembly (Mars 6-wheel rover)
    Wheel_MR,        // Mid-Right wheel assembly (Mars 6-wheel rover)
    Wheel_All,       // Merged wheel primitive containing all wheels (auto-partitioned)
    BrakeCaliper_FL, // Front-Left brake caliper (steers, does not rotate)
    BrakeCaliper_FR, // Front-Right brake caliper (steers, does not rotate)
    BrakeCaliper_RL, // Rear-Left brake caliper (non-steering, does not rotate)
    BrakeCaliper_RR, // Rear-Right brake caliper (non-steering, does not rotate)
    SteeringWheel,   // Interior steering wheel (rotates around steering axis)
    Ignore           // Studio ground planes, lights, cameras, etc.
};

struct LoadedModelPart {
    std::string name;
    Mesh mesh;
    RawMeshGeometry cpuGeometry; // Retained CPU vertices and indices in local node space
    Mat4 localBindMatrix{Mat4::identity()};
    Vec3 localPosition{0.0, 0.0, 0.0};
    Quat localRotation{0.0, 0.0, 0.0, 1.0};
    Vec3 localScale{1.0, 1.0, 1.0};
    MaterialProps material;
    ModelPartRole role{ModelPartRole::Chassis};
    bool isBrakeLight{false};
    bool isReverseLight{false};
    bool isTransparent{false};
    bool isTire{false};

    // For wheel components: offset relative to the calculated wheel rotation pivot
    Vec3 offsetFromWheelCenter{0.0, 0.0, 0.0};
    Quat wheelLocalRotation{0.0, 0.0, 0.0, 1.0};
    Vec3 wheelLocalScale{1.0, 1.0, 1.0};
    Mat4 subPartLocalMatrix{Mat4::identity()}; // Pre-computed relative transform: T(-pivot) * localBindMatrix
};

struct LoadedVehicleModel {
    std::string modelName{"Mars Rover"};
    std::string filePath;
    bool loadedSuccessfully{false};

    // Retained CPU Collision Geometry (Model Space)
    RawMeshGeometry chassisCollisionMesh; // Accumulated chassis body triangles in local model space
    std::array<RawMeshGeometry, 6> wheelMeshes; // Wheels 0=FL, 1=FR, 2=RL, 3=RR, 4=ML, 5=MR
    std::array<RawMeshGeometry, 6> tireMeshes;  // Dedicated pure rubber tyre geometry

    // Identified wheel pivot centers in the model's coordinate frame
    std::array<Vec3, 6> modelWheelCenters{
        Vec3{ 1.144, 0.00,  1.130}, // FL
        Vec3{-1.144, 0.00,  1.130}, // FR
        Vec3{ 1.144, 0.00, -1.130}, // RL
        Vec3{-1.144, 0.00, -1.130}, // RR
        Vec3{ 1.266, 0.00, -0.055}, // ML
        Vec3{-1.266, 0.00, -0.055}  // MR
    };
    std::array<double, 6> modelWheelRadii{0.2625, 0.2625, 0.2625, 0.2625, 0.2625, 0.2625};
    std::array<double, 6> modelWheelWidths{0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
    std::array<bool, 6> wheelsIdentified{false, false, false, false, false, false};

    // Model bounding box (overall and chassis only)
    Vec3 aabbMin{0.0, 0.0, 0.0};
    Vec3 aabbMax{0.0, 0.0, 0.0};
    Vec3 chassisCenterOffset{0.0, 0.0, 0.0}; // Offset between model origin and physics CoM

    // Parts grouped by role for fast runtime rendering
    std::vector<LoadedModelPart> chassisParts;
    std::array<std::vector<LoadedModelPart>, 6> wheelParts; // 0=FL, 1=FR, 2=RL, 3=RR, 4=ML, 5=MR
    std::array<std::vector<LoadedModelPart>, 6> caliperParts;
    std::vector<LoadedModelPart> steeringWheelParts;

    size_t totalVertices{0};
    size_t totalTriangles{0};
    std::vector<std::string> diagnosticWarnings;

    // Retained unique materials for visual diagnostics and inspection
    std::vector<MaterialProps> uniqueMaterials;

    // OpenGL texture resources owned by this model
    std::vector<std::unique_ptr<Texture2D>> loadedTextures;
};

class ModelLoaderGLB {
public:
    /**
     * Loads a 3D vehicle model from a .glb or .gltf file and binds nodes to semantic vehicle roles.
     * Searches relative to project paths and supports an optional JSON manifest.
     */
    static bool loadFromFile(
        const std::string& glbFilePath,
        LoadedVehicleModel& outModel,
        const std::string& manifestFilePath = "",
        bool loadGpuResources = true
    );

    /**
     * Attempts to find a vehicle model file across standard project asset directories.
     */
    static std::string resolveModelPath(const std::string& relPath);
};

} // namespace torquelab
