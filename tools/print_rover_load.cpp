#include <iostream>
#include "renderer/ModelLoaderGLB.h"

using namespace torquelab;

int main() {
    LoadedVehicleModel model{};
    if (!ModelLoaderGLB::loadFromFile("model/PerseveranceRover.glb", model, "", false)) {
        std::cerr << "Failed to load model" << std::endl;
        return 1;
    }

    std::cout << "\n=== Perseverance Model Load Diagnostics ===\n";
    std::cout << "isRover: " << (model.isRover ? "true" : "false") << "\n";
    std::cout << "Chassis AABB Min: (" << model.aabbMin.x << ", " << model.aabbMin.y << ", " << model.aabbMin.z << ")\n";
    std::cout << "Chassis AABB Max: (" << model.aabbMax.x << ", " << model.aabbMax.y << ", " << model.aabbMax.z << ")\n";
    std::cout << "Chassis Extents:  (" << model.aabbMax.x - model.aabbMin.x << ", " << model.aabbMax.y - model.aabbMin.y << ", " << model.aabbMax.z - model.aabbMin.z << ")\n";
    std::cout << "Chassis Center Offset: (" << model.chassisCenterOffset.x << ", " << model.chassisCenterOffset.y << ", " << model.chassisCenterOffset.z << ")\n";

    std::cout << "\n=== Wheel Centers and Radii ===\n";
    for (int i = 0; i < 6; ++i) {
        std::cout << "Wheel " << i << ": Center=(" << model.modelWheelCenters[i].x << ", " << model.modelWheelCenters[i].y << ", " << model.modelWheelCenters[i].z << ") "
                  << "Radius=" << model.modelWheelRadii[i] << " Width=" << model.modelWheelWidths[i] << " Identified=" << model.wheelsIdentified[i]
                  << " PartsCount=" << model.wheelParts[i].size() << "\n";
    }

    std::cout << "\n=== Rover Suspension Parts Count ===\n";
    std::cout << "DiffBar Parts:       " << model.roverDiffBarParts.size() << "\n";
    std::cout << "DiffLinkLeft Parts:  " << model.roverDiffLinkPartsLeft.size() << "\n";
    std::cout << "DiffLinkRight Parts: " << model.roverDiffLinkPartsRight.size() << "\n";
    std::cout << "RockerLeft Parts:    " << model.roverRockerPartsLeft.size() << "\n";
    std::cout << "RockerRight Parts:   " << model.roverRockerPartsRight.size() << "\n";
    std::cout << "BogieLeft Parts:     " << model.roverBogiePartsLeft.size() << "\n";
    std::cout << "BogieRight Parts:    " << model.roverBogiePartsRight.size() << "\n";
    for (int k = 0; k < 4; ++k) {
        std::cout << "Knuckle " << k << " Parts:     " << model.roverKnuckleParts[k].size() << "\n";
    }

    return 0;
}
