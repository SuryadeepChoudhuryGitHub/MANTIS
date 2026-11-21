#include <iostream>
#include "renderer/ModelLoaderGLB.h"

using namespace torquelab;

int main() {
    LoadedVehicleModel model{};
    if (!ModelLoaderGLB::loadFromFile("model/PerseveranceRover.glb", model, "", false)) {
        return 1;
    }

    std::cout << "\n=== Parts in Wheel 2 (Wheel_RL) ===\n";
    for (const auto& p : model.wheelParts[2]) {
        std::cout << "  Part: " << p.name << " localPos=(" << p.localPosition.x << ", " << p.localPosition.y << ", " << p.localPosition.z << ") Mat=" << p.material.name << "\n";
    }

    return 0;
}
