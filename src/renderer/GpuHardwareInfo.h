#pragma once
/**
 * Torque Lab Native — GPU & Graphics Hardware Detection Subsystem.
 *
 * Detects active OpenGL rendering device, queries hardware vendor, GLSL version,
 * driver versions, VRAM capacity (via GL extensions and Windows DXGI),
 * classifies adapter architecture (Discrete/External High-Performance vs Integrated),
 * and reports system GPU topology across multi-GPU / switchable graphics systems.
 */

#include <string>
#include <vector>

namespace torquelab {

struct SystemGpuDevice {
    std::string name;
    size_t dedicatedVramMb{0};
    size_t sharedMemoryMb{0};
    bool isDiscrete{false};
    bool isCurrentOpenGLDevice{false};
};

struct GpuHardwareInfo {
    std::string vendor;
    std::string renderer;
    std::string glVersion;
    std::string glslVersion;
    std::string cleanName;
    std::string shortName;
    int dedicatedVramMb{0};
    int totalAvailableVramMb{0};
    bool isDiscreteGpu{false};
    bool hasSwitchableGraphics{false};
    std::string classification;
    std::vector<SystemGpuDevice> systemAdapters;

    /**
     * Query active OpenGL context and underlying host system (DXGI on Windows)
     * to populate full GPU hardware diagnostics.
     * Must be called AFTER OpenGL context creation and GLAD initialization.
     */
    static GpuHardwareInfo detect();

    /**
     * Format and print a clean diagnostic banner to standard output on startup.
     */
    void printStartupBanner() const;

    /**
     * Produce a multi-line formatted summary string suitable for UI tooltips or logs.
     */
    std::string getDetailedReport() const;
};

} // namespace torquelab
