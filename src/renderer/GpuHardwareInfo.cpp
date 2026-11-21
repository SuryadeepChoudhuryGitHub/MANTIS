#include "renderer/GpuHardwareInfo.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <glad/glad.h>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <dxgi.h>
#endif

namespace torquelab {

namespace {

std::string toUpperStr(std::string_view s) {
    std::string result;
    result.reserve(s.size());
    for (char c : s) {
        result.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
    return result;
}

std::string cleanGpuName(const std::string& raw) {
    std::string name = raw;
    // Strip trailing hardware bus tags like /PCIe/SSE2, /SSE2, etc.
    const size_t slashPos = name.find('/');
    if (slashPos != std::string::npos) {
        name = name.substr(0, slashPos);
    }
    // Trim trailing whitespace
    while (!name.empty() && std::isspace(static_cast<unsigned char>(name.back()))) {
        name.pop_back();
    }
    return name;
}

std::string createShortGpuName(const std::string& clean) {
    std::string s = clean;
    // Remove redundant corporate prefixes for tight HUD displays
    const std::string pfxNvidia = "NVIDIA GeForce ";
    const std::string pfxAmd = "AMD Radeon ";
    if (s.rfind(pfxNvidia, 0) == 0) {
        return s.substr(pfxNvidia.length());
    }
    if (s.rfind("NVIDIA ", 0) == 0) {
        return s.substr(7);
    }
    if (s.rfind(pfxAmd, 0) == 0) {
        return s.substr(pfxAmd.length());
    }
    if (s.rfind("AMD ", 0) == 0) {
        return s.substr(4);
    }
    return s;
}

#if defined(_WIN32)
void enumerateDxgiAdapters(std::vector<SystemGpuDevice>& outAdapters) {
    HMODULE hDxgi = LoadLibraryA("dxgi.dll");
    if (!hDxgi) return;

    typedef HRESULT (WINAPI *PFN_CreateDXGIFactory1)(REFIID riid, void **ppFactory);
    FARPROC rawProc = GetProcAddress(hDxgi, "CreateDXGIFactory1");
    auto pfnCreateDXGIFactory1 = reinterpret_cast<PFN_CreateDXGIFactory1>(reinterpret_cast<void*>(rawProc));

    if (pfnCreateDXGIFactory1) {
        IDXGIFactory1* pFactory = nullptr;
        if (SUCCEEDED(pfnCreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&pFactory)))) {
            IDXGIAdapter1* pAdapter = nullptr;
            for (UINT i = 0; pFactory->EnumAdapters1(i, &pAdapter) != DXGI_ERROR_NOT_FOUND; ++i) {
                DXGI_ADAPTER_DESC1 desc{};
                if (SUCCEEDED(pAdapter->GetDesc1(&desc))) {
                    if (!(desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE)) {
                        char nameBuf[256] = {0};
                        WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, nameBuf, sizeof(nameBuf) - 1, nullptr, nullptr);
                        
                        SystemGpuDevice dev;
                        dev.name = nameBuf;
                        dev.dedicatedVramMb = static_cast<size_t>(desc.DedicatedVideoMemory / (1024 * 1024));
                        dev.sharedMemoryMb = static_cast<size_t>(desc.SharedSystemMemory / (1024 * 1024));

                        const std::string upper = toUpperStr(dev.name);
                        const bool isNvidia = (upper.find("NVIDIA") != std::string::npos || upper.find("GEFORCE") != std::string::npos);
                        const bool isAmd = (upper.find("RADEON") != std::string::npos && upper.find("GRAPHICS") == std::string::npos);
                        const bool isArc = (upper.find("ARC") != std::string::npos);
                        dev.isDiscrete = (isNvidia || isAmd || isArc || dev.dedicatedVramMb >= 1024);

                        outAdapters.push_back(std::move(dev));
                    }
                }
                pAdapter->Release();
            }
            pFactory->Release();
        }
    }

    FreeLibrary(hDxgi);
}
#endif

typedef void (APIENTRY *PFN_glGetIntegerv)(GLenum pname, GLint *data);
PFN_glGetIntegerv getGlGetIntegervProc() {
    typedef void* (*PFN_glfwGetProcAddress)(const char* procname);
    HMODULE hGlfw = GetModuleHandleA(nullptr);
    auto pfnGlfw = reinterpret_cast<PFN_glfwGetProcAddress>(reinterpret_cast<void*>(GetProcAddress(hGlfw, "glfwGetProcAddress")));
    if (pfnGlfw) {
        void* proc = pfnGlfw("glGetIntegerv");
        if (proc) return reinterpret_cast<PFN_glGetIntegerv>(proc);
    }
    HMODULE hGL = GetModuleHandleA("opengl32.dll");
    if (hGL) {
        return reinterpret_cast<PFN_glGetIntegerv>(reinterpret_cast<void*>(GetProcAddress(hGL, "glGetIntegerv")));
    }
    return nullptr;
}

} // namespace

GpuHardwareInfo GpuHardwareInfo::detect() {
    GpuHardwareInfo info;

    const char* rawVendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    const char* rawRenderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    const char* rawVersion = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    const char* rawGlsl = reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION));

    info.vendor = rawVendor ? rawVendor : "Unknown Vendor";
    info.renderer = rawRenderer ? rawRenderer : "Unknown Renderer";
    info.glVersion = rawVersion ? rawVersion : "Unknown Version";
    info.glslVersion = rawGlsl ? rawGlsl : "Unknown GLSL";

    info.cleanName = cleanGpuName(info.renderer);
    info.shortName = createShortGpuName(info.cleanName);

    const std::string upperRenderer = toUpperStr(info.renderer);
    const std::string upperVendor = toUpperStr(info.vendor);

    // Architectural Classification
    const bool isNvidia = (upperVendor.find("NVIDIA") != std::string::npos ||
                           upperRenderer.find("GEFORCE") != std::string::npos ||
                           upperRenderer.find("RTX") != std::string::npos ||
                           upperRenderer.find("GTX") != std::string::npos ||
                           upperRenderer.find("QUADRO") != std::string::npos ||
                           upperRenderer.find("TITAN") != std::string::npos);

    const bool isAmdDiscrete = ((upperVendor.find("ATI") != std::string::npos || upperVendor.find("AMD") != std::string::npos) &&
                                (upperRenderer.find("RX") != std::string::npos ||
                                 upperRenderer.find("PRO") != std::string::npos ||
                                 upperRenderer.find("XT") != std::string::npos));

    const bool isIntelArc = (upperRenderer.find("ARC") != std::string::npos);

    info.isDiscreteGpu = (isNvidia || isAmdDiscrete || isIntelArc);
    info.classification = info.isDiscreteGpu ?
        "High-Performance Discrete / External GPU" :
        "Integrated GPU (Power Saving)";

    // Dedicated & Available Video Memory via OpenGL extensions
    // 1. NVIDIA GL_NVX_gpu_memory_info
    constexpr GLenum GL_GPU_MEMORY_INFO_DEDICATED_VIDMEM_NVX = 0x9047;
    constexpr GLenum GL_GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX = 0x9048;
    auto pfnGetIntegerv = getGlGetIntegervProc();

    if (pfnGetIntegerv && isNvidia) {
        GLint dedicatedKb = 0;
        pfnGetIntegerv(GL_GPU_MEMORY_INFO_DEDICATED_VIDMEM_NVX, &dedicatedKb);
        if (dedicatedKb > 0) {
            info.dedicatedVramMb = dedicatedKb / 1024;
        }

        GLint totalKb = 0;
        pfnGetIntegerv(GL_GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX, &totalKb);
        if (totalKb > 0) {
            info.totalAvailableVramMb = totalKb / 1024;
            if (info.dedicatedVramMb <= 0) {
                info.dedicatedVramMb = info.totalAvailableVramMb;
            }
        }
    }

    // 2. AMD GL_ATI_meminfo
    constexpr GLenum GL_VBO_FREE_MEMORY_ATI = 0x87FB;
    if (pfnGetIntegerv && (upperVendor.find("ATI") != std::string::npos || upperVendor.find("AMD") != std::string::npos)) {
        GLint memInfo[4] = {0};
        pfnGetIntegerv(GL_VBO_FREE_MEMORY_ATI, memInfo);
        if (memInfo[0] > 0) {
            info.totalAvailableVramMb = memInfo[0] / 1024;
        }
    }

#if defined(_WIN32)
    // Enumerate system hardware adapters via Windows DXGI
    enumerateDxgiAdapters(info.systemAdapters);

    int discreteCount = 0;
    int integratedCount = 0;
    for (auto& adapter : info.systemAdapters) {
        if (adapter.isDiscrete) {
            discreteCount++;
        } else {
            integratedCount++;
        }

        // Determine if this DXGI adapter matches the active OpenGL renderer
        const std::string upperAdapt = toUpperStr(adapter.name);
        bool match = false;
        if (isNvidia && upperAdapt.find("NVIDIA") != std::string::npos) {
            match = true;
        } else if ((upperRenderer.find("INTEL") != std::string::npos || upperVendor.find("INTEL") != std::string::npos) &&
                   upperAdapt.find("INTEL") != std::string::npos) {
            match = true;
        } else if ((upperRenderer.find("AMD") != std::string::npos || upperRenderer.find("RADEON") != std::string::npos) &&
                   (upperAdapt.find("AMD") != std::string::npos || upperAdapt.find("RADEON") != std::string::npos)) {
            match = true;
        }

        if (match) {
            adapter.isCurrentOpenGLDevice = true;
            if (info.dedicatedVramMb <= 0 && adapter.dedicatedVramMb > 0) {
                info.dedicatedVramMb = static_cast<int>(adapter.dedicatedVramMb);
            }
        }
    }

    info.hasSwitchableGraphics = (discreteCount > 0 && integratedCount > 0) || (info.systemAdapters.size() > 1);
#endif

    return info;
}

void GpuHardwareInfo::printStartupBanner() const {
    std::printf("====================================================================\n");
    std::printf(" TORQUE LAB NATIVE — GPU & GRAPHICS HARDWARE DETECTION\n");
    std::printf("====================================================================\n");
    std::printf(" Active Device:       %s\n", cleanName.c_str());
    std::printf(" Hardware Vendor:     %s\n", vendor.c_str());
    std::printf(" OpenGL Core Version: %s\n", glVersion.c_str());
    std::printf(" GLSL Version:        %s\n", glslVersion.c_str());
    if (dedicatedVramMb > 0) {
        std::printf(" Dedicated VRAM:      %d MB\n", dedicatedVramMb);
    }
    std::printf(" Classification:      %s [%s]\n",
                classification.c_str(),
                isDiscreteGpu ? "OPTIMAL FOR 6-DOF SIMULATION" : "INTEGRATED FALLBACK");
    std::printf(" Switchable Graphics: %s\n",
                hasSwitchableGraphics ? "Dual GPU Detected (NVIDIA Optimus / AMD High-Performance Active)" : "Single GPU System");

    if (!systemAdapters.empty()) {
        std::printf(" System GPU Topology:\n");
        for (size_t i = 0; i < systemAdapters.size(); ++i) {
            const auto& a = systemAdapters[i];
            std::printf("   [%zu] %s (Dedicated: %zu MB, Shared: %zu MB) [%s]%s\n",
                        i + 1,
                        a.name.c_str(),
                        a.dedicatedVramMb,
                        a.sharedMemoryMb,
                        a.isDiscrete ? "Discrete / External" : "Integrated",
                        a.isCurrentOpenGLDevice ? " <== ACTIVE OPENGL DEVICE" : "");
        }
    }
    std::printf("====================================================================\n");
}

std::string GpuHardwareInfo::getDetailedReport() const {
    std::ostringstream ss;
    ss << "Active GPU: " << cleanName << "\n";
    ss << "Vendor: " << vendor << "\n";
    ss << "Driver: " << glVersion << "\n";
    if (dedicatedVramMb > 0) {
        ss << "Dedicated VRAM: " << dedicatedVramMb << " MB\n";
    }
    ss << "Classification: " << classification << "\n";
    ss << "High-Performance Profiles: NvOptimusEnablement=1, AmdPowerXpress=1\n";
    if (!systemAdapters.empty()) {
        ss << "\nSystem GPU Adapters:\n";
        for (size_t i = 0; i < systemAdapters.size(); ++i) {
            const auto& a = systemAdapters[i];
            ss << " [" << (i + 1) << "] " << a.name
               << " (" << a.dedicatedVramMb << " MB)"
               << (a.isCurrentOpenGLDevice ? " [ACTIVE]" : "") << "\n";
        }
    }
    return ss.str();
}

} // namespace torquelab
