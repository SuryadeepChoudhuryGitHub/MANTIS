#pragma once
/**
 * PROJECT MANTIS — Centralized Planetary Environment System.
 *
 * Defines physical, atmospheric, and optical parameters for planetary bodies
 * (Earth, Mars, and future celestial environments).
 */

#include <string>
#include "renderer/Shader.h" // For Color3

namespace torquelab {

struct PlanetaryEnvironment {
    std::string id{"earth"};
    std::string displayName{"Earth"};
    double gravity{9.81};                   // Surface gravitational acceleration (m/s^2)
    double airDensity{1.225};               // Atmospheric density rho (kg/m^3)
    double ambientPressure{101325.0};       // Surface atmospheric pressure (Pa)
    double temperatureKelvin{288.15};       // Mean surface temperature (K)
    double solarIrradiance{1361.0};         // Solar flux at top of atmosphere (W/m^2)

    // Atmospheric Sky Dome & Scattering Parameters (Linear space)
    Color3 skyZenithColor{Color3::fromHex(0x0a28a6)};
    Color3 skyMidColor{Color3::fromHex(0x1f61d9)};
    Color3 skyHorizonColor{Color3::fromHex(0x619ef0)};
    Color3 groundHazeColor{Color3::fromHex(0x273628)};
    Color3 sunColor{Color3{1.9f, 1.86f, 1.75f}};
    float sunDiscScale{1.0f};               // Angular scale of solar disc relative to Earth
    float dustExtinction{0.0015f};          // Atmospheric haze / dust optical depth factor
};

PlanetaryEnvironment createEarthEnvironment();
PlanetaryEnvironment createMarsEnvironment();
PlanetaryEnvironment createMoonEnvironment();

} // namespace torquelab
