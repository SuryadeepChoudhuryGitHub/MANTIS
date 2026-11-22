#include "physics/PlanetaryEnvironment.h"

namespace torquelab {

PlanetaryEnvironment createEarthEnvironment() {
    PlanetaryEnvironment env{};
    env.id = "earth";
    env.displayName = "Earth";
    env.gravity = 9.81;
    env.airDensity = 1.225;
    env.ambientPressure = 101325.0;
    env.temperatureKelvin = 288.15;
    env.solarIrradiance = 1361.0;
    env.skyZenithColor = Color3::fromHex(0x0a28a6);
    env.skyMidColor = Color3::fromHex(0x1f61d9);
    env.skyHorizonColor = Color3::fromHex(0x619ef0);
    env.groundHazeColor = Color3::fromHex(0x273628);
    env.sunColor = Color3{1.9f, 1.86f, 1.75f};
    env.sunDiscScale = 1.0f;
    env.dustExtinction = 0.0015f;
    return env;
}

PlanetaryEnvironment createMarsEnvironment() {
    PlanetaryEnvironment env{};
    env.id = "mars";
    env.displayName = "Mars";
    env.gravity = 3.71;
    env.airDensity = 0.020;
    env.ambientPressure = 636.0;
    env.temperatureKelvin = 210.0;
    env.solarIrradiance = 590.0;
    // Calibrated Martian sky palette:
    // Zenith: warm amber / butterscotch
    // Mid-sky: tawny reddish-buff
    // Horizon: luminous pinkish-tan dust haze
    // Ground haze: dark iron-oxide rust
    env.skyZenithColor = Color3{0.68f, 0.44f, 0.28f};
    env.skyMidColor = Color3{0.78f, 0.54f, 0.36f};
    env.skyHorizonColor = Color3{0.86f, 0.62f, 0.45f};
    env.groundHazeColor = Color3{0.55f, 0.28f, 0.16f};
    env.sunColor = Color3{1.35f, 1.25f, 1.15f};
    env.sunDiscScale = 0.66f;
    env.dustExtinction = 0.0028f;
    return env;
}

PlanetaryEnvironment createMoonEnvironment() {
    PlanetaryEnvironment env{};
    env.id = "moon";
    env.displayName = "Moon (Lunar Highlands)";
    env.gravity = 1.62;
    env.airDensity = 0.000;
    env.ambientPressure = 0.0;
    env.temperatureKelvin = 220.0;
    env.solarIrradiance = 1361.0;
    // Lunar vacuum sky: pure black space, brilliant solar disc
    env.skyZenithColor = Color3{0.001f, 0.001f, 0.002f};
    env.skyMidColor = Color3{0.002f, 0.002f, 0.004f};
    env.skyHorizonColor = Color3{0.008f, 0.008f, 0.012f};
    env.groundHazeColor = Color3{0.18f, 0.18f, 0.20f};
    env.sunColor = Color3{2.20f, 2.18f, 2.15f};
    env.sunDiscScale = 1.0f;
    env.dustExtinction = 0.0f;
    return env;
}

} // namespace torquelab
