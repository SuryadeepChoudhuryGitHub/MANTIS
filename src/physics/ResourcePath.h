#pragma once
/**
 * PROJECT ARES — Canonical Resource & Asset Path Resolver.
 *
 * Provides deterministic, working-directory-independent asset resolution
 * for shaders, 3D rover models, terrain heightfields, textures, and settings.
 * Resolves paths relative to current_path(), parent directories, and the
 * running executable's module directory on Windows.
 */

#include <filesystem>
#include <string>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace torquelab {

class ResourcePath {
public:
    static std::string resolve(const std::string& relPath) {
        namespace fs = std::filesystem;
        if (relPath.empty()) return relPath;

        const fs::path inputPath(relPath);
        if (inputPath.is_absolute() && fs::exists(inputPath)) {
            return inputPath.string();
        }

        std::vector<fs::path> searchRoots;
        std::error_code ec;

        // 1. Current working directory and parent paths
        const fs::path cwd = fs::current_path(ec);
        if (!ec) {
            searchRoots.push_back(cwd);
            searchRoots.push_back(cwd / "..");
            searchRoots.push_back(cwd / ".." / "..");
        }

        // 2. Executable module directory and parent paths (Windows)
#if defined(_WIN32)
        wchar_t exeBuffer[MAX_PATH];
        const DWORD len = GetModuleFileNameW(nullptr, exeBuffer, MAX_PATH);
        if (len > 0 && len < MAX_PATH) {
            const fs::path exeDir = fs::path(exeBuffer).parent_path();
            searchRoots.push_back(exeDir);
            searchRoots.push_back(exeDir / "..");
            searchRoots.push_back(exeDir / ".." / "..");
        }
#endif

        for (const auto& root : searchRoots) {
            const fs::path candidate = root / inputPath;
            if (fs::exists(candidate, ec)) {
                return fs::weakly_canonical(candidate, ec).string();
            }
        }

        return relPath;
    }
};

} // namespace torquelab
