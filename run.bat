@echo off
cd /d "%~dp0"
echo Starting Project ARES...
if exist "build\ProjectARES.exe" (
    start "" "build\ProjectARES.exe"
) else if exist "build\Release\ProjectARES.exe" (
    start "" "build\Release\ProjectARES.exe"
) else (
    echo Error: ProjectARES.exe not found in build\ or build\Release\.
    pause
)
