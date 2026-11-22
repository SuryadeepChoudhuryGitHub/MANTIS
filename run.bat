@echo off
cd /d "%~dp0"
echo Starting MANTIS Planetary Rover Simulator...
if exist "build\MANTIS.exe" (
    start "" "build\MANTIS.exe"
) else if exist "build\Release\MANTIS.exe" (
    start "" "build\Release\MANTIS.exe"
) else if exist "build\ProjectARES.exe" (
    start "" "build\ProjectARES.exe"
) else if exist "build\Release\ProjectARES.exe" (
    start "" "build\Release\ProjectARES.exe"
) else (
    echo Error: MANTIS.exe not found in build\ or build\Release\.
    pause
)
