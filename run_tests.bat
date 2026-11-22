@echo off
cd /d "%~dp0"
echo Running Project ARES Automated Validation Tests...
if exist "build\ProjectARES.exe" (
    "build\ProjectARES.exe" --tests
) else if exist "build\Release\ProjectARES.exe" (
    "build\Release\ProjectARES.exe" --tests
) else (
    echo Error: ProjectARES.exe not found.
)
pause
