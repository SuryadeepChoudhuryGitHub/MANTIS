@echo off
cd /d "%~dp0"
echo Running MANTIS Automated Validation Tests...
if exist "build\MANTISTests.exe" (
    "build\MANTISTests.exe"
) else if exist "build\Release\MANTISTests.exe" (
    "build\Release\MANTISTests.exe"
) else if exist "build\MANTIS.exe" (
    "build\MANTIS.exe" --tests
) else if exist "build\Release\MANTIS.exe" (
    "build\Release\MANTIS.exe" --tests
) else if exist "build\ProjectARES.exe" (
    "build\ProjectARES.exe" --tests
) else (
    echo Error: MANTISTests.exe / MANTIS.exe not found.
)
pause
