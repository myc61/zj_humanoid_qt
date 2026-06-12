# Build Windows distributable EXE (no Python required on target machine)

param(
    [ValidateSet("internal", "customer")]
    [string]$Edition = "internal",
    [string]$FeatureConfig = ""
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $repoRoot

$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $venvPython)) {
    Write-Host ".venv not found. Running setup first..."
    & (Join-Path $PSScriptRoot "setup_windows.ps1")
}

if (-not (Test-Path $venvPython)) {
    Write-Host "Cannot find .venv Python: $venvPython"
    exit 1
}

Write-Host "[1/4] Installing build dependencies and project requirements..."
& $venvPython -m pip install --upgrade pip
& $venvPython -m pip install -r (Join-Path $repoRoot "requirements.txt") pyinstaller

Write-Host "[2/4] Cleaning old build artifacts..."
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }
if (Test-Path "humanoid-robot-delivery-toolchain.spec") { Remove-Item -Force "humanoid-robot-delivery-toolchain.spec" }

Write-Host "[3/4] Building onedir EXE with PyInstaller..."
& $venvPython -m PyInstaller `
    --noconfirm `
    --clean `
    --windowed `
    --onedir `
    --name "humanoid-robot-delivery-toolchain" `
    --paths "src" `
    --add-data "check\jetpack_check.sh;check" `
    --add-data "config\finger_joints.yaml;config" `
    --add-data "config\wa2_ls_joint_data.yaml;config" `
    --collect-all "PyQt6" `
    --collect-all "autobahn" `
    --collect-all "reportlab" `
    --hidden-import "matplotlib.backends.backend_qtagg" `
    "src/app.py"

if ([string]::IsNullOrWhiteSpace($FeatureConfig)) {
    $FeatureConfig = Join-Path $repoRoot ("config\feature_profiles\" + $Edition + ".yaml")
}
if (-not (Test-Path $FeatureConfig)) {
    Write-Host "Cannot find feature config: $FeatureConfig"
    exit 1
}

Write-Host "[4/4] Creating double-click launcher..."
$defaultDistDir = Join-Path $repoRoot "dist\humanoid-robot-delivery-toolchain"
$distDir = Join-Path $repoRoot ("dist\humanoid-robot-delivery-toolchain-" + $Edition)
$featureConfigPath = Join-Path $distDir "feature_config.yaml"
$launcherPath = Join-Path $distDir "launch.bat"

if (Test-Path $distDir) { Remove-Item -Recurse -Force $distDir }
Move-Item -Path $defaultDistDir -Destination $distDir
Copy-Item -Path $FeatureConfig -Destination $featureConfigPath -Force

$launcherContent = @"
@echo off
setlocal
cd /d "%~dp0"
start "" "%~dp0humanoid-robot-delivery-toolchain.exe" %*
endlocal
"@

[System.IO.File]::WriteAllText($launcherPath, $launcherContent, [System.Text.Encoding]::ASCII)

Write-Host "Build finished. Output folder: $distDir"
Write-Host "Edition: $Edition"
Write-Host "FeatureConfig: $FeatureConfig"
Write-Host "Customer can double-click: $launcherPath"
Write-Host "Or run directly: $(Join-Path $distDir 'humanoid-robot-delivery-toolchain.exe')"
