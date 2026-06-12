# PowerShell script to run the humanoid robot delivery toolchain on Windows

$ErrorActionPreference = "Stop"

# Always run from repository root so relative paths stay stable.
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $repoRoot

$appPath = Join-Path $repoRoot "src\app.py"

if (-not (Test-Path $appPath)) {
    Write-Host "Entry file not found: $appPath"
    exit 1
}

function Get-PythonCommand {
    if ($env:TOOLCHAIN_PYTHON -and (Test-Path $env:TOOLCHAIN_PYTHON)) {
        return $env:TOOLCHAIN_PYTHON
    }

    $venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"
    if (Test-Path $venvPython) {
        return $venvPython
    }

    $pyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($pyLauncher) {
        return "py"
    }

    $pythonCmd = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCmd) {
        return "python"
    }

    return $null
}

$pythonCmd = Get-PythonCommand
if (-not $pythonCmd) {
    Write-Host "Python not found. Please run .\scripts\setup_windows.ps1 first."
    exit 1
}

if ($pythonCmd -eq "py") {
    & py -3 $appPath @args
} else {
    & $pythonCmd $appPath @args
}