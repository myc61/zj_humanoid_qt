# One-time Windows environment setup for source execution

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $repoRoot

Write-Host "[1/4] Checking Python launcher..."
$pyCmd = Get-Command py -ErrorAction SilentlyContinue
$pythonCmd = Get-Command python -ErrorAction SilentlyContinue

function Test-UsablePython($cmd, $args = @()) {
	try {
		$null = & $cmd @args --version 2>$null
		return ($LASTEXITCODE -eq 0)
	}
	catch {
		return $false
	}
}

function Is-WindowsAppsStub($cmdInfo) {
	if (-not $cmdInfo) { return $false }
	if (-not $cmdInfo.Source) { return $false }
	return $cmdInfo.Source -like "*\\Microsoft\\WindowsApps\\python.exe"
}

if (-not $pyCmd -and -not $pythonCmd) {
	Write-Host "Python is not installed (or not on PATH). Install Python 3.10+ first."
	exit 1
}

if (-not $pyCmd -and (Is-WindowsAppsStub $pythonCmd)) {
	Write-Host "Detected Microsoft Store python stub: $($pythonCmd.Source)"
	Write-Host "Please install real Python 3.10+ and enable Add Python to PATH."
	Write-Host "Or set TOOLCHAIN_PYTHON to a valid python.exe path before running this script."
	exit 1
}

$canUsePy = $false
$canUsePython = $false
if ($pyCmd) { $canUsePy = Test-UsablePython "py" @("-3") }
if ($pythonCmd -and -not (Is-WindowsAppsStub $pythonCmd)) { $canUsePython = Test-UsablePython "python" }

if (-not $canUsePy -and -not $canUsePython) {
	Write-Host "No usable Python interpreter found."
	Write-Host "Install Python 3.10+ from python.org (check Add Python to PATH), then rerun this script."
	exit 1
}

Write-Host "[2/4] Creating virtual environment (.venv)..."
if ($canUsePy) {
	& py -3 -m venv .venv
} else {
	& python -m venv .venv
}

$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $venvPython)) {
	Write-Host "Failed to create .venv."
	exit 1
}

Write-Host "[3/4] Upgrading pip..."
& $venvPython -m pip install --upgrade pip

Write-Host "[4/4] Installing requirements..."
& $venvPython -m pip install -r requirements.txt

Write-Host "Done. Start app with: .\scripts\run_windows.ps1"
