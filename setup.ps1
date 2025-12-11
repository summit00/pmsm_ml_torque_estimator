# setup.ps1 - Fixed version with Python detection
Write-Host "Setting up PMSM ML project..." -ForegroundColor Green

# Check if Python is in PATH
Write-Host "Looking for Python..." -ForegroundColor Yellow
$pythonPath = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $pythonPath) {
    Write-Host "Python not found in PATH. Trying python3..." -ForegroundColor Yellow
    $pythonPath = (Get-Command python3 -ErrorAction SilentlyContinue).Source
}

if (-not $pythonPath) {
    Write-Host "❌ Python not found!" -ForegroundColor Red
    Write-Host "Please install Python 3.9+ from python.org" -ForegroundColor Red
    Write-Host "Or add Python to your PATH" -ForegroundColor Red
    exit 1
}

Write-Host "✅ Found Python at: $pythonPath" -ForegroundColor Green

# Check Python version
Write-Host "Checking Python version..." -ForegroundColor Yellow
$version = & $pythonPath --version 2>&1
Write-Host "Python version: $version" -ForegroundColor Green

# Create virtual environment if it doesn't exist
if (-not (Test-Path ".venv")) {
    Write-Host "Creating virtual environment..." -ForegroundColor Yellow
    & $pythonPath -m venv .venv
    if (-not (Test-Path ".venv")) {
        Write-Host "❌ Failed to create virtual environment" -ForegroundColor Red
        exit 1
    }
    Write-Host "✅ Virtual environment created" -ForegroundColor Green
} else {
    Write-Host "Virtual environment already exists" -ForegroundColor Yellow
}

# Activate virtual environment
Write-Host "Activating virtual environment..." -ForegroundColor Yellow
$activatePath = ".venv\Scripts\Activate.ps1"
if (Test-Path $activatePath) {
    & $activatePath
    Write-Host "✅ Virtual environment activated" -ForegroundColor Green
} else {
    Write-Host "❌ Could not find activation script" -ForegroundColor Red
    exit 1
}

# Use the virtual environment's Python and pip
$venvPython = ".venv\Scripts\python.exe"
$venvPip = ".venv\Scripts\pip.exe"

# Upgrade pip
Write-Host "Upgrading pip..." -ForegroundColor Yellow
& $venvPython -m pip install --upgrade pip

# Install from pyproject.toml
Write-Host "Installing dependencies from pyproject.toml..." -ForegroundColor Yellow
& $venvPip install -e .

Write-Host "`n✅ Setup complete!" -ForegroundColor Green
Write-Host "Virtual environment: .venv" -ForegroundColor Cyan
Write-Host "To activate later: .venv\Scripts\Activate.ps1" -ForegroundColor Cyan
Write-Host "Python path: $venvPython" -ForegroundColor Cyan