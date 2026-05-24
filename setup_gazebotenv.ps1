Write-Host "======================================" -ForegroundColor Cyan
Write-Host "GazeBot Environment Setup" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "Step 1: Checking for Python 3.11..." -ForegroundColor Yellow
$pythonVersion = python --version 2>&1
Write-Host "Found: $pythonVersion" -ForegroundColor Green
Write-Host ""

Write-Host "Step 2: Creating virtual environment 'gazebotenv'..." -ForegroundColor Yellow
python -m venv gazebotenv

if ($LASTEXITCODE -eq 0) {
    Write-Host "✅ Virtual environment created successfully" -ForegroundColor Green
} else {
    Write-Host "❌ Failed to create virtual environment" -ForegroundColor Red
    exit 1
}
Write-Host ""

Write-Host "Step 3: Activating virtual environment..." -ForegroundColor Yellow
& .\gazebotenv\Scripts\Activate.ps1

if ($LASTEXITCODE -eq 0) {
    Write-Host "✅ Virtual environment activated" -ForegroundColor Green
} else {
    Write-Host "❌ Failed to activate virtual environment" -ForegroundColor Red
    exit 1
}
Write-Host ""

Write-Host "Step 4: Upgrading pip..." -ForegroundColor Yellow
python -m pip install --upgrade pip

Write-Host "✅ Pip upgraded" -ForegroundColor Green
Write-Host ""

Write-Host "Step 5: Installing requirements from requirements.txt..." -ForegroundColor Yellow
pip install -r requirements.txt

if ($LASTEXITCODE -eq 0) {
    Write-Host "✅ All requirements installed successfully" -ForegroundColor Green
} else {
    Write-Host "❌ Failed to install requirements" -ForegroundColor Red
    exit 1
}
Write-Host ""

Write-Host "Step 6: Verifying installed packages..." -ForegroundColor Yellow
pip list | Select-String -Pattern "opencv|pyserial|mediapipe|cvzone|numpy"
Write-Host ""

Write-Host "======================================" -ForegroundColor Cyan
Write-Host "✅ Setup Complete!" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "To activate the environment in future sessions, run:" -ForegroundColor Cyan
Write-Host ".\gazebotenv\Scripts\Activate.ps1" -ForegroundColor White
Write-Host ""
