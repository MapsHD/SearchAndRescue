$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot

Write-Host "=== Git update ===" -ForegroundColor Cyan

git pull --ff-only
if ($LASTEXITCODE -ne 0)
{
    throw "git pull failed"
}

Write-Host "=== Sync submodules ===" -ForegroundColor Cyan

git submodule sync --recursive
if ($LASTEXITCODE -ne 0)
{
    throw "git submodule sync failed"
}

git submodule update --init --recursive
if ($LASTEXITCODE -ne 0)
{
    throw "git submodule update failed"
}

Write-Host "=== Configure CMake ===" -ForegroundColor Cyan

cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0)
{
    throw "CMake configure failed"
}

Write-Host "=== Build ===" -ForegroundColor Cyan

cmake --build build --config Release -j
if ($LASTEXITCODE -ne 0)
{
    throw "Build failed"
}

Write-Host "=== Done ===" -ForegroundColor Green
