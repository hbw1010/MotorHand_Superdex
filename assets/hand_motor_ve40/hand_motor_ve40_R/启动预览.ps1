$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$vePython='C:\Users\current robotics\Documents\Codex\2026-10-13\error-code-unsupported-country-region-territory\work\python\python.exe'
if (-not (Test-Path -LiteralPath $vePython)) { $vePython='python' }
Write-Host 'Preview: http://localhost:8098/viewer/index.html'
& $vePython (Join-Path $PSScriptRoot 'scripts/launch_preview.py') --port 8098
