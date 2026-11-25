$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$handPython='C:\Users\current robotics\Documents\Codex\2026-10-13\error-code-unsupported-country-region-territory\work\python\python.exe'
if (-not (Test-Path -LiteralPath $handPython)) { $handPython='python' }
Write-Host 'Preview: http://localhost:8099/viewer/index.html'
& $handPython (Join-Path $PSScriptRoot 'scripts/launch_preview.py') --port 8099
