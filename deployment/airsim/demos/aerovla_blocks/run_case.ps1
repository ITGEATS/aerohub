# Stage-5 AeroVLA case runner (degraded topology, PX4 SITL).
# Usage: .\run_case.ps1 -Case formal|cancel|fault
# Thresholds are locked in validation.md BEFORE running.
param(
  [Parameter(Mandatory=$true)]
  [ValidateSet('formal','cancel','fault')]
  [string]$Case
)

$ErrorActionPreference = 'Stop'
$T = 'C:\Users\93290\AppData\Local\Temp\aerodeploy'
$EVD = "C:\Users\93290\Desktop\无人机\aerohub\log\aerovla\stage5\$Case-$(Get-Date -Format yyyyMMdd-HHmmss)"
New-Item -ItemType Directory -Force -Path $EVD | Out-Null
Write-Output "=== [$Case] evidence dir: $EVD ==="

Write-Output "=== [$Case] launch Blocks fresh ==="
& powershell -NoProfile -ExecutionPolicy Bypass -File "$T\launch_blocks.ps1"
if ($LASTEXITCODE -ne 0) { throw 'Blocks launch failed' }

Write-Output "=== [$Case] clean aerohub containers ==="
wsl -d Ubuntu-22.04 bash /mnt/c/Users/93290/AppData/Local/Temp/aerodeploy/cleanup_all.sh

$endpoint = 'http://192.168.2.17:8090'
if ($Case -eq 'fault') { $endpoint = 'http://127.0.0.1:9' }  # unreachable -> planner failure

Write-Output "=== [$Case] run chain (endpoint=$endpoint) ==="
wsl -d Ubuntu-22.04 bash /mnt/c/Users/93290/AppData/Local/Temp/aerodeploy/run_case.sh $Case "/mnt/c/Users/93290/Desktop/无人机/aerohub/log/aerovla/stage5/$(Split-Path $EVD -Leaf)" $endpoint

Write-Output "=== [$Case] done; evidence: $EVD ==="
