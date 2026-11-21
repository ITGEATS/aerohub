# Stage-5 AeroVLA case runner (degraded topology, PX4 SITL).
# Usage:  .\run_case.ps1 -Case formal|cancel|fault
# Thresholds are locked in validation.md BEFORE running.
#
# Local harness (Windows Blocks launcher, WSL native-docker stack scripts) is
# machine-specific and is NOT part of the repo. Supply it via -HarnessRoot
# (Windows path to a directory containing launch_blocks.ps1) and the matching
# WSL path via -HarnessWslRoot (directory containing cleanup_all.sh and
# run_case.sh). Board endpoint comes from -BoardIp / $env:ROBOPI_IP.
# Evidence is written under <repo>/log/aerovla/stage5/ (gitignored).
param(
  [Parameter(Mandatory=$true)]
  [ValidateSet('formal','cancel','fault')]
  [string]$Case,
  [string]$HarnessRoot = $env:AEROVLA_HARNESS_ROOT,
  [string]$HarnessWslRoot = $env:AEROVLA_HARNESS_WSL_ROOT,
  [string]$BoardIp = $env:ROBOPI_IP
)

$ErrorActionPreference = 'Stop'
if (-not $HarnessRoot -or -not $HarnessWslRoot) {
  throw 'Set AEROVLA_HARNESS_ROOT (Windows dir with launch_blocks.ps1) and AEROVLA_HARNESS_WSL_ROOT (matching WSL dir with cleanup_all.sh, run_case.sh). Local harness is not committed.'
}
if (-not $BoardIp) { throw 'Set ROBOPI_IP or -BoardIp (board NPU HTTP host).' }

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
$EVD = Join-Path $RepoRoot "log\aerovla\stage5\$Case-$(Get-Date -Format yyyyMMdd-HHmmss)"
# Convert <drive>:\path to /mnt/<drive_lower>/path for the WSL runner.
$EvdWsl = $EVD.Replace('\','/')
$EvdWsl = '/mnt/' + $EvdWsl.Substring(0,1).ToLower() + $EvdWsl.Substring(2)
New-Item -ItemType Directory -Force -Path $EVD | Out-Null
Write-Output "=== [$Case] evidence dir: $EVD ==="

Write-Output "=== [$Case] launch Blocks fresh ==="
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $HarnessRoot 'launch_blocks.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Blocks launch failed' }

Write-Output "=== [$Case] clean aerohub containers ==="
wsl -d Ubuntu-22.04 bash "$HarnessWslRoot/cleanup_all.sh"

$endpoint = "http://$BoardIp`:8090"
if ($Case -eq 'fault') { $endpoint = 'http://127.0.0.1:9' }  # unreachable -> planner failure

Write-Output "=== [$Case] run chain (endpoint=$endpoint) ==="
wsl -d Ubuntu-22.04 bash "$HarnessWslRoot/run_case.sh" $Case $EvdWsl $endpoint

Write-Output "=== [$Case] done; evidence: $EVD ==="
