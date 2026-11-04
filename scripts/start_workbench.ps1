# Reuse the existing, preconfigured Docker environment; never restart an active experiment.
param([switch]$R5Candidate)
if ($R5Candidate) {
    & (Join-Path $PSScriptRoot 'start_r5_workbench.ps1')
    return
}
$ErrorActionPreference = 'Stop'
$workbenchUrl = 'http://127.0.0.1:8765'
$simulationUrl = 'http://127.0.0.1:8766'
function Get-WorkbenchState {
    param([string]$Origin)
    try { Invoke-RestMethod "$Origin/api/simulation" -TimeoutSec 2 } catch { $null }
}
$workbenchState = Get-WorkbenchState $workbenchUrl
if ($workbenchState.enabled -eq $true) {
    Write-Output "MoonCar is already running: $workbenchUrl"
    Write-Output "Experiment state: $($workbenchState.phase) (preserved)"
    return
}
if ($null -ne $workbenchState) { throw 'Port 8765 has a service without simulation mode. Leave it running; check its configuration.' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$containerJson = docker inspect mooncar-terrain-validation 2>$null
if ($LASTEXITCODE -ne 0) { throw 'The configured mooncar-terrain-validation container is missing. No environment will be installed automatically.' }
$container = ($containerJson | ConvertFrom-Json)[0]
$sourceMount = ($container.Mounts | Where-Object Destination -eq '/source').Source -replace '\\', '/'
$evidenceMount = ($container.Mounts | Where-Object Destination -eq '/evidence').Source -replace '\\', '/'
if ($sourceMount -ne $projectRoot.Replace('\', '/') -or $evidenceMount -ne (Join-Path $projectRoot 'tmp\ros_acceptance').Replace('\', '/')) {
    throw 'The configured container still mounts another project path. Recreate it with the mounts in containers/README.md.'
}
if ((Get-WorkbenchState $simulationUrl).enabled -ne $true) {
if (-not $container.State.Running) {
    docker start mooncar-terrain-validation
    if ($LASTEXITCODE -ne 0) { throw 'Docker could not start the configured container.' }
}
docker exec -d mooncar-terrain-validation bash -lc 'source /work/ros2_ws/install/setup.bash; exec python3 /source/scripts/map_workbench.py --sessions /evidence/frontend-experiments --simulation --host 0.0.0.0 --port 8766 >> /evidence/frontend-experiments-server.log 2>&1'
if ($LASTEXITCODE -ne 0) { throw 'Could not start the workbench service.' }
foreach ($attempt in 1..10) {
    Start-Sleep -Milliseconds 500
    if ((Get-WorkbenchState $simulationUrl).enabled -eq $true) { break }
}
if ((Get-WorkbenchState $simulationUrl).enabled -ne $true) { throw 'Simulation service did not become ready. Check tmp/ros_acceptance/frontend-experiments-server.log.' }
}
# Serve the site on Windows instead of asking Chrome to connect to Docker's published port.
$pythonRuntime = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
if (-not (Test-Path -LiteralPath $pythonRuntime)) { throw 'The existing local Python runtime was not found; no dependencies were installed.' }
Write-Output "Open Chrome at $workbenchUrl . Keep this PowerShell window open."
& $pythonRuntime (Join-Path $PSScriptRoot 'map_workbench.py') --sessions (Join-Path $projectRoot 'tmp\ros_acceptance\frontend-experiments') --port 8765 --simulation-url $simulationUrl
