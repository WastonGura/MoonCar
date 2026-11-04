param([switch]$BackendOnly)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$image = 'sha256:6dc9c3677650c1f238d54ca67859e637980fe7052c64430eb699b826da26285d'
$containerName = 'mooncar-r5-workbench'
$backendUrl = 'http://127.0.0.1:8776'
$frontendUrl = 'http://127.0.0.1:8775'
$dockerCommand = Get-Command docker -ErrorAction SilentlyContinue
$docker = if ($dockerCommand) { $dockerCommand.Source } else { Join-Path $env:LOCALAPPDATA 'Programs\DockerDesktop\resources\bin\docker.exe' }
if (-not (Test-Path -LiteralPath $docker)) { throw 'Docker Desktop CLI is unavailable.' }
function Get-R5State([string]$Origin) {
    try { Invoke-RestMethod "$Origin/api/simulation" -TimeoutSec 2 } catch { $null }
}
if ($null -ne (Get-R5State $frontendUrl)) {
    Write-Output "Workbench already running; preserved: $frontendUrl"
    return
}
$evidencePath = Join-Path $projectRoot 'tmp\r5-workbench'
New-Item -ItemType Directory -Force -Path $evidencePath | Out-Null
if ($null -ne (Get-R5State $backendUrl)) {
    Write-Output "Backend already running; preserved: $backendUrl"
    return
}
$pythonRuntime = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
& $pythonRuntime (Join-Path $PSScriptRoot 'runtime_baseline.py') freeze --output (Join-Path $evidencePath 'runtime-source.json')
if ($LASTEXITCODE -ne 0) { throw 'Commit product changes before starting a versioned workbench.' }
$existing = & $docker inspect $containerName 2>$null
if ($LASTEXITCODE -eq 0) {
    $entry = ($existing | ConvertFrom-Json)[0]
    $source = ($entry.Mounts | Where-Object Destination -eq '/source').Source
    $evidence = ($entry.Mounts | Where-Object Destination -eq '/evidence').Source
    if ($source -ne $projectRoot -or $evidence -ne $evidencePath -or $entry.Image -ne $image -or -not $entry.HostConfig.Init) {
        throw 'Existing R5 container configuration differs. Preserved; inspect it before choosing another container name.'
    }
    if (-not $entry.State.Running) {
        & $docker start $containerName | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'R5 container could not start.' }
    }
} else {
    & $docker image inspect $image 2>$null | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'The pinned accepted R5 runtime image is missing; see containers/README.md.' }
    $arguments = @('run','-d','--name',$containerName,'--init','--cpus','4','--memory','6g','--shm-size','512m',
        '-p','127.0.0.1:8776:8776','--mount',"type=bind,src=$projectRoot,dst=/source,readonly",
        '--mount',"type=bind,src=$evidencePath,dst=/evidence",
        '-e','ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST','-e','ROS_DOMAIN_ID=97','-e','MOONCAR_EXPERIMENT_ROS_DOMAIN_ID=97',
        '-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp','-e','LIBGL_ALWAYS_SOFTWARE=1',
        '-e','GZ_SIM_RESOURCE_PATH=/source/ros2_ws/src','-e','MOONCAR_DART_PLUGIN_ROOT=/opt/mooncar/physics/dart-7.8.0',
        '-e','MOONCAR_IMAGE_DDS_RUNTIME=/opt/mooncar/image-dds/rmw-8.4.4',
        $image,'bash','/source/containers/start_r5_workbench.sh')
    & $docker @arguments | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'R5 container creation failed.' }
}
Write-Output "Building the isolated overlay; logs: docker logs $containerName"
$deadline = [DateTime]::UtcNow.AddSeconds(180)
while ($null -eq (Get-R5State $backendUrl)) {
    if ([DateTime]::UtcNow -gt $deadline) { throw 'R5 backend did not become ready. Inspect its preserved container logs.' }
    Start-Sleep -Milliseconds 500
}
if ($BackendOnly) { Write-Output "R5 backend ready: $backendUrl"; return }
$pythonRuntime = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
if (-not (Test-Path -LiteralPath $pythonRuntime)) { throw 'Existing Windows Python runtime is unavailable.' }
Write-Output "Open $frontendUrl . R5 support is selected for new tasks. Ctrl+C stops this dedicated workbench."
try {
    & $pythonRuntime (Join-Path $PSScriptRoot 'map_workbench.py') --sessions (Join-Path $evidencePath 'sessions') --port 8775 --simulation-url $backendUrl
} finally {
    & $docker stop --time 12 $containerName | Out-Null
}
