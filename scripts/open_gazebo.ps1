# Open a GUI client for the workbench's current simulation; never start a second server.
#Requires -Version 7.0
$ErrorActionPreference = 'Stop'
$backend = 'mooncar-terrain-validation'
$viewer = 'mooncar-wslg-viewer'
$owner = 'mooncar.wslg'
$state = Invoke-RestMethod 'http://127.0.0.1:8766/api/simulation' -NoProxy -TimeoutSec 5
if ($state.phase -notin @('starting', 'recording') -or $state.task.id -notmatch '^experiment-[a-f0-9]{12}$') {
    throw 'Open http://127.0.0.1:8765 and click Apply start and collect first. No simulation was started or replaced.'
}
$experiment = $state.task.id
$image = docker inspect --format '{{.Config.Image}}' $backend
if ($LASTEXITCODE -ne 0) { throw 'The existing simulation container is unavailable.' }
wsl -d Ubuntu -- test -S /mnt/wslg/.X11-unix/X0
if ($LASTEXITCODE -ne 0) { throw 'WSLg display is unavailable. No system settings were changed.' }
$existing = docker ps -aq --filter "name=^/$viewer$"
if ($existing) {
    $info = (docker inspect $viewer | ConvertFrom-Json)[0]
    if ($info.Config.Labels.'mooncar.viewer' -ne $owner) { throw 'Viewer name is occupied by an unrelated container.' }
    if ($info.State.Running -and $info.Config.Labels.'mooncar.experiment' -eq $experiment) {
        Write-Output 'The Gazebo WSLg window for this experiment is already open.'
        return
    }
    # Only remove this script's disposable GUI client; the simulation keeps running.
    docker rm -f $viewer | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Could not close the previous GUI client.' }
}
$arguments = @('run', '-d', '--rm', '--pull', 'never', '--name', $viewer,
    '--label', "mooncar.viewer=$owner", '--label', "mooncar.experiment=$experiment",
    '--network', "container:$backend", '--volumes-from', "${backend}:ro",
    '--mount', 'type=bind,source=/run/desktop/mnt/host/wslg/.X11-unix,target=/tmp/.X11-unix,readonly',
    '--env', 'DISPLAY=:0', '--env', 'QT_QPA_PLATFORM=xcb', '--env', 'QT_X11_NO_MITSHM=1',
    '--env', "GZ_PARTITION=mooncar-$experiment", $image,
    'gz', 'sim', '-g', '--render-engine', 'ogre', '--gui-config', "/evidence/frontend-experiments/$experiment/gui.config")
docker @arguments | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Could not open the WSLg GUI client. The simulation was left running.' }
Write-Output "Gazebo WSLg window opened for $experiment. Closing it leaves recording running."
Write-Output 'After applying a different starting point in the web page, run this script again.'
