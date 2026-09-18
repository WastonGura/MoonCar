# Live smoke check: requires an experiment already started from the web page.
#Requires -Version 7.0
$ErrorActionPreference = 'Stop'
$before = Invoke-RestMethod 'http://127.0.0.1:8766/api/simulation' -NoProxy
& "$PSScriptRoot/open_gazebo.ps1"
$first = (docker inspect mooncar-wslg-viewer | ConvertFrom-Json)[0]
& "$PSScriptRoot/open_gazebo.ps1"
$second = (docker inspect mooncar-wslg-viewer | ConvertFrom-Json)[0]
$after = Invoke-RestMethod 'http://127.0.0.1:8765/api/simulation' -NoProxy
if (!$first.State.Running -or $first.Id -ne $second.Id) { throw 'Viewer was not running or was duplicated.' }
if ($before.task.id -ne $after.task.id -or $after.phase -notin @('starting','recording')) { throw 'The original experiment was replaced or stopped.' }
if ($first.Config.Cmd[0] -ne 'gz' -or '-g' -notin $first.Config.Cmd) { throw 'Viewer must be GUI-only.' }
if (!$after.motion.depth_active) { throw 'The current sensor stream is not healthy.' }
$socket = @($first.Mounts | Where-Object Destination -eq '/tmp/.X11-unix')
if ($socket.Count -ne 1 -or $socket[0].RW) { throw 'WSLg socket must use a read-only mount.' }
Write-Output "PASS: GUI client reused; $($after.task.id) preserved; sensor stream healthy; Windows frontend connected."
