$ErrorActionPreference = 'Stop'
$taskDirectory = $PSScriptRoot
$taskDisk = 'C:\Users\jiker\AppData\Local\Docker\wsl\disk\docker_data.vhdx'
$taskResultPath = Join-Path $taskDirectory 'docker-vhdx-compaction.json'
$taskResult = [ordered]@{
    StartedAt = (Get-Date).ToString('o')
    Disk = $taskDisk
    Status = 'checking'
    OtherDistributionsStopped = $false
    DockerRestarted = $false
}
try {
    $taskIdentity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $taskPrincipal = New-Object Security.Principal.WindowsPrincipal($taskIdentity)
    if (-not $taskPrincipal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'DiskPart compaction requires Windows administrator elevation.'
    }
    if (Get-Process -Name 'Docker Desktop','com.docker.backend' -ErrorAction SilentlyContinue) {
        throw 'Docker is running; refusing to compact its data disk.'
    }
    if ((Get-Item -LiteralPath $taskDisk).FullName -cne $taskDisk) {
        throw 'Unexpected Docker data disk path.'
    }
    $taskHandle = [IO.File]::Open($taskDisk,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::None)
    try {
        $taskHeader = New-Object byte[] 8
        [void]$taskHandle.Read($taskHeader,0,8)
        if ([Text.Encoding]::ASCII.GetString($taskHeader) -cne 'vhdxfile') {
            throw 'Target is not a VHDX file.'
        }
    } finally {
        $taskHandle.Dispose()
    }
    $taskResult.DiskBytesBefore = (Get-Item -LiteralPath $taskDisk).Length
    $taskResult.CFreeBytesBefore = ([IO.DriveInfo]::new('C:\')).AvailableFreeSpace
    $taskResult.Status = 'compacting'
    $taskResult | ConvertTo-Json | Set-Content -LiteralPath $taskResultPath -Encoding UTF8
    $taskInstructions = Join-Path $taskDirectory 'compact-docker-vhdx.diskpart.txt'
    @(
        'select vdisk file="C:\Users\jiker\AppData\Local\Docker\wsl\disk\docker_data.vhdx"'
        'detail vdisk'
        'compact vdisk'
        'detail vdisk'
        'exit'
    ) | Set-Content -LiteralPath $taskInstructions -Encoding ASCII
    $taskOutput = & "$env:SystemRoot\System32\diskpart.exe" /s $taskInstructions 2>&1
    $taskResult.DiskPartExitCode = $LASTEXITCODE
    $taskOutput | Out-File -LiteralPath (Join-Path $taskDirectory 'docker-vhdx-compaction.log') -Encoding UTF8
    $taskResult.DiskBytesAfter = (Get-Item -LiteralPath $taskDisk).Length
    $taskResult.CFreeBytesAfter = ([IO.DriveInfo]::new('C:\')).AvailableFreeSpace
    $taskResult.DiskBytesReduced = $taskResult.DiskBytesBefore - $taskResult.DiskBytesAfter
    $taskResult.CFreeBytesIncrease = $taskResult.CFreeBytesAfter - $taskResult.CFreeBytesBefore
    if ($taskResult.DiskPartExitCode -ne 0) {
        throw ('DiskPart failed with exit code ' + $taskResult.DiskPartExitCode)
    }
    $taskResult.Status = 'completed-check-log'
} catch {
    $taskResult.Status = 'failed'
    $taskResult.Error = $_.Exception.Message
} finally {
    $taskResult.FinishedAt = (Get-Date).ToString('o')
    $taskResult | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $taskResultPath -Encoding UTF8
}
