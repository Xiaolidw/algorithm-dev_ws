$ErrorActionPreference = 'Stop'

$remote = 'u22'
$remoteLauncher = '/home/ros/dev_ws/scripts/competition_control.sh start'
$coStudio = 'D:\costudio\coStudio.exe'

Write-Host '正在启动虚拟机内 Gazebo、ROS 2、导航、抓取、摄像头与 Rosbridge……'
& ssh $remote $remoteLauncher
if ($LASTEXITCODE -ne 0) {
    throw "虚拟机一键启动失败，ssh 返回码 $LASTEXITCODE。"
}

$tunnel = Get-CimInstance Win32_Process -Filter "Name='ssh.exe'" |
    Where-Object { $_.CommandLine -like '*9090:localhost:9090*u22*' } |
    Select-Object -First 1
if (-not $tunnel) {
    Start-Process -FilePath 'ssh.exe' -WindowStyle Hidden -ArgumentList @(
        '-N', '-o', 'ExitOnForwardFailure=yes',
        '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3',
        '-L', '9090:localhost:9090', $remote
    )
}

$ready = $false
for ($attempt = 0; $attempt -lt 30; $attempt++) {
    if (Test-NetConnection -ComputerName localhost -Port 9090 `
            -InformationLevel Quiet -WarningAction SilentlyContinue) {
        $ready = $true
        break
    }
    Start-Sleep -Seconds 1
}
if (-not $ready) {
    throw 'Windows 本机 9090 隧道未就绪，CoStudio 暂时无法连接。'
}

if (-not (Get-Process coStudio -ErrorAction SilentlyContinue)) {
    if (-not (Test-Path -LiteralPath $coStudio)) {
        throw "找不到 CoStudio：$coStudio"
    }
    Start-Process -FilePath $coStudio
}

Write-Host '系统已就绪。CoStudio 数据源请使用 ws://localhost:9090。'
