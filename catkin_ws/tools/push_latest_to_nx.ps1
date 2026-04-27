param(
    [string]$NxHost = "",
    [string]$NxUser = "password123456",
    [string]$RepoRoot = "D:\repos\slam-drone"
)

$ErrorActionPreference = "Stop"

function Get-ActiveIPv4Prefix {
    $defaultRoute = Get-NetRoute -DestinationPrefix "0.0.0.0/0" |
        Sort-Object RouteMetric, ifMetric |
        Select-Object -First 1

    if (-not $defaultRoute) {
        throw "没有找到当前默认路由，无法判断热点网段。"
    }

    $ip = Get-NetIPAddress -InterfaceIndex $defaultRoute.InterfaceIndex -AddressFamily IPv4 |
        Where-Object { $_.IPAddress -notlike '169.*' } |
        Select-Object -First 1 -ExpandProperty IPAddress

    if (-not $ip) {
        throw "没有找到当前活动网卡的 IPv4 地址。"
    }

    return (($ip -split '\.')[0..2] -join '.')
}

function Test-PortQuick {
    param(
        [Parameter(Mandatory = $true)][string]$Host,
        [Parameter(Mandatory = $true)][int]$Port,
        [int]$TimeoutMs = 200
    )

    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $iar = $client.BeginConnect($Host, $Port, $null, $null)
        $ok = $iar.AsyncWaitHandle.WaitOne($TimeoutMs, $false)
        if (-not $ok) {
            return $false
        }
        $client.EndConnect($iar) | Out-Null
        return $true
    }
    catch {
        return $false
    }
    finally {
        $client.Close()
    }
}

function Find-NxCandidates {
    $prefix = Get-ActiveIPv4Prefix
    Write-Host "[信息] 当前活动网段: $prefix.0/24"
    Write-Host "[信息] 正在扫描同网段主机的 22 和 4000 端口，请稍等..."

    $found = New-Object System.Collections.Generic.List[object]
    1..254 | ForEach-Object {
        $ip = "$prefix.$_"
        $sshOpen = Test-PortQuick -Host $ip -Port 22
        $nxOpen = Test-PortQuick -Host $ip -Port 4000
        if ($sshOpen -or $nxOpen) {
            $found.Add([pscustomobject]@{
                IP = $ip
                SSH = $sshOpen
                NoMachine = $nxOpen
            })
        }
    }

    return $found
}

function Resolve-NxHost {
    param([string]$PreferredHost)

    if ($PreferredHost) {
        return $PreferredHost
    }

    $candidates = Find-NxCandidates
    if ($candidates.Count -eq 0) {
        throw "没有在当前热点网段里找到开放 22 或 4000 端口的主机。请确认台式机和 NX 都连到了同一个热点。"
    }

    if ($candidates.Count -eq 1) {
        $auto = $candidates[0].IP
        Write-Host "[信息] 自动找到候选 NX: $auto"
        return $auto
    }

    Write-Host "[信息] 找到多个候选主机："
    $index = 1
    foreach ($item in $candidates) {
        Write-Host ("  [{0}] {1}  SSH={2}  NoMachine={3}" -f $index, $item.IP, $item.SSH, $item.NoMachine)
        $index++
    }

    $choice = Read-Host "请输入要使用的序号"
    $selectedIndex = 0
    if (-not [int]::TryParse($choice, [ref]$selectedIndex)) {
        throw "输入的序号无效。"
    }

    $selectedIndex = $selectedIndex - 1
    if ($selectedIndex -lt 0 -or $selectedIndex -ge $candidates.Count) {
        throw "序号超出范围。"
    }

    return $candidates[$selectedIndex].IP
}

function Run-Ssh {
    param([string]$Host, [string]$Command)
    & ssh "${NxUser}@${Host}" $Command
    if ($LASTEXITCODE -ne 0) {
        throw "SSH 命令执行失败：$Command"
    }
}

function Run-Scp {
    param([string]$Source, [string]$Target)
    & scp $Source "${NxUser}@${resolvedHost}:$Target"
    if ($LASTEXITCODE -ne 0) {
        throw "SCP 传输失败：$Source -> $Target"
    }
}

$resolvedHost = Resolve-NxHost -PreferredHost $NxHost

Write-Host "[信息] 本次使用的 NX 地址: $resolvedHost"
Write-Host "[信息] 准备在 NX 上创建目录..."

Run-Ssh -Host $resolvedHost -Command "mkdir -p ~/catkin_ws/src ~/catkin_ws/tools"

Write-Host "[信息] 开始同步 fastlio_to_mavros 包..."
& scp -r "$RepoRoot/catkin_ws/src/fastlio_to_mavros" "${NxUser}@${resolvedHost}:~/catkin_ws/src/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP 传输失败：fastlio_to_mavros"
}

Write-Host "[信息] 开始同步工具脚本和说明文档..."
Run-Scp -Source "$RepoRoot/catkin_ws/tools/start_uav_stack.sh" -Target "~/catkin_ws/tools/"
Run-Scp -Source "$RepoRoot/catkin_ws/tools/connect_nx_hotspot.sh" -Target "~/catkin_ws/tools/"
Run-Scp -Source "$RepoRoot/catkin_ws/tools/record_hover_diagnostics.sh" -Target "~/catkin_ws/tools/"
Run-Scp -Source "$RepoRoot/catkin_ws/常用启动命令.md" -Target "~/catkin_ws/"

Write-Host "[信息] 设置可执行权限..."
Run-Ssh -Host $resolvedHost -Command "chmod +x ~/catkin_ws/tools/start_uav_stack.sh ~/catkin_ws/tools/connect_nx_hotspot.sh ~/catkin_ws/tools/record_hover_diagnostics.sh ~/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py"

Write-Host
Write-Host "[完成] 已把本地最新版同步到 NX。"
Write-Host "[下一步] 你现在可以在 NX 上执行："
Write-Host "  bash ~/catkin_ws/tools/start_uav_stack.sh"
