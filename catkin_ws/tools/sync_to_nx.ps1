param(
    [Parameter(Mandatory = $true)]
    [string]$NxHost,

    [string]$NxUser = "password123456",
    [string]$RepoRoot = "D:\repos\slam-drone"
)

$ErrorActionPreference = "Stop"

function Run-Ssh {
    param(
        [Parameter(Mandatory = $true)][string]$Host,
        [Parameter(Mandatory = $true)][string]$Command
    )

    & ssh "${NxUser}@${Host}" $Command
    if ($LASTEXITCODE -ne 0) {
        throw "SSH command failed: $Command"
    }
}

Write-Host "[INFO] NX host: $NxUser@$NxHost"
Write-Host "[INFO] Creating target directories..."
Run-Ssh -Host $NxHost -Command "mkdir -p ~/catkin_ws/src ~/catkin_ws/tools"

Write-Host "[INFO] Copying fastlio_to_mavros package..."
& scp -r "$RepoRoot/catkin_ws/src/fastlio_to_mavros" "${NxUser}@${NxHost}:~/catkin_ws/src/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed: fastlio_to_mavros"
}

Write-Host "[INFO] Copying launcher script..."
& scp "$RepoRoot/catkin_ws/tools/start_uav_stack.sh" "${NxUser}@${NxHost}:~/catkin_ws/tools/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed: start_uav_stack.sh"
}

Write-Host "[INFO] Copying diagnostics recorder..."
& scp "$RepoRoot/catkin_ws/tools/record_hover_diagnostics.sh" "${NxUser}@${NxHost}:~/catkin_ws/tools/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed: record_hover_diagnostics.sh"
}

Write-Host "[INFO] Copying startup notes..."
& scp "$RepoRoot/catkin_ws/常用启动命令.md" "${NxUser}@${NxHost}:~/catkin_ws/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed: 常用启动命令.md"
}

Write-Host "[INFO] Copying development notes..."
& scp "$RepoRoot/catkin_ws/开发文档.md" "${NxUser}@${NxHost}:~/catkin_ws/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed: 开发文档.md"
}

Write-Host "[INFO] Copying temporary notes..."
& scp "$RepoRoot/catkin_ws/临时开发文档.md" "${NxUser}@${NxHost}:~/catkin_ws/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed: 临时开发文档.md"
}

Write-Host "[INFO] Copying RViz notes..."
& scp "$RepoRoot/catkin_ws/试飞检查与RViz总说明.md" "${NxUser}@${NxHost}:~/catkin_ws/"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed: 试飞检查与RViz总说明.md"
}

Write-Host "[INFO] Setting execute permissions..."
Run-Ssh -Host $NxHost -Command "chmod +x ~/catkin_ws/tools/start_uav_stack.sh ~/catkin_ws/tools/record_hover_diagnostics.sh ~/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py"

Write-Host ""
Write-Host "[DONE] Local files have been synced to NX."
Write-Host "[NEXT] Run these commands on NX:"
Write-Host "  bash ~/catkin_ws/tools/start_uav_stack.sh"
Write-Host "  bash ~/catkin_ws/tools/record_hover_diagnostics.sh"
