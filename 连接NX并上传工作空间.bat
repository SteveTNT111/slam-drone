@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
set "CONFIG_FILE=%SCRIPT_DIR%nx_upload_config.txt"
set "NX_USER=password123456"

call :ensure_config
call :load_workspace_path

:input_ip
cls
echo ==============================================
echo      NX Upload Helper
echo ==============================================
echo.
echo Saved workspace path:
echo %WORKSPACE_PATH%
echo.
set /p "NX_IP=Paste NX IP address: "
if not defined NX_IP (
    echo.
    echo [ERROR] IP is empty.
    pause
    goto input_ip
)

call :check_tools
if errorlevel 1 goto end_script

echo.
echo [INFO] Testing SSH to %NX_USER%@%NX_IP% ...
ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new %NX_USER%@%NX_IP% "echo SSH_OK"
if errorlevel 1 (
    echo.
    echo [ERROR] SSH test failed.
    echo [HINT] Check NX IP and network, then try again.
    pause
    goto input_ip
)

:main_menu
cls
echo ==============================================
echo      NX Upload Helper
echo ==============================================
echo.
echo Current NX IP: %NX_IP%
echo Current workspace path: %WORKSPACE_PATH%
echo.
echo 1. Upload current workspace to NX
echo 2. Change local workspace path and save
echo 3. Re-enter NX IP
echo 4. Exit
echo.
set /p "CHOICE=Enter 1, 2, 3 or 4: "

if "%CHOICE%"=="1" goto upload_workspace
if "%CHOICE%"=="2" goto change_workspace_path
if "%CHOICE%"=="3" goto input_ip
if "%CHOICE%"=="4" goto end_script

echo.
echo [ERROR] Invalid choice.
pause
goto main_menu

:upload_workspace
if not exist "%WORKSPACE_PATH%" (
    echo.
    echo [ERROR] Workspace path does not exist:
    echo %WORKSPACE_PATH%
    pause
    goto main_menu
)

if not exist "%WORKSPACE_PATH%\src" (
    echo.
    echo [ERROR] This path does not contain a src folder:
    echo %WORKSPACE_PATH%
    pause
    goto main_menu
)

echo.
echo [INFO] Creating target folders on NX ...
ssh %NX_USER%@%NX_IP% "mkdir -p ~/catkin_ws/src ~/catkin_ws/tools"
if errorlevel 1 goto upload_failed

echo.
echo [INFO] Uploading src ...
scp -r "%WORKSPACE_PATH%\src" %NX_USER%@%NX_IP%:~/catkin_ws/
if errorlevel 1 goto upload_failed

if exist "%WORKSPACE_PATH%\tools" (
    echo.
    echo [INFO] Uploading tools ...
    scp -r "%WORKSPACE_PATH%\tools" %NX_USER%@%NX_IP%:~/catkin_ws/
    if errorlevel 1 goto upload_failed
)

if exist "%WORKSPACE_PATH%\*.md" (
    echo.
    echo [INFO] Uploading markdown docs ...
    for %%F in ("%WORKSPACE_PATH%\*.md") do (
        echo [INFO] Uploading %%~nxF
        scp "%%~fF" %NX_USER%@%NX_IP%:~/catkin_ws/
        if errorlevel 1 goto upload_failed
    )
)

echo.
echo [INFO] Setting execute permissions ...
ssh %NX_USER%@%NX_IP% "chmod +x ~/catkin_ws/tools/*.sh ~/catkin_ws/src/fastlio_to_mavros/scripts/*.py 2>/dev/null || true"
if errorlevel 1 goto upload_failed

echo.
echo [DONE] Workspace upload finished.
echo [NEXT] Run on NX:
echo   cd ~/catkin_ws ^&^& catkin_make
echo   source ~/catkin_ws/devel/setup.bash
echo   bash ~/catkin_ws/tools/start_uav_stack.sh
echo.
pause
goto main_menu

:change_workspace_path
echo.
echo Current saved path:
echo %WORKSPACE_PATH%
echo.
set /p "NEW_PATH=Paste new local workspace path: "
if not defined NEW_PATH (
    echo.
    echo [ERROR] Path is empty.
    pause
    goto main_menu
)

set "NEW_PATH=%NEW_PATH:"=%"

if not exist "%NEW_PATH%" (
    echo.
    echo [ERROR] Path does not exist:
    echo %NEW_PATH%
    pause
    goto main_menu
)

if not exist "%NEW_PATH%\src" (
    echo.
    echo [ERROR] This path does not contain a src folder:
    echo %NEW_PATH%
    pause
    goto main_menu
)

> "%CONFIG_FILE%" echo %NEW_PATH%
set "WORKSPACE_PATH=%NEW_PATH%"

echo.
echo [DONE] New workspace path saved:
echo %WORKSPACE_PATH%
pause
goto main_menu

:upload_failed
echo.
echo [ERROR] Upload failed.
echo 1. Check whether NX is online.
echo 2. Check whether the IP is correct.
echo 3. Check whether the hotspot network is stable.
pause
goto main_menu

:ensure_config
if not exist "%CONFIG_FILE%" (
    > "%CONFIG_FILE%" echo D:\repos\slam-drone\catkin_ws
)
exit /b 0

:load_workspace_path
set "WORKSPACE_PATH="
for /f "usebackq delims=" %%A in ("%CONFIG_FILE%") do (
    set "WORKSPACE_PATH=%%A"
)
if not defined WORKSPACE_PATH (
    set "WORKSPACE_PATH=D:\repos\slam-drone\catkin_ws"
)
exit /b 0

:check_tools
where ssh >nul 2>nul
if errorlevel 1 (
    echo.
    echo [ERROR] ssh command not found.
    echo [HINT] Install Windows OpenSSH client first.
    pause
    exit /b 1
)

where scp >nul 2>nul
if errorlevel 1 (
    echo.
    echo [ERROR] scp command not found.
    echo [HINT] Install Windows OpenSSH client first.
    pause
    exit /b 1
)
exit /b 0

:end_script
endlocal
exit /b 0
