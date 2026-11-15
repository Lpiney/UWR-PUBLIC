@echo off
setlocal
cd /d "%~dp0"
if exist ".venv-windows\Scripts\python.exe" goto install
set "UWR_PYTHON=py -3.11"
%UWR_PYTHON% -c "import sys" >nul 2>&1
if not errorlevel 1 goto create
set "UWR_PYTHON=py -3.12"
%UWR_PYTHON% -c "import sys" >nul 2>&1
if not errorlevel 1 goto create
set "UWR_PYTHON=python"
%UWR_PYTHON% -c "import sys; assert sys.platform == 'win32' and (3,11) <= sys.version_info[:2] <= (3,12)" >nul 2>&1
if errorlevel 1 goto missing
:create
%UWR_PYTHON% -c "import struct; assert struct.calcsize('P') == 8"
if errorlevel 1 goto missing
%UWR_PYTHON% -m venv ".venv-windows"
if errorlevel 1 goto failed
:install
".venv-windows\Scripts\python.exe" -m pip install -r "host\requirements.txt"
if errorlevel 1 goto failed
".venv-windows\Scripts\python.exe" -c "import cv2, pygame, serial, numpy; assert hasattr(cv2, 'aruco'); print('Dependencies ready.')"
if errorlevel 1 goto failed
echo Setup complete. Double-click run-console.cmd to start.
if /i not "%~1"=="--no-pause" pause
exit /b 0
:missing
echo Install 64-bit Python 3.11 or 3.12 from python.org, then run this again.
:failed
echo Setup failed. Keep this window open and copy the error above.
if /i not "%~1"=="--no-pause" pause
exit /b 1
