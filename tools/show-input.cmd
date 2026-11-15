@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv-windows\Scripts\python.exe" goto missing
".venv-windows\Scripts\python.exe" "host\pad_bridge.py" --show-input %*
set "UWR_EXIT=%errorlevel%"
if not "%UWR_EXIT%"=="0" pause
exit /b %UWR_EXIT%
:missing
echo Run setup-windows.cmd first.
pause
exit /b 1
