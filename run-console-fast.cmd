@echo off
call "%~dp0run-console.cmd" --camera-size 640x480 --camera-fps 30 %*
exit /b %errorlevel%
