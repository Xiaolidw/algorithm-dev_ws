@echo off
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0一键启动比赛系统.ps1"
if errorlevel 1 pause
