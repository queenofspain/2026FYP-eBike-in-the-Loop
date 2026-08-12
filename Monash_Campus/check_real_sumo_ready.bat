@echo off
REM Checks whether Flask, SUMO_HOME, TraCI, and SUMO config files are ready.

cd /d "%~dp0"
python check_real_sumo_ready.py
