@echo off
REM Sends controlled no-SUMO feedback scenarios to the rider feedback webpage.

cd /d "%~dp0"
python test_feedback_scenarios.py --loop
