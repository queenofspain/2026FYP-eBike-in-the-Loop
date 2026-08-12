@echo off
REM Starts only the Flask web server for the phone page and rider feedback page.

cd /d "%~dp0"

echo [web] Starting Flask server on http://localhost:5000
echo [web] Phone telemetry page:  http://localhost:5000/
echo [web] Rider feedback page:   http://localhost:5000/feedback
echo.

python server.py
