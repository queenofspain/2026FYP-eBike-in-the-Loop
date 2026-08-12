@echo off
REM ============================================================
REM  eBike-in-the-Loop launcher
REM  Opens Flask, ngrok, and SUMO bridge windows.
REM ============================================================

REM --- EDIT THESE IF NEEDED ---
set "PROJECT_DIR=%~dp0"
set "NGROK_DOMAIN=kennedy-duelistic-favoredly.ngrok-free.dev"
set "NGROK_AUTH=bike:bikebike"
set "FLASK_PORT=5000"
set "SUMO_SCRIPT=live_phone_to_sumo.py"
set "SERVER_SCRIPT=server.py"
REM ----------------------------

cd /d "%PROJECT_DIR%"

echo [launcher] Starting Flask server...
start "Flask Server" cmd /k "cd /d %PROJECT_DIR% && python %SERVER_SCRIPT%"

echo [launcher] Waiting for Flask to bind to port %FLASK_PORT%...
timeout /t 3 /nobreak >nul

echo [launcher] Starting ngrok tunnel...
start "ngrok" cmd /k "ngrok http %FLASK_PORT% --domain=%NGROK_DOMAIN% --basic-auth=%NGROK_AUTH%"

echo [launcher] Waiting for tunnel to come up...
timeout /t 4 /nobreak >nul

echo [launcher] Starting SUMO bridge...
start "SUMO Bridge" cmd /k "cd /d %PROJECT_DIR% && python %SUMO_SCRIPT%"

echo.
echo [launcher] Processes launched in separate windows.
echo [launcher] Local phone telemetry page: http://localhost:%FLASK_PORT%/
echo [launcher] Local rider feedback page:  http://localhost:%FLASK_PORT%/feedback
echo [launcher] Phone URL: https://%NGROK_DOMAIN%/
echo.
pause
