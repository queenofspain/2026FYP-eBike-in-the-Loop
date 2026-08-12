@echo off
REM ============================================================
REM  eBike-in-the-Loop launcher
REM  Opens 3 windows: Flask server, ngrok tunnel, SUMO bridge
REM ============================================================

REM --- EDIT THESE IF NEEDED ---
set "PROJECT_DIR=C:\Uni\Final Year Project\Code\Edited Code\Monash_Campus"
set "NGROK_DOMAIN=outward-expanse-result.ngrok-free.dev"
set "NGROK_AUTH=bike:bikebike"
set "FLASK_PORT=5000"
set "SUMO_SCRIPT=two_bikes.py"
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

echo.
echo ============================================================
echo  Tunnel should now be active.
echo  Phone URL: https://%NGROK_DOMAIN%/
echo  Please open the website on your device and press START
echo  off
echo  Please press enter...
echo ============================================================
echo.
pause

echo [launcher] Starting SUMO bridge script...
start "SUMO Bridge" cmd /k "cd /d %PROJECT_DIR% && python %SUMO_SCRIPT%"

echo.
echo [launcher] All processes launched.
pause