@echo off
setlocal
cd /d "%~dp0"

if not defined EBIKE_PORT set "EBIKE_PORT=5000"
echo Starting rider feedback server on port %EBIKE_PORT%...
start "Rider feedback server" cmd /k "python feedback_server.py"
timeout /t 3 /nobreak >nul

echo Starting HTTPS tunnel. Copy its public URL and add /feedback on your phone.
if defined EBIKE_NGROK_AUTH (
  start "Rider feedback ngrok" cmd /k "ngrok http %EBIKE_PORT% --basic-auth=%EBIKE_NGROK_AUTH%"
) else (
  start "Rider feedback ngrok" cmd /k "ngrok http %EBIKE_PORT%"
)

echo.
echo Desktop advice: http://127.0.0.1:%EBIKE_PORT%/feedback
echo Phone advice:   https://YOUR-NGROK-URL/feedback
echo In the browser, press Start GPS tracking.
echo Then press a key here to start the SUMO bridge.
pause >nul

start "Rider feedback SUMO bridge" cmd /k "python feedback_live.py --log"
echo Select the map matching method and Kalman option in the bridge window.
echo The original GPS and map matching source files are unchanged.
pause
