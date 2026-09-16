# Running the eBike project with rider advice

This guide has two independent modes:

- **Live ride:** phone GPS → existing Flask GPS endpoints → selected map matcher → SUMO bike → rider advice on the phone.
- **Testing:** SUMO's `simBike` route → rider advice, with no phone, GPS signal, or ngrok.

Run **one mode at a time**. The original `Monash_Campus/start_all.bat` is not used for rider advice; it starts the original server rather than the feedback server and also uses port 5000.

## Before either mode
```powershell
python --version
python -c "import flask, requests; print('Python packages ready')"
sumo --version
echo $env:SUMO_HOME
```

`SUMO_HOME` must point to the SUMO installation folder containing `tools`, and `sumo` and `sumo-gui` must be on `PATH`. If Flask or Requests is missing, install it in the Python environment you will use:

```powershell
python -m pip install flask requests
```

All commands below start in:

```powershell
cd D:\2026SEM2\ECE4072\2026FYP-eBike-in-the-Loop\Monash_Campus
```

## Mode 1 — Live phone GPS + map matching + rider advice

Use this mode on the Monash campus area covered by the SUMO network. Away from that map, GPS points cannot be matched reliably; use Mode 2 instead.

1. Make sure no earlier `server.py`, `feedback_server.py`, `start_all.bat`, or SUMO bridge is still running on port 5000.
2. In PowerShell, run:

   ```powershell
   .\start_feedback.bat
   ```

   The launcher opens a **feedback server** window and an **ngrok** window, then pauses before starting SUMO. Leave both windows open. The launcher uses its own folder as the working directory, so its `PROJECT_DIR` does not need editing.

3. In the ngrok window, copy the HTTPS **Forwarding** URL. On the phone, open that URL with `/feedback` added, for example:

   ```text
   https://your-ngrok-address.ngrok-free.app/feedback
   ```

   If ngrok presents a browser confirmation page, continue to the app. If you configured ngrok basic authentication, enter those credentials. Phone geolocation requires a secure browser origin, which the ngrok HTTPS URL supplies.

4. On the **Rider advice** page, tap **Start GPS tracking** and allow location access. Its status should change to `GPS sent`. Keep this page open during the ride. This page both sends GPS and displays advice; the original GPS page at `/` does not need to be open.
5. Return to the launcher window and press a key. A **SUMO bridge** window opens and asks for a map matching method and Kalman filter option. For the first run, select `1` (`native`) and press Enter for Kalman **off**. Later you can select `topo`, `fuzzy`, `hmm`, or `st` and compare runs. The launcher adds `--log`, so the bridge writes its usual CSV run log under `Monash_Campus/runs/`.
6. Wait for a fresh GPS fix. The SUMO bridge should print `[OK]` for successful placements. The phone page should then show `Advice current`, speed, road limit, road context, and one primary action such as **Approach carefully** or **Reduce speed**. If data stops arriving, it changes to an unavailable/waiting state rather than presenting old advice as current.

To stop, tap **Stop GPS** on the phone, press `Ctrl+C` in the SUMO bridge window, and then close/stop the server and ngrok windows. Do not use **Start SUMO demo** while the live bridge is running.

Optional: set `EBIKE_NGROK_AUTH` before launching if you want ngrok basic authentication:

```powershell
$env:EBIKE_NGROK_AUTH = 'your-user:your-password'
.\start_feedback.bat
```

The feedback server also serves the dashboard locally at `http://127.0.0.1:5000/feedback` for viewing on this computer. The phone must use the HTTPS tunnel URL for GPS collection.

## Mode 2 — SUMO-only testing, no live phone GPS

### Automatic end-to-end test

Run one command from `Monash_Campus`:

```powershell
python -B test_feedback_sumo_run.py
```

This starts a temporary local feedback server on a free port, drives the current campus `simBike` route in SUMO for 12 seconds, checks fresh telemetry, speed limit, upcoming intersection and traffic light data, advice selection, and clean completion, then shuts itself down. It does **not** start ngrok or use `/update`, `/latest`, or any phone GPS data. A successful run ends with `PASS: ...` and exits with code 0. The completed run on this machine produced 12 fresh SUMO samples and passed.

### Watch the SUMO-only ride on the dashboard

If you want to see the page update rather than just read test output:

1. In PowerShell from `Monash_Campus`, run:

   ```powershell
   python feedback_server.py
   ```

2. On this computer, open `http://127.0.0.1:5000/feedback`.
3. Click **Start SUMO demo**. You can change **Demo duration** first (10–900 seconds). The dashboard will show SUMO speed, current lane limit, the next intersection, the next traffic light on the known demo route, and the selected advice.
4. Click **Stop demo** to end early, or let the ride finish. The page should show **Ride stopped** or **Ride complete**, with no current telemetry. Press `Ctrl+C` in PowerShell to stop the server.

This visual demo does not need a phone, ngrok, or the live SUMO bridge. It uses the current `Monash_Campus/2026-08-25-19-43-30` map and route.

## What each value on the rider feedback screen means

The dashboard reads `/api/feedback/status` about once per second. Values are shown only while the server has a fresh SUMO sample (no more than five seconds old). `—` means the value is unavailable; it is **not** zero. The displayed speed and road limit are rounded to whole km/h, acceleration to one decimal place, and heading to the nearest degree/compass direction.

| Screen item | Meaning | Live phone GPS mode | SUMO-only testing mode |
|---|---|---|---|
| **Connection / ride state** | Whether current advice is available: ready, waiting, current, delayed, stopped, complete, error, or server offline. | The feedback server checks its bridge heartbeat and the age of the last bike sample. The phone can keep sending GPS while SUMO is not yet providing advice. | The feedback server checks the demo process and age of its last bike sample. |
| **Current advice** and reason | One action selected from the current speed and SUMO road context; the reason names the relevant limit or distance. Colour and icon indicate warning, caution, good, or neutral. | Selected by `rider_advice.py` after `feedback_live.py` publishes a successfully placed bike sample. | Selected by the same `rider_advice.py` rules after `feedback_demo.py` publishes a SUMO sample. |
| **Current speed (km/h)** | Bike speed used by the advice rules, displayed as `speed_mps × 3.6`. | Comes from the phone browser's GPS `coords.speed`, through the existing GPS bridge. The current bridge converts an unavailable phone speed to `0`, so a stationary reading can be misleading if the phone does not report speed. | Comes from SUMO `traci.vehicle.getSpeed(simBike)`. |
| **Acceleration (m/s²)** | Change in speed per second; positive means speeding up, negative means slowing down. | Calculated from consecutive phone speeds and phone fix timestamps when they are 0.1–10 seconds apart. Otherwise shown as `—`. | Comes from SUMO `traci.vehicle.getAcceleration(simBike)`. |
| **Heading** | Travel direction, shown as degrees clockwise from north and one of N, NE, E, SE, S, SW, W, NW. It is not the orientation of the phone in the rider's hand. | Normally the phone browser's GPS `coords.heading`. If absent, the feedback adapter reads SUMO `traci.vehicle.getAngle()` after placement. | Comes from SUMO `traci.vehicle.getAngle(simBike)`. |
| **Road speed limit (km/h)** | Maximum speed recorded for the bike's current SUMO lane (`m/s × 3.6`); **not** a verified posted/legal road limit. “Current junction lane” appears on an internal junction edge. | Read from the lane on which the selected map matcher placed the bike: `traci.lane.getMaxSpeed(lane_id)`. | Read from `simBike`'s current SUMO lane with the same TraCI query. |
| **Upcoming intersection** | Metres along the known road/route to the next intersection. `Crossing` means the bike is inside one; `None` means none was found on the known continuation; `—` means the route or query is uncertain. | Calculated from the matched current edge, lane position, and SUMO network geometry. It follows only unambiguous continuations, up to 250 m; it does not guess a turn. | Follows `simBike`'s predefined SUMO route and uses `traci.vehicle.getDrivingDistance()` to the next qualifying junction. |
| **Upcoming traffic light** | Metres to the next known light relevant to the road/route. `None` and `—` have the same absence/unknown distinction as above. It can be at a different distance from the next intersection. | Identified from a traffic-light junction on the current edge or an unambiguous continuation. Beyond an uncertain turn it remains unknown. | Comes from SUMO `traci.vehicle.getNextTLS(simBike)`, which follows the predefined route. |
| **Signal colour**, if shown | Optional state for the upcoming light, shown below its distance. It does not decide the primary advice. | Usually absent: the live bridge has no reliable intended turn or movement-specific signal state. | Comes from the state returned by `getNextTLS`; only recognised states are displayed. |
| **Telemetry / SUMO time** | Whether a current sample exists, the SUMO simulation clock in seconds, and whether the sample is live GPS or demo. SUMO time is not the phone's wall-clock time. | From the running SUMO process after each successful GPS placement. | From the SUMO demo after each simulation step. |
| **GPS status** | Whether this page acquired and uploaded a browser location fix; includes the phone-reported horizontal accuracy in metres. “GPS sent” alone does not mean map matching succeeded. | From the browser Geolocation API and the response to the existing `/update` endpoint. | Not used; GPS stays off. |
| **Diagnostics: edge, lane, context** | SUMO identifiers and how far road context can be trusted. These are technical IDs, not street names. | The edge/lane after map matching and `moveToXY`; context says current edge and unambiguous continuations. | `simBike`'s current edge/lane; context says known SUMO route. |

### When each advice message appears

The server chooses **one** primary message for each fresh sample, in this order. A higher row wins if several conditions are true. The distances in the road cards remain visible even when a different action wins.

| Priority | Message | Condition |
|---|---|---|
| Before road checks | **Start GPS or a demo ride**, **Waiting for rider data**, **Ride stopped**, **Ride complete**, or **Advice unavailable** | No current ride/sample, startup, stop, completion, error, or telemetry older than five seconds. If the browser cannot fetch the server, it shows **Advice unavailable**. Old telemetry is cleared rather than presented as current. |
| 1 | **Reduce speed** | Current speed is more than **1.08 km/h** above the SUMO lane limit. Once this warning is active, it remains until the excess falls to **0.36 km/h or less** to reduce flicker. Both speed and limit must be known. |
| 2 | **Check the traffic light** | A traffic light is the selected nearby feature, the bike is moving under **0.2 m/s**, and the light is **5 m or less** ahead. This uses distance, not signal colour. |
| 3 | **Approach carefully** | An upcoming intersection or traffic light is within `max(30, min(80, speed_mps × 6))` metres (30–80 m). The nearest feature is chosen; a same-distance tie favours the light. The selected feature may remain active for an extra 10 m and may be retained when another is within 5 m, reducing message flicker. |
| 4 | **Continue carefully** | The bike is currently inside a SUMO intersection and no higher-priority condition applies. |
| 5 | **Stay attentive** | Speed or lane limit is unavailable, or either upcoming-feature status is unknown, and no higher-priority warning applies. This avoids claiming that an incompletely observed road is clear. |
| 6 | **Check before moving** | Speed is under **0.2 m/s**, road information is complete, and no nearby-feature message applies. |
| 7 | **Keep a steady pace** | Fresh speed and road context are complete, speed is within the lane-limit warning threshold, and none of the conditions above applies. |

These are prototype SUMO advice thresholds. Signal red/green/yellow is supporting context only; a green indication never produces a “safe to go” action. In live mode, unknown turns can leave road context incomplete, so **Stay attentive** may be more common than in the predefined-route demo.

## If the live advice does not appear

Check the path in order:

| Check | Expected result |
|---|---|
| Phone `/feedback` page | `GPS sent` after allowing location access |
| Feedback server window | Receives `POST /update` from the phone |
| SUMO bridge window | Prints `[OK]` for matched GPS placements |
| `http://127.0.0.1:5000/api/feedback/status` on the computer | `"fresh": true`, a `telemetry` object, and an `advice` object |

If the bridge only reports invalid or stale GPS, leave the phone page open, confirm location permission, and check that the phone is on the mapped campus area. If port 5000 is already in use, stop the older server before starting a mode. If ngrok fails, read its window for an account/configuration error; the SUMO-only test does not require ngrok.

Live road lookahead is deliberately limited when the rider's next turn is unknown. The testing route is predefined, so its intersection and traffic light distances can be derived farther ahead. Signal color is shown only when SUMO can associate it with the known route.
