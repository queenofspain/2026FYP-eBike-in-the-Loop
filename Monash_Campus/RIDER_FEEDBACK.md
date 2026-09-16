# Rider advice add-on

This add-on uses the advice rules and dashboard design from the supplied ZIP,
adapted to the current campus map and the current five-method GPS bridge. It
adds files only; the existing GPS page, server, matchers, and bridge source stay
unchanged.

## Run with live GPS

1. Ensure `SUMO_HOME` points to the installed SUMO folder. Install `flask` and
   `requests` in the Python environment used to run the project.
2. From this folder, run `start_feedback.bat`. This starts `feedback_server.py`,
   ngrok, and then `feedback_live.py` after you press a key. If you want ngrok
   basic authentication, set `EBIKE_NGROK_AUTH=user:password` before launching.
3. Open the ngrok HTTPS URL with `/feedback` on the phone. Press **Start GPS
   tracking** and allow location access. The page sends the same `/update`
   payload as the original GPS page.
4. In the bridge window, select any of the current map matching methods and
   the optional Kalman filter. The bridge still writes its normal CSV log when
   launched with `--log`.

For a desktop demo without a phone or bridge, run `python feedback_server.py`,
open `http://127.0.0.1:5000/feedback`, and press **Start SUMO demo**. The demo
uses the current `2026-08-25-19-43-30` campus network and `simBike` route.

For an automated SUMO-only test, run `python -B test_feedback_sumo_run.py` from
this folder. It starts a temporary local feedback server on a free port, runs
the current SUMO route for 12 seconds, verifies fresh samples and advice, and
shuts down. It needs no phone, GPS signal, ngrok, or separate server window.

Do not run `start_all.bat` at the same time: both launchers use port 5000. The
original GPS page remains at `/`; the new combined GPS and advice page is at
`/feedback`.

## Data flow

`/feedback` collects phone GPS → unchanged `/update` and `/latest` → unchanged
map matching and `moveToXY` in `live_phone_to_sumo.py` → additive
`feedback_live.py` reads the bike's SUMO state → `/api/feedback/telemetry` →
`rider_advice.py` selects one action → `/api/feedback/status` → rider page.

The add-on reads the current matched edge and follows only unambiguous road
continuations. The original bridge has a dummy SUMO route, so an upcoming
feature beyond an unknown turn is reported as unknown. Signal color is not
shown without a reliable movement-specific value. Road limits come from the
current SUMO lane; they are not a legal speed limit claim. Advice disappears
when samples are more than five seconds old.

The demo runner is separate from the live GPS bridge. Run one mode at a time.
This is a prototype advisory interface, not a safety-critical navigation
system.
