# Rider Feedback Webpage Set Up

## Set up
- Run "python server.py" in comand line (make sure directory is Monash_Campus)
- Open "http://localhost:5000/feedback" in browser
- If you do not have SUMO running yet, open another comand line and run "python test_feedback_scenarios.py --loop"
- The feedback webpage should change between OK, WARN and DANGER cases
- To stop the test script, press Ctrl + C

## Test without SUMO
- Run "python server.py" in first comand line
- Open "http://localhost:5000/feedback"
- Run "python test_feedback_scenarios.py --loop" in second comand line
- This will test:
  - OK normal riding
  - WARN poor GPS accuracy
  - WARN traffic ahead
  - WARN above SUMO lane speed
  - DANGER vehicle very close ahead
  - DANGER phone GPS data stale

## Run with real SUMO (I get error when running at home)
- Run "python server.py" in first comand line
- Open "http://localhost:5000/feedback" in browser
- Run "python live_phone_to_sumo.py" in second comand line
- Use the phone telemetry page as before
- The flow should be:
  - Phone GPS goes to Flask /update
  - live_phone_to_sumo.py reads /latest
  - SUMO updates ebike0
  - live_phone_to_sumo.py reads SUMO output
  - Feedback goes to Flask /feedback/update
  - Webpage reads /feedback/latest
\
