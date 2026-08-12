# Rider Feedback Info

## What it has now
- Feedback status: OK, WARN or DANGER
- Feedback message: short reason for the status
- SUMO speed: rider speed from SUMO in km/h
- Speed limit use: rider speed compared with SUMO allowed lane speed
- Phone data age: how old the phone GPS packet is
- GPS Link: whether phone GPS is connected to the mapped route
- Route Position: rider map position status
- Lane Status: rider lane connection status
- Heading: rider angle inside SUMO
- Traffic Count: number of vehicles on current edge
- Leader Gap: distance to vehicle ahead, if SUMO can detect it
- Phone GPS: latest phone latitude and longitude
- Updated: time Flask last received feedback

## Where data comes from
- Phone GPS data comes from the phone webpage:
  - lat
  - lon
  - speed
  - course
  - accuracy
  - timestamp
- SUMO live data comes from live_phone_to_sumo.py using TraCI:
  - traci.vehicle.getSpeed
  - traci.vehicle.getAngle
  - traci.vehicle.getRoadID
  - traci.vehicle.getLaneID
  - traci.vehicle.getLanePosition
  - traci.vehicle.getAllowedSpeed
  - traci.vehicle.getLeader
  - traci.edge.getLastStepVehicleNumber
  - traci.edge.getLastStepMeanSpeed
  - traci.edge.getLastStepHaltingNumber
  - traci.edge.getTraveltime
- Feedback status and message are calculated in live_phone_to_sumo.py
- Raw SUMO edge/lane IDs are still kept inside the feedback JSON for debugging, but they are not shown on the rider webpage

## Current feedback rules
- DANGER:
  - Phone GPS data is older than 5 seconds
  - Vehicle ahead is closer than 5 m
- WARN:
  - GPS accuracy is worse than 10 m
  - Vehicle ahead is closer than 10 m
  - Rider speed is above SUMO allowed lane speed
- OK:
  - None of the warning or danger conditions are active

## Important unclear parts
- Need team to confirm if 5 m and 10 m are correct safety distances
- Need team to decide if feedback should later include audio or vibration
- Nearby vehicle detection approach is still being figured out. Current version uses SUMO getLeader, but this may need to be improved later

