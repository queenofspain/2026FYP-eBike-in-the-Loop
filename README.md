# 2026FYP-eBike-in-the-Loop

## Set up
- Download ngrok from windows store (follow instrucions on how to set up)
- Run "python server.py" in comand line (make sure directory is correct)
- Run "ngrok http 5000 --basic-auth="username:password"" in a separate command line. This will give a https url
- Put the url into phone browser
- Put the same url into the url line inside the app with /update at the end
- Click start tracking on the webapp and make sure there are no errors
- Run "python live_phone_to_sumo.py" in another command line and it should bring up sumo and start running
 
> **NOTE:** You will have to get another map through OSMwebwizard if you want to use at home (cant map to the uni map)
