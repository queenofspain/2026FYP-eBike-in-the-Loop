# 2026FYP-eBike-in-the-Loop

## Set up
- Download ngrok from windows store (follow instrucions on how to set up)
- Go to [https://app.netlify.com/drop) and drop the html or html_new folder (new has better formating)
- Use the link it gives on a phone for the webapp
- Run "python server.py" in comand line (make sure directory is correct)
- Run "ngrok http 5000" in a separate command line. This will give a https url, which will be used in the url section of the webapp (sends the data over this link)
- Click start tracking on the webapp and make sure there are no errors
- Run "python live_phone_to_sumo.py" in another command line and it should bring up sumo and start running
 
> **NOTE:** You will have to get another map through OSMwebwizard if you want to use at home (cant map to the uni map)
