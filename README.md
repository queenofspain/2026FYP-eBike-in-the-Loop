# 2026FYP-eBike-in-the-Loop

## Set up
### Automatic
- Open 'start_all.bat' in a text editor
- Ensure 'PROJECT_DIR' is set to your local filepath (where the code is stored)
- Ensure 'NGROK_DOMAIN' is set to your own domain (if running your own server)
- Save and close 'start_all.bat'
- Double click 'start_all.bat' to run
 - This will open 3 command prompt terminals:
  - Master running terminal
  - Flask server status
  - nGrok status 
 - This will establish a connection with the server and start up the website
- Once the website is running, paste the URL into phone browser
- Paste the same URL into the URL line in the website, with "/update" at the end
- Click start tracking on the website
- In the master terminal, press enter to open SUMO and begin simulation
### Manual
- Download ngrok from windows store (follow instrucions on how to set up)
- Run "python server.py" in comand line (make sure directory is correct)
- Run "ngrok http 5000 --basic-auth="username:password"" in a separate command line. This will give a https url
- Put the url into phone browser
- Put the same url into the url line inside the app with /update at the end
- Click start tracking on the webapp and make sure there are no errors
- Run "python live_phone_to_sumo.py" in another command line and it should bring up sumo and start running
 
> **NOTE:** You will have to get another map through OSMwebwizard if you want to use at home (can't map to the uni map)
