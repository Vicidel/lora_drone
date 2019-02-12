# LoRa localization on a drone

In the folder "tuino_lora" is the Arduino code running on the ground node.
  Use: send an empty message in order to collect the signal metadata on the server
  
In the folder "server_app" is the Python code running on the Swisscom server.
  Use: decode the incomming message and store it in a database
