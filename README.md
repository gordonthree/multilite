# multiswitch
esp32 automated configuration and firmware update

Very much a work in progress.

Code started in Arduino IDE but moved over to Platformio.
Code contains a lot of stuff for WiFi based remote switches, DC mosfet control with current sensing, AC relay control no current sensing yet.

Goals:

Separate the configuration and firmware update functions into a library that can be more easily / cleanly included into other projects.
