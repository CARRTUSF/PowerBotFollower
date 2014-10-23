PowerBotFollower (Non Path Planning Ver.)
=========================================

## Different Versions
* The Path Planning variant of the code (inside the `master` branch) uses PCL to detect the marker location in 3d space with respect to PowerBot then calls PowerBot's ARNL Path Planner to navigate to that location
* The Non Path Planning variant of the code (in the `non-path-planning` and `non-path-planning-powerbot` branches) use the camera to keep PowerBot centered towards the marker, and will continuously move forward (provided a marker is detected) until the distance sensor cutoff threshold has been reached

## Build Instructions
* The branch `non-path-planning` contains the client side (the computer attached the camera) code, while the branch `non-path-planning-powerbot` contains the server side (computer inside powerbot) code.
* The client-side code required the library ARUCO (included)

## Launching the example
* First, run the server program on PowerBot's computer,
* Next, run the client program while ensuring that the IP Address and Port point to PowerBot's computer
