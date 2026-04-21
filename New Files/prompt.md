# Prompt: Add Three.js Drone Model to Web UI

Please update the frontend of the SASE AQI Drone project to include a 3D visualization of the drone's current state. 

Using the existing architecture where the web app (`templates/index.html`) polls `/api/status` for real-time telemetry (pitch, roll, etc.):

1. Add a new panel on the left side of the sleek, dark-themed control dashboard to house a 3D container.
2. Integrate `Three.js` (via CDN) into the frontend `index.html`.
3. Create a Three.js scene in the new left panel that renders a basic 3D model (e.g., a simple box or drone placeholder representing the quadcopter).
4. Update the existing JavaScript polling loop mapping the `/api/status` JSON response to dynamically update the rotation (pitch, roll, yaw) and position (if altitude is available) of the 3D model in real-time.
5. Ensure the new layout remains responsive and fits well with the existing D-pad, takeoff/land buttons, and throttle slider.
