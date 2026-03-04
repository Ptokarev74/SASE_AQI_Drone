# SASE AQI Drone: PWA Installation & Testing Guide

This guide explains how to test the SASE AQI Drone frontend as a Progressive Web App (PWA) using your local computer and your mobile device.

To be considered a true PWA, the app needs to be served over **HTTPS** (or `localhost`). This allows the browser to securely register the Service Worker, enable offline caching, and offer the "Add to Home Screen" prompt.

---

## 1. Testing on your Computer (Quickest Method)

You can easily verify that the Service Worker is building correctly and that the app is installable directly on your PC.

1. Open a terminal in the `apps/pwa` directory.
2. Build the production app:
   ```powershell
   npm run build
   ```
3. Start the production preview server:
   ```powershell
   npm run preview
   ```
4. Open your browser (Chrome or Edge) and navigate to `http://localhost:4173`.
5. **Verify Installation:** Look at the right side of the URL address bar. You should see a small monitor icon with a down arrow. Click it to "Install" the app to your computer.
6. **Verify Offline Mode:** After loading the app, disconnect your computer from Wi-Fi and refresh the page. The app will immediately load from the cache.

---

## 2. Testing on your Mobile Device (Same Wi-Fi Network)

If you want to test the touch joysticks on your actual phone, the easiest way is to connect to your computer's local IP address. 

*Note: Because this uses `http` over a local IP instead of `https`, Apple iOS may block the Service Worker and the "Add to Home Screen" prompt for security reasons, but the UI itself will work perfectly.*

1. Ensure your phone and your computer are on the **exact same Wi-Fi network**.
2. Start the preview server and tell it to expose itself to your network:
   ```powershell
   npm run preview -- --host
   ```
3. The terminal will print a "Network" IP address (e.g., `http://192.168.1.5:4173` or `http://10.0.0.x:4173`).
4. Type that exact URL into your phone's browser (Safari or Chrome).
5. The dashboard will load, and the interface will automatically switch from Keyboard controls to Touch Joysticks.

---

## 3. The "Full" Mobile PWA Experience (External HTTPS Tunnel)

To get the 100% true PWA experience on a mobile device (getting the "Add to Home Screen" prompt on iOS Safari and enabling full offline mode), you must serve the app over **HTTPS**.

The easiest way to do this without buying a domain name is to run an SSH tunnel.

1. Ensure your preview server is running: `npm run preview -- --host`.
2. Open a **new, separate terminal**.
3. Run this built-in Windows SSH command to create a secure tunnel to your local port 4173:
   ```powershell
   ssh -p 443 -R0:localhost:4173 a.pinggy.io
   ```
4. It may ask you to confirm the host key. Type `yes` and press Enter.
5. The terminal will print out a temporary public URL that looks like this:
   `https://random-words.a.free.pinggy.link`
6. Open that exact `https://...` link on your iPhone or Android.
7. **Install it:**
   - **iOS (Safari):** Tap the Share icon (square with up arrow), scroll down, and select **"Add to Home Screen"**.
   - **Android (Chrome):** An "Install App" banner should automatically appear at the bottom of the screen.
8. Close your browser and launch the new "AQI Drone" app directly from your phone's home screen. It will load in full-screen mode without an address bar.
