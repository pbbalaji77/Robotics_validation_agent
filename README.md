# RoboSight 3D: Robotics Detection Validation & Evaluation Dashboard

RoboSight 3D is a premium, real-time client-side evaluation and validation dashboard designed for robotics object detection and autonomous navigation verification. Built with modern, glassmorphic dark-mode aesthetics, the dashboard visualizes raw sensor feeds (IMU, Depth Camera), reconstructs 3D spatial points, meshes surfaces, reprojects detections, and tracks telemetry performance metrics at a solid 60 FPS.

---

## 🚀 Key Features

*   **3D Point Cloud & Delaunay Mesh Reconstruction**: Converts raw depth images into 3D Cartesian coordinates and generates localized triangular mesh grids, discarding discontinuities over empty spaces.
*   **Real-Time Validation Metrics**: ComputesPrecision, Recall, F1-Score, and 3D Oriented Bounding Box Intersection over Union (IoU) on a rolling 10-second window.
*   **Bot Scrapping Proximity Alerts (CV-Based)**: Implements spatial filtering to check if walls or obstacles enter the robot's physical cylinder chassis ($0.25\text{m}$ radius with a $7\text{cm}$ warnings envelope), triggering flashing HUD `ALARM` visual panels.
*   **Bidirectional Mobile App Sync (Remote Controller)**: Employs a zero-latency sync channel supporting virtual joystick driving (teleoperation), UV disinfection light controls, and target-following autonomous navigation modes.
*   **Failures Log Gallery**: Captures model errors (False Positives and False Negatives) automatically with visual overlays, allowing downloads of frames (PNG) and coordinate attributes (JSON).
*   **PDF-Ready HTML Report Compiler**: Aggregates statistical summaries, historical performance trend charts, and captured failure cards into an offline engineering report.

---

## 🛠️ Architecture & Core Modules

The dashboard is structured using pure, native ES6 JavaScript modules with no compiling or bundlers required:

*   [`index.html`](index.html): Defines the responsive layout grid for maximize-toggled viewports.
*   [`src/index.css`](src/index.css): Core neon glassmorphic design system and keyframe alert flashing animations.
*   [`src/main.js`](src/main.js): App orchestrator coordinating loops, screenshots, report compilations, and tab communications.
*   [`src/modules/spatialPipeline.js`](src/modules/spatialPipeline.js): Core math pipeline handling depth unprojection, mesh triangulation, 3D centroid clustering, and 3D-to-2D camera reprojections.
*   [`src/modules/metricsEngine.js`](src/modules/metricsEngine.js): Core evaluation compiler checking box overlaps and spatial drift errors.
*   [`src/modules/simulator.js`](src/modules/simulator.js): Produces synthetic RGB-D frames, IMU noise, and 3D detection labels.
*   [`src/modules/rosConnector.js`](src/modules/rosConnector.js): Connects to live robots over WebSockets via `roslibjs`.
*   [`src/modules/uiController.js`](src/modules/uiController.js): WebGL drawing in Three.js, Chart.js trend timelines, and compass dials.

---

## 📐 Mathematical Frameworks

### 1. Coordinate Unprojection
Maps a depth coordinate pixel $(u, v)$ with distance $d$ into 3D camera coordinates:
$$X = \frac{(u - c_x) \cdot d}{f_x}$$
$$Y = -\frac{(v - c_y) \cdot d}{f_y}$$
$$Z = d$$

### 2. Discontinuity Filtering
Filters grid triangles spanning depth boundaries:
$$\max(|d_A - d_C|, |d_A - d_B|, |d_C - d_B|) < 0.35\text{ meters}$$

### 3. Chassis Proximity Check
Tracks horizontal Euclidean chassis scraping distance:
$$\text{dist} = \sqrt{x^2 + z^2} \quad \text{for } y \in [-0.85\text{m}, -0.40\text{m}]$$
$$\text{Trigger warning if } \text{dist} < 0.32\text{m} \text{ for } \ge 8\text{ points}$$

---

## 💻 Getting Started

### Prerequisites
*   A browser supporting WebGL (Chrome, Edge, Firefox, or Safari).
*   Python (for hosting the server) or Node.js.

### 1. Start the Local Server
Navigate to the directory and host a web server (needed for ES6 Module imports to bypass CORS boundaries):
```bash
# Using Python
python -m http.server 8000

# OR using Node.js
npx http-server -p 8000
```

### 2. Run the Application
Open your web browser and navigate to:
👉 **[http://localhost:8000](http://localhost:8000)**

### 3. Remote Control Mobile App
1. Enter your computer's local Wi-Fi IP address in the **HOST IP FOR MOBILE PAIRING** panel and click **SET**.
2. Scan the scannable **URL LINK** QR code with your phone. It will open `http://<IP>:8000/mobile.html` on your mobile device.
3. Select **MANUAL** mode to activate the touch joystick, or select **DISINFECT** to activate the UVC light module.

---

## 📈 Report Validation Sample Output
The downloaded reports compile statistics inside structured tables matching standardized tolerance levels:
*   **Average Rate**: Optimal ($\ge 25.0$ Hz)
*   **Centroid Error**: Tolerance bounds ($< 0.25\text{m}$)
*   **Proximity Alert state**: Safe/Alarm tracking

---

## 📄 License
This project is open-source and available under the MIT License.
