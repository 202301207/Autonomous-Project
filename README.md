# SmartNav

SmartNav is an instructional Android project that demonstrates and contrasts two fundamental mobile localization techniques: inertial dead reckoning (PDR) and visual-inertial SLAM. The application allows users to walk down a hallway and see the trajectories from both methods plotted in real-time, providing a clear visualization of sensor drift and the corrective power of SLAM.

This project is designed for students and developers interested in mobile robotics, sensor fusion, and ARCore.

## Requirements

- Android Studio Iguana (or newer)
- Android Gradle Plugin 8.13+
- Kotlin 2.2.0+
- A physical Android phone that:
  - Runs Android 8.0 (API 26) or newer
  - Supports Google ARCore

## Getting Started

1.  Clone this repository and open the folder in Android Studio.
2.  Allow the initial Gradle sync to complete.
3.  Connect an ARCore-capable Android phone with Developer Mode and USB debugging enabled.
4.  Click **Run ▶** in Android Studio and select the connected device.

### Runtime Permissions

The app requests **Camera** access on first launch to power the ARCore SLAM system. This permission is required for the SLAM trajectory to function.

## App Walkthrough

1.  Launch the SmartNav application.
2.  Tap **Start Tracking** to begin sensor data collection.
3.  Walk at a steady pace, for instance, down a hallway.
4.  Observe the two trajectories being drawn on the screen:
    -   **Blue Path**: Dead Reckoning (PDR) from inertial sensors.
    -   **Red Path**: SLAM from ARCore's visual-inertial tracking.
5.  Tap **Stop Tracking** to end the session.
6.  Use the **Save Session** and **Load Last Session** buttons to persist and review trajectories as CSV files.

## Technical Overview

### Dead Reckoning (PDR)

Located in `dr/DeadReckoningEngine.kt`, this module implements a simple Pedestrian Dead Reckoning algorithm:

-   It listens to the device's **linear accelerometer** to detect steps using a thresholded peak-detection filter.
-   It uses the **rotation vector sensor** (via `sensors/OrientationProvider.kt`) to estimate the device's heading (yaw).
-   Each detected step advances the 2D pose according to the classic model:

    ```
    xₖ₊₁ = xₖ + L · cos(θ)
    yₖ₊₁ = yₖ + L · sin(θ)
    ```

    where `L` is an assumed step length (0.75 m). This method is prone to drift as small sensor errors accumulate over time, which is a key learning objective.

### SLAM (ARCore)

The SLAM implementation in `slam/SlamTracker.kt` provides a robust, world-referenced position by leveraging Google's ARCore library via the modern `SceneView` wrapper.

-   It directly uses an `ARSceneView` component, configured to use the latest camera image and to find horizontal and vertical planes.
-   On each frame update, the `onFrame` callback provides the device's current camera pose from ARCore.
-   This 3D pose is projected into a 2D plane `(x, -z)` and reported as a `Pose2D` object.

Because ARCore anchors its position estimate to visual features in the environment, this SLAM trajectory remains far more accurate over time than PDR and serves as a ground-truth comparison.

### Data & Visualization

-   `data/TrajectoryRepository.kt`: Buffers the poses from both engines, computes the instantaneous drift between them, and handles the saving/loading of trajectories to CSV files in the format: `timestamp_ms,dr_x,dr_y,slam_x,slam_y`.
-   `ui/TrajectoryView.kt`: A custom `View` that renders the two paths on a top-down 2D grid, automatically scaling and centering the trajectories.
-   `ui/MainActivity.kt`: The main activity that coordinates the UI, permissions, and the DR/SLAM engines.
