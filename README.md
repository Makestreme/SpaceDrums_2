# SpaceDrums_2
Version 2.0 of spacedrums
---

#User Guide

This system uses advanced 3D quaternion kinematics, dual-IMU sensor fusion, and direct ESP-NOW wireless transmission to give you a highly responsive, 7-piece drum kit in thin air.

## Phase 1: Software & Hub Setup

1. **Plug in the Hub:** Connect your ESP32-S3 Receiver Hub to your PC via USB.
2. **Verify COM Port:** Note which COM port the Hub is assigned (e.g., `COM3` on Windows or `/dev/ttyUSB0` on Mac/Linux).
3. **Audio Files:** Ensure your drum sample files (`crash.wav`, `snare.wav`, `tom1.wav`, `tom2.wav`, `ride.wav`, `hihat.wav`, `floor_tom.wav`) are in the exact same folder as your Python script.
4. **Start the Engine:** Run `audio_server.py`. You should see the console say:
`Audio Engine Ready. Listening to Receiver Hub on COM_X at 500000 baud...`

---

## Phase 2: Powering On & Quick Boot Calibration

Your drumsticks do not know where "Forward" or "Flat" is until you tell them. Every time you power the sticks on, they perform a **Quick Calibration**.

1. **Get in Position:** Sit in your playing chair. Hold both drumsticks in front of you, pointing straight ahead and completely parallel to the floor (this is your virtual **Snare Drum** location).
2. **Power On:** Turn on the drumsticks. The Status LED will turn **Solid On** for 2 seconds.
3. **Hold Still (Averaging Phase):** The LED will begin to **blink rapidly**. Do not move your hands! The sticks are taking hundreds of micro-measurements to zero out the gyroscopes and set your forward center.
4. **Ready:** After 1.5 seconds, you will feel a **short haptic click**. The LED will switch to a slow "heartbeat" blink (1 flash per second).
*Your kit is now mapped and ready to play!*

---

## Phase 3: Playing the Spatial Drum Kit

The physics engine maps a 7-piece drum kit around your body. To hit a drum, simply swing downward. The system evaluates your *intent* (where you are aiming) just before the stick snaps to a stop.

**The Bottom Row (Aim Flat / Parallel to the floor):**

* **Far Left:** Hi-Hat
* **Center:** Snare Drum
* **Far Right:** Floor Tom

**The Top Row (Aim High / ~25° to 45° angled upward):**

* **Far Left:** Crash Cymbal
* **Center-Left:** Tom 1
* **Center-Right:** Tom 2
* **Far Right:** Ride Cymbal

**Pro-Tip for Hitting:** Play naturally! You do not need to abruptly freeze your arms in mid-air. Swing down like you are hitting a physical drumhead. The harder you swing, the harder the haptic motor will kick and the louder the sound will play (8 distinct velocity zones).

---

## Phase 4: Manual Deep Calibration (The Figure-8)

Over time, or if you move to a room with different magnetic interference, the "Forward" direction (Yaw) might drift. If your Snare drum starts feeling like it is drifting to the left or right, perform a **Deep Calibration**.

*You can do this on one stick at a time, or both simultaneously.*

**Step 1: Trigger the Calibration**

* Give the power button a **short press**.
* The Status LED will turn solid, and the console will print `[FULL CALIBRATION TRIGGERED]`.

**Step 2: The Figure-8 (Magnetometer Mapping)**

* After 2 seconds, you will feel a **LONG haptic pulse**.
* Immediately start waving the drumstick in a large, continuous **"Figure-8" (Infinity ∞) motion** in the air, twisting your wrist as you do it.
* Keep doing this for **15 seconds**. The system is mapping the 3D magnetic field of your room.

**Step 3: Return to Center**

* The LED will turn OFF, and you will feel a short haptic pulse.
* **Stop moving.** Bring the drumstick back to your perfect center **Snare position**. You have 4 seconds to get into position.

**Step 4: The Deep Lock**

* The LED will start blinking rapidly. **Hold perfectly still for 5 seconds.** * The system is calculating the absolute offset between the room's magnetic north and your physical Snare drum position, while re-zeroing the gyros.
* When finished, you will feel a **short haptic click**. The LED will return to the slow heartbeat.
* *Your drift is now permanently corrected and saved to the onboard flash memory!*

---

## Troubleshooting

* **Missed Hits / Dropped Audio:** Ensure the XIAO Receiver Hub has a clear line of sight to your drumsticks. Do not hide the Hub behind a thick metal PC case, as ESP-NOW relies on a clean 2.4GHz RF environment.
* **Hitting Toms triggers the Snare:** You are not lifting the sticks high enough during your wind-up. Make sure you are clearly aiming *upward* (> 25 degrees) when reaching for the Cymbals and Toms.
* **Stick goes totally unresponsive:** Ensure the battery is charged. If the slow heartbeat LED stops blinking, reset the stick.
