# Aria: Wearable Respiratory Monitoring System

Aria is a proof-of-concept wearable device designed for real-time respiratory tracking, posture classification, and tactile intervention. Built with a focus on Apnea and hyperventilation awareness, the system utilizes a pneumatic pressure-sensing belt and an Arduino micro-controller to monitor breathing patterns, detect anomalies (apnea and hyperventilation events), and trigger haptic feedback as a form of tactile arousal therapy.

> **⚠️ Safety Disclaimer:** This device is intended for personal health awareness and prototyping purposes only. It is **not** a certified diagnostic medical instrument or a replacement for clinical CPAP therapy.

## ✨ Key Features
* **Pneumatic Respiratory Sensing:** Tracks chest-wall displacement to extract real-time Breaths Per Minute (BPM) and respiratory waveforms.
* **IMU Posture Classification:** Automatically detects user orientation (Upright, Supine, Walking) to contextualize respiratory data.
* **Tactile Arousal Intervention:** Haptic vibration motor triggers during detected anomalous events to stimulate micro-arousals and restore airway neuromuscular tone.
* **Capacitive Gesture Control:** Supports double-pinch and long-pinch interactions for local state control.
* **Real-Time Telemetry:** Streams live data to through a localized Web Dashboard over Wi-Fi.

## 📊 Operational Constraints & Performance
Due to the physical design of the pneumatic sensing mechanism, system accuracy is highly dependent on the user's kinetic state and posture.

| Activity / State | Primary Error Source 
| :--- | :--- | :--- |
| **Standing (Deep/Normal)** | Negligible baseline drift |
| **Supine (Deep/Normal)** | None (High Signal-to-Noise Ratio) |
| **Shallow Breathing** | Low mechanical signal amplitude |
| **Slow Walking** | Motion artifacts & baseline shifting |
| **Non-Supine Lying** | Mechanical sensor decoupling (No Estimation Possible) |

* **Anatomical Constraint:** Monitoring in a lying position is strictly limited to the **supine (face-up)** posture. Prone or lateral positions compress the belt, resulting in signal loss.
* **Proximity Constraint:** The user’s designated host device (PC, laptop, or smartphone) must remain within the effective Wi-Fi range of the microcontroller.
