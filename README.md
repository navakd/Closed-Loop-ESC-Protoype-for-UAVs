# 🚁 Closed Loop Electronic Speed Controller (ESC) for UAVs

This project implements a prototype **Electronic Speed Controller (ESC)** for Brushless DC Motors (BLDC) using **open-loop and closed-loop control** strategies on an **ATmega328P-based microcontroller (Arduino Nano)**. The system controls a 3-phase BLDC motor using **PWM commutation**, **back-EMF sensing**, and **adaptive filtering** for precise rotor position detection — enabling reliable speed control for UAV and robotics applications.

---

## 📂 Repository Structure
├── Open Loop prototype/
│ ├── ESC_timer_open_loop_arduino_pulseinput_library.ino
│ └── ESC_timer_open_loop_arduino_pinchangeinterrupt.ino
├── Closed Loop Prototype/
│ ├── ESC_Closed_loop.ino
│ └── ESC_Closed_loop_Adaptive_Filtering.ino
├── docs/
│ └── ESC_documentation.pdf
└── README.md


---

## 📷 Circuit Diagrams

### Schematic (Courtesy: ![Sidharth Mohan Nair](https://github.com/sidharthmohannair/OpenESC?tab=readme-ov-file)
![ckt dgm ](https://github.com/sidharthmohannair/OpenESC/blob/main/hardware/schematic_diagram_v1.0.pdf)

**Voltage Divider Feedback Circuit**

![Screenshot 2025-06-18 122652](https://github.com/user-attachments/assets/ffba2734-e6cb-4ae2-96d8-6830f7d48a37)

---

## 🧠 Project Highlights

### 🔁 1. Open Loop Motor Control using Timers
- **Timer1** and **Timer2** configured in **Phase Correct PWM mode** for symmetric waveform generation.
- Commutation is handled using fixed delay-based 6-step sequence.
- Duty cycle of PWM is directly proportional to throttle signal width received via **PulseInput** library.
- Results: Stable PWM generation and basic commutation sequence implementation.
- **Code:** [`ESC_timer_open_loop_arduino_pulseinput_library.ino`](Open%20Loop%20prototype/ESC_timer_open_loop_arduino_pulseinput_library.ino)

### 🟡 2. Pin Change Interrupt-Based Throttle Reading
- Throttle input is read using pin change interrupts instead of polling.
- Improved responsiveness and reduced blocking delays (no `delayMicroseconds()`).
- Sets the foundation for transitioning to closed-loop control.
- **Code:** [`ESC_timer_open_loop_arduino_pinchangeinterrupt.ino`](Open%20Loop%20prototype/ESC_timer_open_loop_arduino_pinchangeinterrupt.ino)

### 🔒 3. Closed Loop Control with Back-EMF Feedback
- Analog Comparator used to detect **Zero Crossing (ZC)** of back-EMF.
- System starts motor in open-loop until sufficient back-EMF is generated, then switches to closed-loop.
- Comparator interrupt triggers next commutation step.
- Real-time throttle mapping and commutation timing control.
- **Code:** [`ESC_Closed_loop.ino`](Closed%20Loop%20Prototype/ESC_Closed_loop.ino)

### 🧹 4. Adaptive Filtering for Back-EMF Noise
- Dynamic ZC confirmation threshold depending on motor speed.
- Adaptive minimum interval between commutations to avoid noise-induced false triggers.
- Moving average buffer for commutation timing to reject transient disturbances.
- **Code:** [`ESC_Closed_loop_Adaptive_Filtering.ino`](Closed%20Loop%20Prototype/ESC_Closed_loop_Adaptive_Filtering.ino)

---

## 📉 Observations & Results

- Successful implementation of open-loop and closed-loop commutation with visual verification via trapezoidal waveforms.
- Adaptive filtering led to reduced **stalls and voltage drops** at high RPM.
- Limitation: The 8-bit timers and ISR latency on ATmega328P caused performance degradation at higher speeds.
- Recommendation: Upgrade to higher-resolution MCUs like **STM32** for better PWM fidelity and computation headroom.

---

## 📘 Documentation

👉 Full detailed documentation with figures, explanations, and implementation logic is available here:  
📄 [`ESC_documentation.pdf`](documentation/ESC_documentation.pdf)

---

## ⚙️ Software and Hardware Tools

- Arduino IDE
- ATmega328P (Arduino Nano, Uno)
- PulseInput library (for reading PWM RC input)
- 3-phase BLDC Motor
- Voltage Divider + Comparator circuit for back-EMF sensing

---

## 🧪 Future Improvements

- Replace comparator with ADC-based back-EMF sampling for more robust detection.
- Implement **FOC (Field-Oriented Control)** using STM32 for smoother control.
- Add current sensing for overload and stall protection.

---


## 📄 License

This project is open-source and available under the [MIT License](LICENSE).

---

*If you find this project useful, please ⭐ the repository and feel free to contribute or suggest improvements!*


