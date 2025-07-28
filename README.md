# HC-SR04 Ultrasonic Distance Sensor

The **HC-SR04** ultrasonic sensor provides non-contact distance measurement from **2 cm to 400 cm** with an accuracy of up to **3 mm**. It is widely used in robotics, obstacle avoidance, and distance measurement applications.

---

## 📌 Features

- **Non-contact distance measurement**
- **Range**: 2 cm – 400 cm
- **Accuracy**: ±3 mm
- **Operating voltage**: 5V
- **Ultrasonic frequency**: 40 kHz
- **Low power consumption**: ~15 mA

---

## 📐 Basic Working Principle

1. The **TRIG** pin is set HIGH for at least **10 μs** to initiate measurement.
2. The module sends **8 ultrasonic pulses at 40 kHz**.
3. It waits for the echo signal to return.
4. The **ECHO** pin goes HIGH for a duration proportional to the round-trip time.

