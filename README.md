# IR Entry/Exit Counter

An IoT-enabled people counter that uses infrared break-beam sensors to detect and track individuals moving through a doorway, determines the direction of movement (entry vs. exit), and transmits live occupancy counts to a cloud-hosted web dashboard over cellular connectivity.

Built as a semester project for **GEEN 1400: Engineering Projects** at the **University of Colorado Boulder**.

---

## Overview

The system is designed around a simple but robust detection method: two parallel IR break-beam sensors mounted in a doorway. When a person passes through, the sensors are triggered in sequence — the order of those triggers determines the direction of travel. An onboard ESP32 microcontroller reads the sensors, maintains the running occupancy count, and periodically pushes the count to a backend server over NB-IoT cellular (no Wi-Fi required).

A live web dashboard visualizes the current occupancy in real time.

---

## Hardware

| Component | Purpose |
|---|---|
| ESP32 DevKit V1 | Main microcontroller — sensor logic, serial comms, modem control |
| SIM7000G | NB-IoT / LTE-M cellular module for data uplink |
| IR break-beam sensors (×2) | Doorway detection (entry vs. exit direction) |
| Hall effect sensor | Auxiliary detection input |
| A4403 buck converter | 9V → regulated rails for logic and modem |
| TP4056 | LiPo battery charging circuit |
| 9V battery (×2 in parallel) | Portable power supply (~1.2 Ah capacity) |

The full custom PCB was designed from scratch in **Altium Designer**.

---

## Wiring Diagram

![Wiring Schematic](hardware/wiring_schematic.png)

Key design notes:
- The Hall effect sensor's 5V signal is dropped to 3.3V via a voltage divider before reaching the ESP32 GPIO
- The ESP32 TX line to the SIM7000G also passes through a divider to match logic levels
- Bypass capacitors are placed near the SIM7000G to handle its ~2A current spikes
- Two 9V batteries are connected in parallel to double the available capacity

---

## Firmware

The ESP32 firmware (`firmware/people_counter.ino`) is written using the Arduino framework in C++. It handles:

- Sensor polling and debounced edge detection
- Direction inference based on which sensor is broken first
- SIM7000G initialization and NB-IoT network registration (APN: `america.bics`)
- HTTP GET requests to the backend with the latest occupancy count
- Automatic recovery: if the modem or network fails, the ESP32 restarts and retries

Occupancy updates are sent every 10 seconds while the system is running.

---

## Team

Built for GEEN 1400 at CU Boulder by:
- Yonas Wassen
- Abel Amnesisa
- Suhit Mogadati
- Fischer Woods
- Jackson Kadlec
- Ubaldo Landa Posas

---

## License

Open for educational reference. If you build on this for your own class project, please credit the team.
